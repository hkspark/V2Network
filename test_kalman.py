import os
import sys
import math
import argparse
from pathlib import Path
from collections import deque
import heapq

import traci
import pandas as pd


BASE = Path(__file__).resolve().parent.parent
MAPS = BASE / "maps"
RESULTS = BASE / "results"
RESULTS.mkdir(exist_ok=True)


STEP_LENGTH = 0.1
BSM_INTERVAL = 0.1
DEADLINE = 0.15
SIM_END = 300.0
BSM_RESIDUAL_THRESHOLD = 5.0



class SimpleKalmanTracker:
    """Tracks sender vehicle dynamics (x, y, speed) to predict future states

    and compute state residuals for spoofing/attack detection.
    """

    def __init__(self, initial_x, initial_y, initial_speed, dt=0.1):
        self.dt = dt
        # State vector: [x, y, vx, vy] (assuming moving along x-axis vector)
        self.x = initial_x
        self.y = initial_y
        self.vx = initial_speed
        self.vy = 0.0

        # Simple covariance / variance estimates
        self.p_var = 1.0

    def predict(self, dt=0.1):
        # Motion prediction step
        pred_x = self.x + self.vx * dt
        pred_y = self.y + self.vy * dt
        return pred_x, pred_y

    def update(self, meas_x, meas_y, meas_speed):
        # Predict current step state
        pred_x, pred_y = self.predict(self.dt)

        # Innovation / Residual calculation (euclidean distance difference)
        residual = math.sqrt((meas_x - pred_x) ** 2 + (meas_y - pred_y) ** 2)

        # Kalman state update
        alpha = 0.6  # Filter gain factor
        self.x = pred_x + alpha * (meas_x - pred_x)
        self.y = pred_y + alpha * (meas_y - pred_y)
        self.vx = meas_speed  # Speed updated directly

        return residual


def compute_kalman_priority(ego_state, sender_state, residual):
    """Calculates a priority score based on Kalman prediction metrics,

    Time-to-Collision (TTC), and residual consistency.
    """
    ego_x, ego_y, ego_speed = ego_state
    x, y, speed = sender_state

    distance = abs(x - ego_x)
    relative_speed = max(speed - ego_speed, 0.1)
    ttc = distance / relative_speed

    # 1. Base kinematic criticality
    is_crit = distance < 60 or ttc < 3.0

    # 2. Risk Score Formulation (Lower score = Higher Priority in Min-Heap)
    # High residual implies anomalous/spoofed trajectory behavior -> penalize priority score
    residual_penalty = residual * 10.0

    # High risk (low TTC / close distance) yields smaller numerical score (higher priority)
    risk_score = ttc + (distance / 50.0) + residual_penalty

    return is_crit, risk_score


def run_experiment(name, verify_capacity_k, attack_rate):
    sumo_binary = "sumo"
    config = str(MAPS / "config.sumocfg")

    traci.start(
        [
            sumo_binary,
            "-c",
            config,
            "--step-length",
            str(STEP_LENGTH),
            "--no-warnings",
            "true",
        ]
    )

    # Priority queue stores items as: (priority_score, insertion_counter, msg)
    priority_queue = []
    trackers = {}  # Map of sender_id -> SimpleKalmanTracker
    logs = []

    total_verified = 0
    total_expired = 0
    total_critical_arrivals = 0
    total_critical_missed = 0
    total_arrivals = 0
    counter = 0
    step = 0
    total_forged_bsms = 0
    total_forged_bsms_detected = 0
    total_false_alarms = 0
    total_legitimate_bsms = 0

    while traci.simulation.getTime() < SIM_END:
        traci.simulationStep()
        now = traci.simulation.getTime()
        veh_ids = traci.vehicle.getIDList()

        if len(veh_ids) == 0:
            continue

        ego_id = veh_ids[0]
        ego_pos = traci.vehicle.getPosition(ego_id)
        ego_speed = traci.vehicle.getSpeed(ego_id)
        ego_state = (ego_pos[0], ego_pos[1], ego_speed)

        arrivals_this_step = 0
        critical_arrivals_this_step = 0
        forged_detected_this_step = 0
        false_alarms_this_step = 0

        # Process Legitimate Messages
        for vid in veh_ids:
            if vid == ego_id:
                continue

            pos = traci.vehicle.getPosition(vid)
            speed = traci.vehicle.getSpeed(vid)
            sender_state = (pos[0], pos[1], speed)

            # Initialize or update Kalman Tracker for sender
            if vid not in trackers:
                trackers[vid] = SimpleKalmanTracker(pos[0], pos[1], speed)
                residual = 0.0
            else:
                residual = trackers[vid].update(pos[0], pos[1], speed)

            critical, priority_score = compute_kalman_priority(
                ego_state, sender_state, residual
            )
            detected = residual> BSM_RESIDUAL_THRESHOLD
            
            total_legitimate_bsms += 1
            
            if detected:
                total_false_alarms += 1
                false_alarms_this_step += 1

            msg = {
                "arrival_time": now,
                "sender": vid,
                "critical": critical,
                "fake": False,
                "residual": residual,
                "detected": detected
            }

            heapq.heappush(priority_queue, (priority_score, counter, msg))
            counter += 1
            arrivals_this_step += 1
            total_arrivals += 1

            if critical:
                critical_arrivals_this_step += 1
                total_critical_arrivals += 1

        # Process Fake/Attacker Messages (Attacker attempts spoofing with arbitrary values)
        for i in range(attack_rate):
            fake_sender = f"attacker_{step}_{i}"
            fake_pos = (
                ego_pos[0] + 10.0 + (i % 5) * 10,
                ego_pos[1],
            )  # Fake proximity
            fake_speed = 35.0

            # Unregistered or chaotic trajectory creates artificial residual penalty
            residual = 15.0  # Simulated high trajectory discrepancy
            detected = residual > BSM_RESIDUAL_THRESHOLD
            total_forged_bsms += 1
            
            if detected:
                total_forged_bsms_detected += 1
                forged_detected_this_step += 1

            critical, priority_score = compute_kalman_priority(
                ego_state, (fake_pos[0], fake_pos[1], fake_speed), residual
            )

            fake_msg = {
                "arrival_time": now,
                "sender": fake_sender,
                "critical": False,  # Label truth: Fake message
                "fake": True,
                "residual": residual,
                "detected": detected
            }

            heapq.heappush(priority_queue, (priority_score, counter, fake_msg))
            counter += 1
            arrivals_this_step += 1
            total_arrivals += 1

        verified_this_step = 0
        expired_this_step = 0
        critical_missed_this_step = 0
        delays = []

        # Filter out expired messages (> DEADLINE)
        valid_queue = []
        while priority_queue:
            score, cnt, msg = heapq.heappop(priority_queue)
            age = now - msg["arrival_time"]

            if age > DEADLINE:
                total_expired += 1
                expired_this_step += 1

                if msg["critical"]:
                    total_critical_missed += 1
                    critical_missed_this_step += 1
            else:
                heapq.heappush(valid_queue, (score, cnt, msg))

        priority_queue = valid_queue

        # Cryptographic Verification Step: Verify up to Capacity K highest-priority messages
        while priority_queue and verified_this_step < verify_capacity_k:
            score, cnt, msg = heapq.heappop(priority_queue)
            delay = now - msg["arrival_time"]

            verified_this_step += 1
            total_verified += 1
            delays.append(delay)

        mean_delay = sum(delays) / len(delays) if delays else 0

        logs.append(
            {
                "time": now,
                "vehicles": len(veh_ids),
                "arrivals": arrivals_this_step,
                "verify_capacity_k": verify_capacity_k,
                "attack_rate": attack_rate,
                "verified": verified_this_step,
                "expired": expired_this_step,
                "queue_length": len(priority_queue),
                "critical_arrivals": critical_arrivals_this_step,
                "critical_missed": critical_missed_this_step,
                "mean_delay": mean_delay,
                "n_greater_than_k": arrivals_this_step > verify_capacity_k,
                "forged_bsms_detected": forged_detected_this_step,
                "false_alarms": false_alarms_this_step
            }
        )

        step += 1

    traci.close()

    df = pd.DataFrame(logs)
    out_file = RESULTS / f"{name}_kalman.csv"
    df.to_csv(out_file, index=False)

    critical_miss_rate = (
        total_critical_missed / total_critical_arrivals
        if total_critical_arrivals > 0
        else 0
    )

    summary = {
        "experiment": f"{name}_kalman",
        "verify_capacity_k": verify_capacity_k,
        "attack_rate": attack_rate,
        "total_arrivals": total_arrivals,
        "total_verified": total_verified,
        "total_expired": total_expired,
        "total_critical_arrivals": total_critical_arrivals,
        "total_critical_missed": total_critical_missed,
        "critical_miss_rate_percent": critical_miss_rate * 100,
        "mean_queue_length": df["queue_length"].mean(),
        "max_queue_length": df["queue_length"].max(),
        "percent_steps_N_greater_K": df["n_greater_than_k"].mean() * 100,
        "total_forged_bsms": total_forged_bsms,
        "forged_bsms_detected": total_forged_bsms_detected,
        "forged_detection_rate_percent": (total_forged_bsms_detected / total_forged_bsms * 100 if total_forged_bsms > 0 else 0),
        "false_alarms": total_false_alarms,
        "total_legitimate_bsms": total_legitimate_bsms,
        "false_alarm_rate_percent": ( total_false_alarms / total_legitimate_bsms * 100 if total_legitimate_bsms > 0 else 0)
    }

    return summary


def main():
    experiments = [
        {
            "name": "01_dense_ecdsa_like",
            "verify_capacity_k": 10,
            "attack_rate": 0,
        },
        {
            "name": "02_dense_pqc_like",
            "verify_capacity_k": 2,
            "attack_rate": 0,
        },
        {
            "name": "03_gridlock_attack_pqc_like",
            "verify_capacity_k": 2,
            "attack_rate": 50,
        },
        {
            "name": "04_severe_gridlock_attack_pqc_like",
            "verify_capacity_k": 2,
            "attack_rate": 150,
        },
    ]

    summaries = []

    for exp in experiments:
        print(f"Running Kalman Policy Experiment: {exp['name']}...")
        summary = run_experiment(
            name=exp["name"],
            verify_capacity_k=exp["verify_capacity_k"],
            attack_rate=exp["attack_rate"],
        )
        summaries.append(summary)
        print(summary)

    summary_df = pd.DataFrame(summaries)
    summary_df.to_csv(RESULTS / "kalman_summary.csv", index=False)

    print("\nKalman experiments finished. Results saved in results/")


if __name__ == "__main__":
    main()
