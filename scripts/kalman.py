import os
import sys
import math
import argparse
from pathlib import Path
from collections import deque
import heapq

import traci
import pandas as pd
import numpy as np


BASE = Path(__file__).resolve().parent.parent
MAPS = BASE / "maps"
RESULTS = BASE / "results"
RESULTS.mkdir(exist_ok=True)


STEP_LENGTH = 0.1
DEADLINE = 0.15
SIM_END = 300.0
BSM_RESIDUAL_THRESHOLD = 20.0


class SimpleKalmanTracker:
    """
    Constant-velocity Kalman filter.

    State: [x, y, vx, vy]
    Measurement: [x, y, vx, vy]
    """

    def __init__(
        self,
        initial_x,
        initial_y,
        initial_vx,
        initial_vy,
        dt=0.1
    ):
        self.dt = dt

        self.x = np.array([
            initial_x,
            initial_y,
            initial_vx,
            initial_vy
        ], dtype=float)

        self.P = np.diag([4.0, 4.0, 9.0, 9.0])

        # Measurement noise covariance
        self.R = np.diag([
            2.5 ** 2,
            2.5 ** 2,
            1.5 ** 2,
            1.5 ** 2
        ])

        # Acceleration process-noise standard deviation
        self.accel_std = 3.0

    def predict(self, dt=None):
        if dt is None:
            dt = self.dt

        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=float)

        q = self.accel_std ** 2

        q_block = np.array([
            [dt ** 4 / 4, dt ** 3 / 2],
            [dt ** 3 / 2, dt ** 2]
        ]) * q

        self.Q = np.zeros((4, 4))

        # Position/velocity noise for x
        self.Q[np.ix_([0, 2], [0, 2])] = q_block

        # Position/velocity noise for y
        self.Q[np.ix_([1, 3], [1, 3])] = q_block

        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        return self.x.copy()

    def update(self, meas_x, meas_y, meas_vx, meas_vy):
        """
        Predict, calculate innovation and NIS, then update only
        if the measurement passes the statistical gate.
        """
        predicted_state = self.predict()

        z = np.array([
            meas_x,
            meas_y,
            meas_vx,
            meas_vy
        ], dtype=float)

        H = np.eye(4)

        innovation = z - (H @ predicted_state)

        S = H @ self.P @ H.T + self.R

        # NIS: innovation.T * inverse(S) * innovation
        nis = float(
            innovation.T @ np.linalg.solve(S, innovation)
        )

        position_residual = float(
            np.linalg.norm(innovation[:2])
        )

        # 4 degrees of freedom, approximately 99% confidence
        nis_threshold = 13.2767

        suspicious = nis > BSM_RESIDUAL_THRESHOLD

        if not suspicious:
            K = self.P @ H.T @ np.linalg.inv(S)

            self.x = predicted_state + K @ innovation

            I = np.eye(4)

            # Joseph form covariance update for numerical stability
            self.P = (
                (I - K @ H) @ self.P @ (I - K @ H).T
                + K @ self.R @ K.T
            )

        return {
            "nis": nis,
            "nis_threshold": nis_threshold,
            "position_residual": position_residual,
            "suspicious": suspicious,
            "state": self.x.copy()
        }


def compute_kalman_priority(residual, nis):
    """
    Pure Kalman-based priority without TTC or kinematic distance.
    Lower score = higher priority in the min-heap.
    """
    # Flag as critical if position residual exceeds threshold (or NIS > 95% chi-square)
    is_crit = residual > BSM_RESIDUAL_THRESHOLD

    # Pure Kalman score based on innovation residual and NIS
    risk_score = (residual * 10.0) + nis
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
    message_logs = []

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

    true_positives = 0
    false_positives = 0
    true_negatives = 0
    false_negatives = 0

    while traci.simulation.getTime() < SIM_END:
        traci.simulationStep()
        now = traci.simulation.getTime()
        veh_ids = traci.vehicle.getIDList()

        if len(veh_ids) == 0:
            step += 1
            continue

        ego_id = veh_ids[0]

        arrivals_this_step = 0
        critical_arrivals_this_step = 0
        forged_detected_this_step = 0
        false_alarms_this_step = 0

        # Select existing senders to spoof.
        # Do not attack during the first step; trackers need an initial observation.
        sender_ids = [
            vid for vid in veh_ids
            if vid != ego_id and vid in trackers
        ]

        attack_count = min(attack_rate, len(sender_ids))

        if step == 0 or not sender_ids:
            attacked_ids = set()
        else:
            start_index = (step * attack_count) % len(sender_ids)

            attacked_ids = {
                sender_ids[(start_index + i) % len(sender_ids)]
                for i in range(attack_count)
            }

        # Process each sender's BSM.
        for vid in veh_ids:
            if vid == ego_id:
                continue

            pos = traci.vehicle.getPosition(vid)
            speed = traci.vehicle.getSpeed(vid)
            angle = math.radians(traci.vehicle.getAngle(vid))

            vx = speed * math.sin(angle)
            vy = speed * math.cos(angle)

            # Copy actual state into reported BSM values.
            reported_x = pos[0]
            reported_y = pos[1]
            reported_vx = vx
            reported_vy = vy

            is_forged = vid in attacked_ids

            # Apply simulated forgery.
            if is_forged:
                reported_x += 30.0
                reported_y += 25.0
                reported_vx += 8.0
                reported_vy -= 6.0

            # Initialize tracker using the actual vehicle state.
            # First observation cannot be evaluated yet.
            if vid not in trackers:
                trackers[vid] = SimpleKalmanTracker(
                    pos[0],
                    pos[1],
                    vx,
                    vy,
                    dt=STEP_LENGTH
                )

                message_logs.append({
                    "time": now,
                    "sender": vid,
                    "fake": is_forged,
                    "classification": "not_evaluated",
                    "nis": None,
                    "residual": None,
                    "detected": False
                })

                continue

            # Evaluate the reported BSM using the Kalman tracker.
            result = trackers[vid].update(
                reported_x,
                reported_y,
                reported_vx,
                reported_vy
            )

            nis = result["nis"]
            residual = result["position_residual"]
            detected = result["suspicious"]

            # Update classification counters.
            if is_forged:
                total_forged_bsms += 1

                if detected:
                    true_positives += 1
                    total_forged_bsms_detected += 1
                    forged_detected_this_step += 1
                    classification = "true_positive"
                else:
                    false_negatives += 1
                    classification = "false_negative"

            else:
                total_legitimate_bsms += 1

                if detected:
                    false_positives += 1
                    total_false_alarms += 1
                    false_alarms_this_step += 1
                    classification = "false_positive"
                else:
                    true_negatives += 1
                    classification = "true_negative"

            # Record individual BSM evaluation.
            message_logs.append({
                "time": now,
                "sender": vid,
                "fake": is_forged,
                "classification": classification,
                "nis": nis,
                "residual": residual,
                "detected": detected
            })

            # Reject suspicious BSMs before queue insertion.
            if detected:
                continue

            # Calculate pure Kalman verification priority.
            critical, priority_score = compute_kalman_priority(
                residual,
                nis
            )

            msg = {
                "arrival_time": now,
                "sender": vid,
                "critical": critical,
                "fake": is_forged,
                "residual": residual,
                "nis": nis,
                "detected": detected
            }

            heapq.heappush(
                priority_queue,
                (priority_score, counter, msg)
            )

            counter += 1
            arrivals_this_step += 1
            total_arrivals += 1

            if critical:
                critical_arrivals_this_step += 1
                total_critical_arrivals += 1

        # -------------------------------------------------
        # Queue processing happens ONCE per simulation step.
        # It must remain outside the vehicle-processing loop.
        # -------------------------------------------------

        verified_this_step = 0
        expired_this_step = 0
        critical_missed_this_step = 0
        delays = []

        # Remove expired messages.
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
                heapq.heappush(
                    valid_queue,
                    (score, cnt, msg)
                )

        priority_queue = valid_queue

        # Verify up to K messages per simulation step.
        while priority_queue and verified_this_step < verify_capacity_k:
            score, cnt, msg = heapq.heappop(priority_queue)

            delay = now - msg["arrival_time"]

            verified_this_step += 1
            total_verified += 1
            delays.append(delay)

        mean_delay = (
            sum(delays) / len(delays)
            if delays else 0
        )

        # Record simulation-step metrics.
        logs.append({
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
        })
        step += 1

    traci.close()

    # Save simulation-step results.
    df = pd.DataFrame(logs)
    out_file = RESULTS / f"{name}_kalman.csv"
    df.to_csv(out_file, index=False)

    # Save individual BSM results.
    message_df = pd.DataFrame(message_logs)
    message_file = RESULTS / f"{name}_messages.csv"
    message_df.to_csv(message_file, index=False)

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
        "forged_detection_rate_percent": (
            total_forged_bsms_detected / total_forged_bsms * 100
            if total_forged_bsms > 0 else 0
        ),
        "false_alarms": total_false_alarms,
        "total_legitimate_bsms": total_legitimate_bsms,
        "false_alarm_rate_percent": (
            total_false_alarms / total_legitimate_bsms * 100
            if total_legitimate_bsms > 0 else 0
        ),
        "true_positives": true_positives,
        "false_positives": false_positives,
        "true_negatives": true_negatives,
        "false_negatives": false_negatives
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
