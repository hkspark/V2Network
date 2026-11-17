import os
import sys
import math
import argparse
import random
from pathlib import Path
from collections import deque

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

#Function used to calculate TTC, will return distance, ttc(999 if vehicles will never intersect) and if critical (ttc under 3 seconds)
def get_ttc(ego_state, sender_state):
    ego_x, ego_y, ego_speed, ego_angle = ego_state
    x, y, speed, angle = sender_state
    dx = x - ego_x
    dy = y -ego_y

    distance = math.hypot(dx, dy)
    if distance < 0.01:
        return 0.0, 0.0, True

    ego_rad = math.radians(ego_angle)
    ego_vx = ego_speed * math.sin(ego_rad)
    ego_vy = ego_speed * math.cos(ego_rad)

    sender_rad = math.radians(angle)
    vx = speed * math.sin(sender_rad)
    vy = speed * math.cos(sender_rad)

    closing_speed = -((dx * (vx - ego_vx)) + (dy * (vy - ego_vy))) / distance

    if closing_speed > 0.1:
        ttc = distance / closing_speed
    else:
        ttc = 999.0
    critical = (distance < 60.0) or (ttc < 3.0)
    return distance, ttc, critical


def run_experiment(name, verify_capacity_k, attack_rate):
    sumo_binary = "sumo"
    config = str(MAPS / "config.sumocfg")

    traci.start([
        sumo_binary,
        "-c", config,
        "--step-length", str(STEP_LENGTH),
        "--no-warnings", "true"
    ])

    queue = []
    logs = []

    total_verified = 0
    total_expired = 0
    total_critical_arrivals = 0
    total_critical_missed = 0
    total_arrivals = 0

    step = 0

    while traci.simulation.getTime() < SIM_END:
        traci.simulationStep()
        now = traci.simulation.getTime()
        veh_ids = traci.vehicle.getIDList()

        if len(veh_ids) == 0:
            continue

        ego_id = veh_ids[0]
        ego_pos = traci.vehicle.getPosition(ego_id)
        ego_speed = traci.vehicle.getSpeed(ego_id)
        ego_angle = traci.vehicle.getAngle(ego_id)
        ego_state = (ego_pos[0], ego_pos[1], ego_speed, ego_angle)

        step_arrivals = []
        #arrivals_this_step = 0
        critical_arrivals_this_step = 0

        for vid in veh_ids:
            if vid == ego_id:
                continue

            pos = traci.vehicle.getPosition(vid)
            speed = traci.vehicle.getSpeed(vid)
            angle = traci.vehicle.getAngle(vid)
            sender_state = (pos[0], pos[1], speed, angle)

            distance, ttc, critical = get_ttc(ego_state, sender_state)

            step_arrivals.append ({
                "arrival_time": now,
                "sender": vid,
                "distance": distance,
                "ttc": ttc,
                "critical": critical,
                "fake": False
            })

            #queue.append(msg)
            #arrivals_this_step += 1
            #total_arrivals += 1

            if critical:
                critical_arrivals_this_step += 1
                total_critical_arrivals += 1

        for i in range(attack_rate):
            fake_msg = {
                "arrival_time": now,
                "sender": f"attacker_{step}_{i}",
                "distance": 999.0,
                "ttc": 999.0,
                "critical": False,
                "fake": True
            }
            step_arrivals.append(fake_msg)
           #arrivals_this_step += 1
           #total_arrivals += 1

        #randomizes the order of BSMs this step and adds them to the queue
        random.shuffle(step_arrivals)
        queue.extend(step_arrivals)

        arrivals_this_step = len(step_arrivals)
        total_arrivals += arrivals_this_step

        expired_this_step = 0
        critical_missed_this_step = 0
        
        new_queue = []

        for msg in queue:
            age = now - msg["arrival_time"]

            if age > DEADLINE:
                total_expired += 1
                expired_this_step += 1

                if msg["critical"]:
                    total_critical_missed += 1
                    critical_missed_this_step += 1
            else:
                new_queue.append(msg)

        queue = new_queue
        #sort queue by ttc
        queue.sort(key=lambda msg: (msg["ttc"], msg["distance"]))
        
        verified_this_step = 0
        delays = []

        while queue and verified_this_step < verify_capacity_k:
            msg = queue.pop(0)
            delay = now - msg["arrival_time"]

            verified_this_step += 1
            total_verified += 1
            delays.append(delay)

        mean_delay = sum(delays) / len(delays) if delays else 0

        logs.append({
            "time": now,
            "vehicles": len(veh_ids),
            "arrivals": arrivals_this_step,
            "verify_capacity_k": verify_capacity_k,
            "attack_rate": attack_rate,
            "verified": verified_this_step,
            "expired": expired_this_step,
            "queue_length": len(queue),
            "critical_arrivals": critical_arrivals_this_step,
            "critical_missed": critical_missed_this_step,
            "mean_delay": mean_delay,
            "n_greater_than_k": arrivals_this_step > verify_capacity_k
        })

        step += 1

    traci.close()

    df = pd.DataFrame(logs)
    out_file = RESULTS / f"{name}.csv"
    df.to_csv(out_file, index=False)

    critical_miss_rate = (
        total_critical_missed / total_critical_arrivals
        if total_critical_arrivals > 0 else 0
    )

    summary = {
        "experiment": name,
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
        "percent_steps_N_greater_K": df["n_greater_than_k"].mean() * 100
    }

    return summary


def main():
    experiments = [
        {
            "name": "01_ttc_dense_ecdsa_like",
            "verify_capacity_k": 10,
            "attack_rate": 0
        },
        {
            "name": "02_ttc_dense_pqc_like",
            "verify_capacity_k": 2,
            "attack_rate": 0
        },
        {
            "name": "03_ttc_gridlock_attack_pqc_like",
            "verify_capacity_k": 2,
            "attack_rate": 50
        },
        {
            "name": "04_ttc_severe_gridlock_attack_pqc_like",
            "verify_capacity_k": 2,
            "attack_rate": 150
        }
    ]
    #Will run the simulation as many times as SIM_RUNS
    SIM_RUNS = 5
    summaries = []

    for exp in experiments:
        for run in range(SIM_RUNS):
            run_id = f"{exp['name']}_run_{run}"
            print(f"Running {run_id}...")
            summary = run_experiment(
                name= run_id,
                verify_capacity_k=exp["verify_capacity_k"],
                attack_rate=exp["attack_rate"]
            )
            summaries.append(summary)
            print(summary)

    summary_df = pd.DataFrame(summaries)
    summary_df.to_csv(RESULTS / "summary.csv", index=False)

    print("\nFinished. Results saved in results/")


if __name__ == "__main__":
    main()