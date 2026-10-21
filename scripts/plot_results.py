from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

BASE = Path(__file__).resolve().parent.parent
RESULTS = BASE / "results"

files = [
    "01_dense_ecdsa_like.csv",
    "02_dense_pqc_like.csv",
    "03_gridlock_attack_pqc_like.csv",
    "04_severe_gridlock_attack_pqc_like.csv"
]

for file in files:
    df = pd.read_csv(RESULTS / file)
    label = file.replace(".csv", "")

    plt.figure()
    plt.plot(df["time"], df["queue_length"])
    plt.xlabel("Simulation time (s)")
    plt.ylabel("Queue length")
    plt.title(f"Verification queue length: {label}")
    plt.savefig(RESULTS / f"{label}_queue.png", dpi=300)
    plt.close()

    plt.figure()
    plt.plot(df["time"], df["arrivals"])
    plt.axhline(df["verify_capacity_k"].iloc[0], linestyle="--")
    plt.xlabel("Simulation time (s)")
    plt.ylabel("BSM arrivals per step")
    plt.title(f"N vs K: {label}")
    plt.savefig(RESULTS / f"{label}_n_vs_k.png", dpi=300)
    plt.close()

summary = pd.read_csv(RESULTS / "summary.csv")

plt.figure()
plt.bar(summary["experiment"], summary["critical_miss_rate_percent"])
plt.xticks(rotation=45, ha="right")
plt.ylabel("Critical miss rate (%)")
plt.title("Critical BSM miss rate under FCFS")
plt.tight_layout()
plt.savefig(RESULTS / "critical_miss_rate_comparison.png", dpi=300)
plt.close()

print("Plots saved in results/")