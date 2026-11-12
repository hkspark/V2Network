from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

BASE = Path(__file__).resolve().parent.parent
RESULTS = BASE / "results"

files = list(RESULTS.glob("*_run_*.csv"))
#files = [
#    "01_dense_ecdsa_like.csv",
#    "02_dense_pqc_like.csv",
#    "03_gridlock_attack_pqc_like.csv",
#    "04_severe_gridlock_attack_pqc_like.csv"
#]

exp_type = {}
for file in files:
    exp_name = file.stem.split("_run_")[0]
    exp_type.setdefault(exp_name, []).append(file)

for exp, file_list in exp_type.items():
    #df = pd.read_csv(RESULTS / file)
    #label = file.replace(".csv", "")
    dfs = [pd.read_csv(f) for f in file_list]
    combined = pd.concat(dfs)
    avg_df = combined.groupby("time").mean().reset_index()

    plt.figure()
    plt.plot(avg_df["time"], avg_df["queue_length"], label="Mean Queue Length")
    plt.xlabel("Simulation time (s)")
    plt.ylabel("Queue length")
    plt.title(f"Average Verification queue length: {exp}")
    plt.savefig(RESULTS / f"{exp}_avg_queue.png", dpi=300)
    plt.close()

    plt.figure()
    plt.plot(avg_df["time"], avg_df["arrivals"], label = "Mean Arrivals")
    plt.axhline(avg_df["verify_capacity_k"].iloc[0], linestyle="--", label="Capacity K")
    plt.xlabel("Simulation time (s)")
    plt.ylabel("BSM arrivals per step")
    plt.title(f"Avg N vs K: {exp}")
    plt.savefig(RESULTS / f"{exp}_avg_n_vs_k.png", dpi=300)
    plt.close()

#summary = pd.read_csv(RESULTS / "summary.csv")
summaries = pd.read_csv(RESULTS / "summary.csv")
summaries["experiment_name"] = summaries["experiment"].apply(lambda x: x.split("_run_")[0])
group_summary = summaries.groupby("experiment_name").agg(
    mean_miss_rate=("critical_miss_rate_percent", "mean"),
    std_miss_rate=("critical_miss_rate_percent", "std")
).reset_index()

plt.figure()
#plt.bar(summary["experiment"], summary["critical_miss_rate_percent"])
plt.bar(
    group_summary["experiment_name"],
    group_summary["mean_miss_rate"],
    yerr = group_summary["std_miss_rate"],
    capsize = 5,
    color = "skyblue",
    edgecolor = "black"
)
plt.xticks(rotation=45, ha="right")
plt.ylabel("Mean Critical Miss Rate (%)")
plt.title("Critical BSM miss rate under FCFS (Mean + Std Dev)")
plt.tight_layout()
plt.savefig(RESULTS / "critical_miss_rate_comparison.png", dpi=300)
plt.close()

print("Plots saved in results/")