#!/usr/bin/env python3
"""Render an existing read-only prefix comparison using system matplotlib."""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    output = args.report.parent / "ramp23_prefix_comparison.png"
    if output.exists():
        raise FileExistsError(output)
    fig, axes = plt.subplots(2, 3, figsize=(13, 6), constrained_layout=True)
    for row, case in enumerate(report["cases"]):
        name = case["direction"]
        sign = -1 if name == "up" else 1
        isaac = json.loads((Path(case["Isaac_CPU"]["run"]) / f"ramp23_{name}_prefix_trace.json").read_text())
        gazebo = [json.loads(line) for line in (Path(case["Gazebo"]["run"]) / "telemetry.jsonl").read_text().splitlines() if line.strip()]
        for column, (field, title) in enumerate([
            ("progress", "World forward progress (m)"),
            ("lateral", "Lateral offset from ramp center (m)"),
            ("clearance", "Body support clearance (m)"),
        ]):
            ax = axes[row, column]
            for rows, simulator, color in [(isaac, "Isaac CPU prefix", "#2563eb"), (gazebo, "Gazebo actual", "#dc2626")]:
                t = np.array([r["t"] if simulator.startswith("Isaac") else r["sim_time"] for r in rows])
                pose = np.array([r["position"] for r in rows])
                if field == "progress":
                    values = sign * (pose[:, 0] - pose[0, 0])
                elif field == "lateral":
                    values = pose[:, 1] - 7
                else:
                    key = "clearance" if simulator.startswith("Isaac") else "body_clearance"
                    values = np.array([r[key] for r in rows])
                ax.plot(t, values, label=simulator, color=color, lw=1.2)
            ax.axvline(3, color="gray", lw=.7, ls=":")
            if field == "clearance":
                ax.axhline(.18, color="gray", lw=.8, ls="--", label="Gazebo full-protocol 0.18m")
                ax.axhline(.16, color="#6b7280", lw=.6, ls=":", label="Isaac diagnostic 0.16m")
            ax.set_title(name.upper() + " · " + title)
            ax.set_xlim(0, 30)
            ax.grid(alpha=.2)
            ax.set_xlabel("Logical test time (s)")
            ax.legend(fontsize=7)
    fig.suptitle("Frozen Teacher: actual ramp23 motion prefixes, different nominal dynamics\nFinite prefix survival does not prove full traversal or stopping", fontsize=12)
    fig.savefig(output, dpi=150)
    plt.close(fig)
    receipt = {"plot": str(output.resolve()), "plot_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
               "report_sha256": hashlib.sha256(args.report.read_bytes()).hexdigest(),
               "plotter_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               "scope": "Plots existing real traces only; no model evaluation, ROS or simulation"}
    (args.report.parent / "plot_receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt))


if __name__ == "__main__":
    main()
