import argparse
import glob
import os
import sys
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

''' 
python tests/plot_haptic.py log/              # both arms
python tests/plot_haptic.py log/ --side right # right only
python tests/plot_haptic.py log/ --side left --save fig.png
'''

def load(log_dir, side):
    candidates = {
        "left":  os.path.join(log_dir, "arm_left_control.csv"),
        "right": os.path.join(log_dir, "arm_right_control.csv"),
    }
    sides = ["left", "right"] if side == "both" else [side]
    data = {}
    for s in sides:
        path = candidates[s]
        if not os.path.exists(path):
            print(f"Missing: {path}")
            continue
        data[s] = pd.read_csv(path)
    if not data:
        print("No log files found.")
        sys.exit(1)
    return data


def plot_arm(name, df, fig, gs):
    t = df["time"].values
    color = {"left": "#2166ac", "right": "#d6604d"}[name]

    ax_pos = fig.add_subplot(gs[0])
    ax_pos.set_title(f"arm_{name}", fontsize=11)
    for axis, ls in zip(["x", "y", "z"], ["-", "--", ":"]):
        ax_pos.plot(t, df[f"dev_p{axis}"], color=color, ls=ls, lw=1.2, label=f"dev_{axis}")
        ax_pos.plot(t, df[f"arm_p{axis}"], color="gray", ls=ls, lw=1.0, alpha=0.7, label=f"arm_{axis}")
    ax_pos.set_ylabel("position [m]")
    ax_pos.legend(ncol=3, fontsize=7)
    ax_pos.grid(True, alpha=0.3)

    ax_vel = fig.add_subplot(gs[1], sharex=ax_pos)
    for axis, ls in zip(["x", "y", "z"], ["-", "--", ":"]):
        ax_vel.plot(t, df[f"dev_v{axis}"], color=color, ls=ls, lw=1.0, label=axis)
    ax_vel.set_ylabel("dev velocity [m/s]")
    ax_vel.legend(ncol=3, fontsize=7)
    ax_vel.grid(True, alpha=0.3)

    ax_f = fig.add_subplot(gs[2], sharex=ax_pos)
    f_imp_norm = np.linalg.norm(df[["f_imp_x", "f_imp_y", "f_imp_z"]].values, axis=1)
    f_pc_norm  = np.linalg.norm(df[["f_pc_x",  "f_pc_y",  "f_pc_z" ]].values, axis=1)
    f_tot_norm = np.linalg.norm(df[["f_tot_x", "f_tot_y", "f_tot_z"]].values, axis=1)
    ax_f.plot(t, f_imp_norm, label="|F_imp|", color="#4dac26", lw=1.2)
    ax_f.plot(t, f_pc_norm,  label="|F_pc|",  color="#f4a582", lw=1.2)
    ax_f.plot(t, f_tot_norm, label="|F_tot|", color="#d01c8b", lw=1.4)
    ax_f.set_ylabel("force magnitude [N]")
    ax_f.legend(ncol=3, fontsize=7)
    ax_f.grid(True, alpha=0.3)

    ax_fxyz = fig.add_subplot(gs[3], sharex=ax_pos)
    for axis, ls in zip(["x", "y", "z"], ["-", "--", ":"]):
        ax_fxyz.plot(t, df[f"f_tot_{axis}"], color=color, ls=ls, lw=1.0, label=axis)
    ax_fxyz.axhline(0, color="black", lw=0.5)
    ax_fxyz.set_ylabel("F_tot components [N]")
    ax_fxyz.legend(ncol=3, fontsize=7)
    ax_fxyz.grid(True, alpha=0.3)

    ax_pc = fig.add_subplot(gs[4], sharex=ax_pos)
    ax_pc_r = ax_pc.twinx()
    ax_pc.plot(t, df["e_obs"],    color="#7b3294", lw=1.2, label="e_obs [J]")
    ax_pc_r.plot(t, df["b_linear"], color="#c2a5cf", lw=1.0, ls="--", label="b_linear")
    ax_pc.set_ylabel("energy obs [J]", color="#7b3294")
    ax_pc_r.set_ylabel("b_linear [Ns/m]", color="#c2a5cf")
    ax_pc.set_xlabel("time [s]")
    lines = ax_pc.get_legend_handles_labels()[0] + ax_pc_r.get_legend_handles_labels()[0]
    labels = ax_pc.get_legend_handles_labels()[1] + ax_pc_r.get_legend_handles_labels()[1]
    ax_pc.legend(lines, labels, fontsize=7)
    ax_pc.grid(True, alpha=0.3)

    for ax in [ax_pos, ax_vel, ax_f, ax_fxyz]:
        plt.setp(ax.get_xticklabels(), visible=False)


def main():
    parser = argparse.ArgumentParser(description="Haptic interface control log viewer")
    parser.add_argument("log_dir", nargs="?", default="log")
    parser.add_argument("--side", choices=["left", "right", "both"], default="both")
    parser.add_argument("--save", metavar="FILE", help="save figure to file instead of showing")
    args = parser.parse_args()

    data = load(args.log_dir, args.side)
    n = len(data)

    fig = plt.figure(figsize=(14, 5 * n))
    fig.suptitle("Haptic interface — control log", fontsize=13)
    outer = gridspec.GridSpec(1, n, figure=fig, wspace=0.35)

    for col, (name, df) in enumerate(data.items()):
        inner = gridspec.GridSpecFromSubplotSpec(5, 1, subplot_spec=outer[col], hspace=0.12)
        plot_arm(name, df, fig, inner)

    if args.save:
        plt.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"Saved -> {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
