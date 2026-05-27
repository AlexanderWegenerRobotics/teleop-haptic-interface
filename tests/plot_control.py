import sys
import glob
import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

MOTION_SCALING = 4.0
SAFETY_FLOOR_Z = 0.562  # table_height + margin + fingertip


def load_sigma_logs(log_dir):
    pattern = os.path.join(log_dir, "*_control.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"No *_control.csv files found in {log_dir}")
        sys.exit(1)
    return {os.path.basename(f).replace("_control.csv", ""): pd.read_csv(f) for f in files}


def load_arm_logs(arm_log_dir):
    pattern = os.path.join(arm_log_dir, "arm_*.csv")
    files = sorted(glob.glob(pattern))
    result = {}
    for f in files:
        name = os.path.basename(f).replace(".csv", "")
        df = pd.read_csv(f, sep=";")
        result[name] = df
    return result


def plot_sigma_arm(name, df, fig, gs_row):
    t = df["time"].values

    ax1 = fig.add_subplot(gs_row[0])
    ax1.plot(t, df["arm_pz"], label="arm_pz", color="#2166ac", lw=1.5)
    ax1.plot(t, df["dev_pz"] * MOTION_SCALING + df["arm_pz"].iloc[0],
             label=f"dev_pz × {MOTION_SCALING} (cmd target)", color="#f4a582", lw=1, ls="--")
    ax1.axhline(SAFETY_FLOOR_Z, color="#d73027", lw=0.8, ls=":", label=f"floor {SAFETY_FLOOR_Z} m")
    ax1.set_ylabel("z position [m]")
    ax1.set_title(name)
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(gs_row[1], sharex=ax1)
    ax2.plot(t, df["f_imp_z"], label="f_imp_z", color="#4dac26", lw=1.2)
    ax2.plot(t, df["f_tot_z"], label="f_tot_z", color="#d01c8b", lw=1.2)
    ax2.set_ylabel("force z [N]")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(gs_row[2], sharex=ax1)
    ax3_r = ax3.twinx()
    ax3.plot(t, df["e_obs"], label="e_obs", color="#7b3294", lw=1.2)
    ax3_r.plot(t, df["b_linear"], label="b_linear", color="#c2a5cf", lw=1, ls="--")
    ax3.set_ylabel("energy obs [J]", color="#7b3294")
    ax3_r.set_ylabel("b_linear [Ns/m]", color="#c2a5cf")
    ax3.set_xlabel("time [s]")
    lines1, labels1 = ax3.get_legend_handles_labels()
    lines2, labels2 = ax3_r.get_legend_handles_labels()
    ax3.legend(lines1 + lines2, labels1 + labels2, fontsize=8)
    ax3.grid(True, alpha=0.3)

    plt.setp(ax1.get_xticklabels(), visible=False)
    plt.setp(ax2.get_xticklabels(), visible=False)


def plot_avatar_arm(name, df, fig, gs_row):
    # O_T_EE: column-major 4×4 — indices 12,13,14 = EE translation x,y,z in robot base frame
    # F_ext = O_F_ext_hat_K: estimated external wrench in world frame [Fx, Fy, Fz, Mx, My, Mz]
    t        = df["time"].values
    ee_z     = df["O_T_EE_14"].values
    ee_z_cmd = df["O_T_EE_cmd_14"].values
    f_ext_x  = df["F_ext_0"].values
    f_ext_y  = df["F_ext_1"].values
    f_ext_z  = df["F_ext_2"].values

    ax1 = fig.add_subplot(gs_row[0])
    ax1.plot(t, ee_z,     label="EE z actual", color="#2166ac", lw=1.5)
    ax1.plot(t, ee_z_cmd, label="EE z cmd",    color="#f4a582", lw=1, ls="--")
    ax1.set_ylabel("EE z base frame [m]")
    ax1.set_title(f"{name} — avatar")
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(gs_row[1], sharex=ax1)
    ax2.plot(t, f_ext_x, label="F_ext x", color="#4dac26", lw=1.2)
    ax2.plot(t, f_ext_y, label="F_ext y", color="#984ea3", lw=1.2)
    ax2.plot(t, f_ext_z, label="F_ext z", color="#d01c8b", lw=1.5)
    ax2.set_ylabel("ext. force [N]")
    ax2.set_xlabel("time [s]")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    plt.setp(ax1.get_xticklabels(), visible=False)


def main():
    log_dir     = sys.argv[1] if len(sys.argv) > 1 else "log"
    arm_log_dir = sys.argv[2] if len(sys.argv) > 2 else None

    out_dir = "docs"
    os.makedirs(out_dir, exist_ok=True)

    sigma_logs = load_sigma_logs(log_dir)
    n = len(sigma_logs)

    fig = plt.figure(figsize=(14, 4 * n))
    fig.suptitle("Teleop control log — haptic interface", fontsize=13)
    outer = gridspec.GridSpec(n, 1, figure=fig, hspace=0.5)

    for row, (name, df) in enumerate(sigma_logs.items()):
        inner = gridspec.GridSpecFromSubplotSpec(3, 1, subplot_spec=outer[row], hspace=0.15)
        plot_sigma_arm(name, df, fig, inner)

    sigma_out = os.path.join(out_dir, "control_plot.png")
    plt.savefig(sigma_out, dpi=150, bbox_inches="tight")
    print(f"Saved → {sigma_out}")
    plt.close()

    if arm_log_dir:
        arm_logs = load_arm_logs(arm_log_dir)
        if not arm_logs:
            print(f"No arm_*.csv files found in {arm_log_dir}")
        else:
            m = len(arm_logs)
            fig2 = plt.figure(figsize=(14, 4 * m))
            fig2.suptitle("Teleop control log — avatar (robot side)", fontsize=13)
            outer2 = gridspec.GridSpec(m, 1, figure=fig2, hspace=0.5)

            for row, (name, df) in enumerate(arm_logs.items()):
                inner2 = gridspec.GridSpecFromSubplotSpec(2, 1, subplot_spec=outer2[row], hspace=0.15)
                plot_avatar_arm(name, df, fig2, inner2)

            arm_out = os.path.join(out_dir, "arm_plot.png")
            plt.savefig(arm_out, dpi=150, bbox_inches="tight")
            print(f"Saved → {arm_out}")
            plt.close()


if __name__ == "__main__":
    main()
