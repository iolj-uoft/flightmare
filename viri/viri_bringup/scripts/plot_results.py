#!/usr/bin/env python3
"""Plot an engagement CSV written by engagement_monitor.py.

    rosrun viri_bringup plot_results.py ~/.ros/viri/hybrid_hover_*.csv [--out DIR] [--since-engage]

For each CSV, prints per-axis LTVKF RMSE (paper Table III) and writes:
  <name>_ltvkf.png       estimate vs ground truth with a +-3 sigma band (paper Figs. 4-6)
  <name>_trajectory.png  3-D chaser/target paths with the intercept point (paper Fig. 7)
"""
import argparse
import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ESTIMATE = "#2a78d6"   # categorical slot 1
TRUTH = "#eb6834"      # categorical slot 2
BAND = "#cde2fb"       # slot-1 ramp, light step
INK_MUTED = "#6b6a63"
PRONAV, HEADING = 1, 2


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0].keys()} if rows else {}


def style(ax):
    ax.grid(True, color="#e4e3dc", linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK_MUTED)
    ax.tick_params(colors=INK_MUTED, labelsize=8)


def rmse(d):
    out = []
    for a in "xyz":
        errs = [(e - g) ** 2 for e, g in zip(d["est_" + a], d["gt_" + a]) if not math.isnan(e)]
        out.append(math.sqrt(sum(errs) / len(errs)) if errs else float("nan"))
    return out


def plot_ltvkf(d, name, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    t = d["t"]
    for ax, a in zip(axes, "xyz"):
        est, sig = d["est_" + a], d["sig_" + a]
        lo = [e - 3 * s for e, s in zip(est, sig)]
        hi = [e + 3 * s for e, s in zip(est, sig)]
        ax.fill_between(t, lo, hi, color=BAND, linewidth=0, label="±3σ")
        ax.plot(t, est, color=ESTIMATE, linewidth=1.2, label="LTVKF estimate")
        ax.plot(t, d["gt_" + a], color=TRUTH, linewidth=1.2, linestyle="--", label="Ground truth")
        ax.set_ylabel("%s (m)" % a, color=INK_MUTED)
        style(ax)
    axes[0].legend(loc="upper right", fontsize=8, frameon=False, ncol=3)
    axes[-1].set_xlabel("Time (s)", color=INK_MUTED)
    rx, ry, rz = rmse(d)
    fig.suptitle("LTVKF relative state (GA frame) — %s\nRMSE: x = %.3f m, y = %.3f m, z = %.3f m"
                 % (name, rx, ry, rz), fontsize=10)
    fig.tight_layout()
    path = os.path.join(out_dir, name + "_ltvkf.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_trajectory(d, name, out_dir):
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(d["chaser_x"], d["chaser_y"], d["chaser_z"], color=ESTIMATE, linewidth=2, label="Chaser path")
    ax.plot(d["target_x"], d["target_y"], d["target_z"], color=TRUTH, linewidth=2, linestyle="--",
            label="Target path")
    ax.scatter([d["chaser_x"][0]], [d["chaser_y"][0]], [d["chaser_z"][0]], color=ESTIMATE, s=40,
               label="Chaser start")
    ax.scatter([d["target_x"][0]], [d["target_y"][0]], [d["target_z"][0]], color=TRUTH, s=40,
               label="Target start")
    # Intercept = first row inside the intercept radius after engagement
    hit = next((i for i, (r, m) in enumerate(zip(d["range_gt"], d["mode"]))
                if r < 1.0 and i > 0 and any(mm in (PRONAV, HEADING) for mm in d["mode"][:i])), None)
    title = name
    if hit is not None:
        ax.scatter([d["chaser_x"][hit]], [d["chaser_y"][hit]], [d["chaser_z"][hit]], marker="*",
                   s=200, color="#e34948", label="Intercept")
        engaged = next(i for i, m in enumerate(d["mode"]) if m in (PRONAV, HEADING))
        title += "\nintercept %.1f s after engagement, |r| = %.2f m" % (
            d["t"][hit] - d["t"][engaged], d["range_gt"][hit])
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(title, fontsize=10)
    ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    path = os.path.join(out_dir, name + "_trajectory.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv", nargs="+")
    p.add_argument("--out", default=None, help="output directory (default: next to the CSV)")
    p.add_argument("--since-engage", action="store_true",
                   help="only use rows after guidance engaged for RMSE and LTVKF plots")
    args = p.parse_args()

    print("%-40s %10s %10s %10s" % ("run", "RMSE x", "RMSE y", "RMSE z"))
    for path in args.csv:
        d = load(path)
        if not d:
            print("%s: empty" % path)
            continue
        name = os.path.splitext(os.path.basename(path))[0]
        out_dir = args.out or os.path.dirname(os.path.abspath(path))
        os.makedirs(out_dir, exist_ok=True)

        plot_trajectory(d, name, out_dir)
        valid = [i for i, e in enumerate(d["est_x"]) if not math.isnan(e)]
        if args.since_engage:
            valid = [i for i in valid if d["mode"][i] in (PRONAV, HEADING)]
        if not valid:
            print("%-40s no LTVKF estimates" % name)
            continue
        dv = {k: [v[i] for i in valid] for k, v in d.items()}
        plot_ltvkf(dv, name, out_dir)
        print("%-40s %10.3f %10.3f %10.3f" % ((name,) + tuple(rmse(dv))))


if __name__ == "__main__":
    main()
