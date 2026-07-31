#!/usr/bin/env python3
"""plot_sim.py — 自动舵闭环仿真结果可视化工具

读取 ar_sim 导出的 CSV（含 t/heading/headingRef/rudderCmd/rudderAct/rate/error/Kp/Kd/mode），
生成多子图分析图与（可选）机动极坐标轨迹图。

用法：
    python3 plot_sim.py step.csv                       # 单文件，弹窗显示
    python3 plot_sim.py step.csv -o step.png           # 保存为 PNG
    python3 plot_sim.py a.csv b.csv c.csv -o cmp.png   # 多文件叠加对比
    python3 plot_sim.py maneuver.csv --polar -o m.png  # 追加机动极坐标图
    python3 plot_sim.py *.csv --grid -o grid.png       # 多文件网格平铺
    python3 plot_sim.py step.csv --metrics             # 打印指标表

依赖：matplotlib、numpy（系统包 python3-matplotlib / python3-numpy）。
"""
import argparse, csv, os, sys
from pathlib import Path
import numpy as np
import matplotlib
if os.environ.get("PLOT_SIM_AGG"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# 自动检测并启用 CJK 字体（避免中文乱码）
for _cjk in ["Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Zen Hei",
             "WenQuanYi Micro Hei", "AR PL UMing CN", "AR PL UKai CN"]:
    try:
        matplotlib.font_manager.findfont(_cjk, fallback_to_default=False)
        matplotlib.rcParams["font.sans-serif"] = [_cjk] + matplotlib.rcParams["font.sans-serif"]
        break
    except Exception:
        continue
matplotlib.rcParams["axes.unicode_minus"] = False

COLUMNS = ["index", "t", "heading", "headingRef", "rudderCmd", "rudderAct",
           "rate", "error", "Kp", "Kd", "mode"]


def load_csv(path):
    data = {c: [] for c in COLUMNS}
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise ValueError(f"{path}: 空 CSV")
        header = [h.strip() for h in header]
        idx_map = {h: i for i, h in enumerate(header)}
        for row in reader:
            if not row or len(row) < len(COLUMNS):
                continue
            for c in COLUMNS:
                if c not in idx_map:
                    continue
                v = row[idx_map[c]]
                try:
                    data[c].append(int(float(v)) if c == "mode" else float(v))
                except ValueError:
                    data[c].append(0.0)
    return {c: np.array(data[c]) for c in COLUMNS}


def compute_metrics(data):
    t, psi, ref, err = data["t"], data["heading"], data["headingRef"], data["error"]
    step_deg = float(np.max(np.abs(np.diff(ref)))) if len(ref) > 1 else 0.0
    if step_deg > 1e-6:
        idx_step = int(np.argmax(np.abs(np.diff(ref)))) + 1
        post = psi[idx_step:]
        target = ref[idx_step] if idx_step < len(ref) else 0.0
        if len(post) and abs(target) > 1e-6:
            overshoot = (np.max(post) - target) if target >= 0 else (target - np.min(post))
            overshoot_pct = overshoot / abs(target) * 100.0
        else:
            overshoot_pct = 0.0
        band = 0.05 * step_deg
        settle = 0.0
        for ti, ei in zip(t, err):
            if abs(ei) > band:
                settle = float(ti)
    else:
        overshoot_pct, settle = 0.0, 0.0
    if len(t) > 1:
        mask = t >= t[-1] - 20.0
        ess = float(np.std(err[mask])) if mask.sum() > 1 else 0.0
    else:
        ess = 0.0
    cmd = data["rudderCmd"]
    if len(cmd) > 1 and (t[-1] - t[0]) > 0:
        freq = int(np.sum((cmd[:-1] * cmd[1:]) < 0)) * 60.0 / (t[-1] - t[0])
    else:
        freq = 0.0
    return {"overshoot_pct": overshoot_pct, "settle_s": settle,
            "ess_1sigma": ess, "rudder_freq": freq}


def print_metrics_table(metrics_dict):
    print(f"{'case':<24} {'overshoot':>10} {'settle':>8} {'ess1σ':>8} {'freq':>6}")
    print("-" * 60)
    for name, m in metrics_dict.items():
        print(f"{name:<24} {m['overshoot_pct']:>9.2f}% {m['settle_s']:>7.2f}s "
              f"{m['ess_1sigma']:>7.3f} {m['rudder_freq']:>5.2f}")


def _annotate_mode_changes(axes, t, mode):
    if len(mode) < 2:
        return
    for i in range(1, len(mode)):
        if mode[i] != mode[i-1]:
            for ax in axes:
                ax.axvline(t[i], color="g", ls="-.", lw=0.6, alpha=0.6)


def plot_multipane(data, title=""):
    fig = plt.gcf()
    gs = GridSpec(5, 1, figure=fig, hspace=0.35, left=0.08, right=0.97,
                  top=0.92, bottom=0.08)
    t = data["t"]
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, data["heading"], "C0", lw=1.2, label="航向 ψ")
    ax1.plot(t, data["headingRef"], "k--", lw=1.0, label="参考 ψ_ref")
    ax1.set_ylabel("航向 (°)"); ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(alpha=0.3); ax1.set_title(title, fontsize=10)

    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    ax2.plot(t, data["rudderCmd"], "C1", lw=1.2, label="舵指令 δ_cmd")
    ax2.plot(t, data["rudderAct"], "C7", lw=0.8, alpha=0.7, label="实际 δ_act")
    ax2.axhline(35, color="r", ls=":", lw=0.8); ax2.axhline(-35, color="r", ls=":", lw=0.8)
    ax2.set_ylabel("舵角 (°)"); ax2.legend(loc="upper right", fontsize=8); ax2.grid(alpha=0.3)

    ax3 = fig.add_subplot(gs[2], sharex=ax1)
    ax3.plot(t, data["rate"], "C2", lw=1.0)
    ax3.set_ylabel("转首率 (°/s)"); ax3.grid(alpha=0.3)

    ax4 = fig.add_subplot(gs[3], sharex=ax1)
    ax4.plot(t, data["error"], "C3", lw=1.0); ax4.axhline(0, color="k", lw=0.5)
    ax4.set_ylabel("误差 e (°)"); ax4.grid(alpha=0.3)

    ax5 = fig.add_subplot(gs[4], sharex=ax1)
    ax5.plot(t, data["Kp"], "C4", lw=1.0, label="Kp")
    ax5b = ax5.twinx()
    ax5b.plot(t, data["Kd"], "C5", lw=1.0, label="Kd")
    ax5.set_ylabel("Kp", color="C4"); ax5b.set_ylabel("Kd", color="C5")
    ax5.tick_params(axis="y", labelcolor="C4"); ax5b.tick_params(axis="y", labelcolor="C5")
    ax5.set_xlabel("时间 (s)"); ax5.grid(alpha=0.3)

    _annotate_mode_changes([ax1, ax2, ax3, ax4, ax5], t, data["mode"])
    return [ax1, ax2, ax3, ax4, ax5]


def plot_multipane_overlay(datasets, colors, labels, title=""):
    fig = plt.gcf()
    gs = GridSpec(5, 1, figure=fig, hspace=0.35, left=0.08, right=0.97,
                  top=0.92, bottom=0.08)
    axes = []
    for j, (data, c, lab) in enumerate(zip(datasets, colors, labels)):
        t = data["t"]
        if j == 0:
            ax1 = fig.add_subplot(gs[0]); ax1.set_ylabel("航向 (°)")
            ax1.set_title(title, fontsize=10); ax1.grid(alpha=0.3); axes.append(ax1)
        else:
            ax1 = axes[0]
        ax1.plot(t, data["heading"], c, lw=1.0, label=f"{lab} ψ")
        ax1.plot(t, data["headingRef"], c, lw=0.8, ls="--", alpha=0.6)

        if j == 0:
            ax2 = fig.add_subplot(gs[1], sharex=ax1); ax2.set_ylabel("舵角 (°)")
            ax2.axhline(35, color="r", ls=":", lw=0.8); ax2.axhline(-35, color="r", ls=":", lw=0.8)
            ax2.grid(alpha=0.3); axes.append(ax2)
        else:
            ax2 = axes[1]
        ax2.plot(t, data["rudderCmd"], c, lw=1.0, label=lab)

        if j == 0:
            ax3 = fig.add_subplot(gs[2], sharex=ax1); ax3.set_ylabel("转首率 (°/s)")
            ax3.grid(alpha=0.3); axes.append(ax3)
        else:
            ax3 = axes[2]
        ax3.plot(t, data["rate"], c, lw=0.9, alpha=0.85)

        if j == 0:
            ax4 = fig.add_subplot(gs[3], sharex=ax1); ax4.set_ylabel("误差 e (°)")
            ax4.grid(alpha=0.3); axes.append(ax4)
        else:
            ax4 = axes[3]
        ax4.plot(t, data["error"], c, lw=0.9, alpha=0.85)

        if j == 0:
            ax5 = fig.add_subplot(gs[4], sharex=ax1); ax5.set_ylabel("Kp")
            ax5.set_xlabel("时间 (s)"); ax5.grid(alpha=0.3); ax5b = ax5.twinx()
            ax5b.set_ylabel("Kd"); axes.append(ax5); axes.append(ax5b)
        else:
            ax5, ax5b = axes[4], axes[5]
        ax5.plot(t, data["Kp"], c, lw=0.9, alpha=0.85)
        ax5b.plot(t, data["Kd"], c, lw=0.9, alpha=0.6, ls=":")

    axes[0].legend(loc="upper right", fontsize=7)
    axes[1].legend(loc="upper right", fontsize=7)
    return axes


def plot_polar(data, ax=None, color=None, label=""):
    psi = np.deg2rad(data["heading"])
    dpsi = np.abs(np.diff(data["heading"], prepend=data["heading"][0]))
    r = np.cumsum(dpsi)
    if ax is None:
        fig = plt.gcf()
        ax = fig.add_subplot(111, projection="polar")
    ax.plot(psi, r, color=color or "C0", lw=1.2, label=label or "轨迹")
    ax.scatter([psi[0]], [r[0]], color="g", s=40, zorder=5, label="起点")
    ax.scatter([psi[-1]], [r[-1]], color="r", s=40, zorder=5, label="终点")
    ax.set_theta_zero_location("N"); ax.set_theta_direction(-1)
    ax.set_title("机动轨迹极坐标（θ=航向, r=累计转首角）", pad=15)
    ax.legend(loc="lower right", fontsize=8, bbox_to_anchor=(1.15, 0.0))
    return ax


def _polar_path(out):
    p = Path(out)
    return str(p.with_name(p.stem + "_polar" + p.suffix))


def main():
    ap = argparse.ArgumentParser(description="自动舵闭环仿真结果可视化")
    ap.add_argument("csv", nargs="+", help="ar_sim 导出的 CSV 路径（可多个）")
    ap.add_argument("-o", "--out", help="输出图片路径；不指定则弹窗显示")
    ap.add_argument("--polar", action="store_true", help="追加机动极坐标轨迹图")
    ap.add_argument("--no-multipane", action="store_true", help="不画多子图")
    ap.add_argument("--grid", action="store_true", help="多文件网格平铺")
    ap.add_argument("--metrics", action="store_true", help="打印指标表")
    ap.add_argument("--title", default="", help="图总标题")
    args = ap.parse_args()

    paths = [Path(p) for p in args.csv]
    for p in paths:
        if not p.exists():
            sys.exit(f"CSV 不存在: {p}")

    datasets = [load_csv(p) for p in paths]
    labels = [p.stem for p in paths]
    colors = [f"C{i}" for i in range(len(datasets))]
    n = len(datasets)

    if args.metrics:
        m = {labels[i]: compute_metrics(datasets[i]) for i in range(n)}
        print_metrics_table(m)

    if args.grid and n > 1:
        ncols = min(2, n); nrows = (n + ncols - 1) // ncols
        fig = plt.figure(figsize=(7 * ncols, 4 * nrows))
        fig.suptitle(args.title or f"自动舵仿真对比 ({n} 场景)", fontsize=12)
        for i, (data, lab, c) in enumerate(zip(datasets, labels, colors)):
            ax = plt.subplot(nrows, ncols, i + 1)
            t = data["t"]
            ax.plot(t, data["heading"], c, lw=1.0, label=f"{lab} ψ")
            ax.plot(t, data["headingRef"], "k--", lw=0.8, alpha=0.6, label="ψ_ref")
            ax.set_title(lab, fontsize=9); ax.set_xlabel("时间 (s)"); ax.set_ylabel("°")
            ax.grid(alpha=0.3); ax.legend(fontsize=7)
    elif not args.no_multipane:
        if n == 1:
            fig = plt.figure(figsize=(11, 10))
            plot_multipane(datasets[0], title=args.title or f"自动舵闭环仿真 — {labels[0]}")
        else:
            fig = plt.figure(figsize=(11, 11))
            plot_multipane_overlay(datasets, colors, labels,
                                    title=args.title or f"自动舵仿真叠加对比 ({n} 场景)")

    if args.polar:
        if n == 1 and not args.no_multipane:
            fig2 = plt.figure(figsize=(8, 8))
            plot_polar(datasets[0], label=labels[0])
        else:
            fig = plt.figure(figsize=(8, 8))
            ax = None
            for data, c, lab in zip(datasets, colors, labels):
                ax = plot_polar(data, ax=ax, color=c, label=lab)

    if args.out:
        if args.polar and (n == 1 and not args.no_multipane):
            plt.figure(1); plt.savefig(args.out, dpi=120, bbox_inches="tight")
            print(f"多子图已保存: {args.out}")
            plt.figure(2); polar_out = _polar_path(args.out)
            plt.savefig(polar_out, dpi=120, bbox_inches="tight")
            print(f"极坐标图已保存: {polar_out}")
        else:
            plt.savefig(args.out, dpi=120, bbox_inches="tight")
            print(f"图已保存: {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
