#!/usr/bin/env python3
"""plot_esc_sim.py — ESC 仿真图（esc_sim_ 前缀，输出到 assets/）

读取 ar_sim --tune 导出的 CSV，生成四张图：
  esc_sim_rls.png      RLS 在线辨识 K̂/T̂ 收敛曲线（标注真值）
  esc_sim_gains.png    双面板：ESC 增益中心轨迹 + 各窗口代价 J 柱状图
  esc_sim_dither.png   约 200 s 窗内 ±5% sin/cos 载波抖动 + 航向误差 + 舵角
  esc_sim_overview.png 全链路概览：误差 / 舵角 / Kp·Kd / 窗口 J 随时间

用法：
    python3 plot_esc_sim.py tune.csv --trueK 0.2 --trueT 8 -o assets/
"""
import argparse
import csv
import os
from pathlib import Path

import numpy as np
import matplotlib
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-cfg")
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---- 统一风格：中文 Noto Sans CJK SC、白底、网格、≥150 dpi ----
matplotlib.rcParams.update({
    "font.family": "sans-serif",
    # matplotlib 将 NotoSansCJK-Regular.ttc 注册为 JP 字面（同一字体集，CJK 全覆盖）
    "font.sans-serif": ["Noto Sans CJK JP"],
    "axes.unicode_minus": False,
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "axes.grid": True,
    "grid.alpha": 0.3,
    "savefig.facecolor": "white",
    "font.size": 10,
})
DPI = 160

COLS = ["index", "t", "psi", "psiRef", "rudderCmd", "rudderAct", "rate", "error",
        "Kp", "Kd", "KpCenter", "KdCenter", "Khat", "That", "identValid",
        "baseline", "Jwindow", "winIndex", "settle"]


def load_csv(path):
    samples, settles = [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            d = {c: (int(row[c]) if c in ("settle", "winIndex", "identValid", "baseline")
                     else float(row[c])) for c in COLS if c in row}
            (settles if d["settle"] else samples).append(d)
    return samples, settles


def col(samples, name):
    return np.array([s[name] for s in samples])


def fig_rls(samples, out, Ktrue, Ttrue):
    t = col(samples, "t")
    khat, that = col(samples, "Khat"), col(samples, "That")
    base_t = next((s["t"] for s in samples if s["baseline"]), None)
    errK = (khat[-1] - Ktrue) / Ktrue * 100.0
    errT = (that[-1] - Ttrue) / Ttrue * 100.0

    fig, axs = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axs[0].plot(t, khat, "C0", lw=1.2, label=r"$\hat{K}$（RLS 在线估计）")
    axs[0].axhline(Ktrue, color="C3", ls="--", lw=1.2, label=f"真值 K = {Ktrue:g}")
    axs[0].set_ylabel(r"$\hat{K}$ (1/s)")
    axs[0].set_title("RLS 在线辨识收敛（Nomoto 一阶模型参数）", fontsize=12)
    axs[0].text(0.985, 0.05, f"末值 {khat[-1]:.4f}，相对误差 {errK:+.1f}%",
                transform=axs[0].transAxes, ha="right", fontsize=9, color="C0")

    axs[1].plot(t, that, "C1", lw=1.2, label=r"$\hat{T}$（RLS 在线估计）")
    axs[1].axhline(Ttrue, color="C3", ls="--", lw=1.2, label=f"真值 T = {Ttrue:g} s")
    axs[1].set_ylabel(r"$\hat{T}$ (s)")
    axs[1].set_xlabel("时间 (s)")
    axs[1].text(0.985, 0.05, f"末值 {that[-1]:.4f}，相对误差 {errT:+.1f}%",
                transform=axs[1].transAxes, ha="right", fontsize=9, color="C1")
    if base_t is not None:
        for ax in axs:
            ax.axvline(base_t, color="C2", ls="-.", lw=1.0, alpha=0.8)
        axs[0].text(base_t, axs[0].get_ylim()[1], " 基线建立（ESC 启动）",
                    color="C2", fontsize=9, va="top")
    for ax in axs:
        ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    print(f"图已保存: {out}")


def fig_gains(samples, settles, out):
    t = col(samples, "t")
    kpc, kdc = col(samples, "KpCenter"), col(samples, "KdCenter")
    kp0, kd0 = kpc[0], kdc[0]
    ws = np.array([s["winIndex"] for s in settles])
    js = np.array([s["Jwindow"] for s in settles])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8), sharex=False,
                                   gridspec_kw={"height_ratios": [1.6, 1]})
    ax1.plot(t, kpc, "C0", lw=1.2, label="Kp 中心（去载波持久增益）")
    ax1.plot(t, kdc, "C1", lw=1.2, label="Kd 中心（去载波持久增益）")
    ax1.axhline(kp0, color="C0", ls=":", lw=0.9, alpha=0.6,
                label=f"极点配置起点 Kp = {kp0:.3f}")
    ax1.axhline(kd0, color="C1", ls=":", lw=0.9, alpha=0.6,
                label=f"极点配置起点 Kd = {kd0:.3f}")
    for s in settles:
        ax1.axvline(s["t"], color="C2", ls="--", lw=0.7, alpha=0.4)
    ax1.set_ylabel("增益")
    ax1.set_xlabel("时间 (s)")
    ax1.legend(loc="center right", fontsize=9, ncol=2)
    ax1.set_title("ESC 增益中心轨迹与各评估窗口代价 J（窗口长 60 s）", fontsize=12)

    ax2.bar(ws, js, width=0.72, color="C4", alpha=0.85, label="窗口 ITAE 代价 J")
    ax2.plot(ws, js, "ko-", ms=3, lw=0.7)
    # 各窗 J 集中在窄带内（93196~93640），截断 y 轴以显示窗口间差异
    ax2.set_ylim(js.min() * 0.998, js.max() * 1.001)
    ax2.set_xlabel("ESC 评估窗口序号")
    ax2.set_ylabel("J（y 轴截断）")
    ax2.set_xticks(ws)
    ax2.annotate(f"首窗 {js[0]:.0f}", (ws[0], js[0]), textcoords="offset points",
                 xytext=(6, 8), fontsize=8)
    ax2.annotate(f"末窗 {js[-1]:.0f}", (ws[-1], js[-1]), textcoords="offset points",
                 xytext=(-52, 10), fontsize=8)
    ax2.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    print(f"图已保存: {out}")


def fig_dither(samples, out, t0, t1):
    win = [s for s in samples if t0 <= s["t"] <= t1]
    t = col(win, "t")
    kp, kd = col(win, "Kp"), col(win, "Kd")
    kpc, kdc = col(win, "KpCenter"), col(win, "KdCenter")
    err = col(win, "error")
    rc, ra = col(win, "rudderCmd"), col(win, "rudderAct")

    fig, axs = plt.subplots(4, 1, figsize=(11, 10), sharex=True)

    axs[0].plot(t, kp, "C0", lw=1.0, label="Kp 实际输出（含 sin 载波）")
    axs[0].plot(t, kpc, "k--", lw=1.0, label="Kp 中心")
    axs[0].plot(t, kpc * 1.05, "C3", ls=":", lw=0.9, label="中心 ×(1±5%)")
    axs[0].plot(t, kpc * 0.95, "C3", ls=":", lw=0.9)
    axs[0].set_ylabel("Kp")
    axs[0].set_title(f"ESC 注入载波验证（t = {t0:.0f}–{t1:.0f} s，"
                     f"载波周期 2π/0.05 ≈ 125.7 s）", fontsize=12)
    axs[0].legend(loc="upper right", fontsize=8, ncol=2)

    axs[1].plot(t, kd, "C1", lw=1.0, label="Kd 实际输出（含 cos 载波）")
    axs[1].plot(t, kdc, "k--", lw=1.0, label="Kd 中心")
    axs[1].plot(t, kdc * 1.05, "C3", ls=":", lw=0.9, label="中心 ×(1±5%)")
    axs[1].plot(t, kdc * 0.95, "C3", ls=":", lw=0.9)
    axs[1].set_ylabel("Kd")
    axs[1].legend(loc="upper right", fontsize=8, ncol=2)

    axs[2].plot(t, err, "C3", lw=0.9)
    axs[2].axhline(0, color="k", lw=0.5)
    axs[2].set_ylabel("航向误差 e (°)")

    axs[3].plot(t, rc, "C2", lw=0.9, label="舵指令 δ_cmd")
    axs[3].plot(t, ra, "C7", lw=0.7, alpha=0.8, label="实际舵角 δ_act")
    axs[3].set_ylabel("舵角 (°)")
    axs[3].set_xlabel("时间 (s)")
    axs[3].legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    fig.savefig(out, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    print(f"图已保存: {out}")


def fig_overview(samples, settles, out):
    t = col(samples, "t")
    fig, axs = plt.subplots(4, 1, figsize=(11, 11), sharex=True)

    axs[0].plot(t, col(samples, "error"), "C3", lw=0.9)
    axs[0].axhline(0, color="k", lw=0.5)
    axs[0].set_ylabel("航向误差 e (°)")
    axs[0].set_title("自动调参全链路概览（RLS 辨识 → 极点配置基线 → ESC 窗口寻优）",
                     fontsize=12)

    axs[1].plot(t, col(samples, "rudderCmd"), "C2", lw=0.9, label="舵指令 δ_cmd")
    axs[1].plot(t, col(samples, "rudderAct"), "C7", lw=0.7, alpha=0.8,
                label="实际舵角 δ_act")
    axs[1].axhline(35, color="r", ls=":", lw=0.8)
    axs[1].axhline(-35, color="r", ls=":", lw=0.8)
    axs[1].set_ylabel("舵角 (°)")
    axs[1].legend(loc="upper right", fontsize=8)

    axs[2].plot(t, col(samples, "KpCenter"), "C0", lw=1.2, label="Kp 中心")
    axs[2].plot(t, col(samples, "KdCenter"), "C1", lw=1.2, label="Kd 中心")
    axs[2].set_ylabel("增益")
    axs[2].legend(loc="center right", fontsize=8)

    ts = np.array([s["t"] for s in settles])
    js = np.array([s["Jwindow"] for s in settles])
    axs[3].plot(ts, js, "C4o-", ms=4, lw=1.0, label="窗口 ITAE 代价 J（结算时刻）")
    # J 集中在窄带内，截断 y 轴以显示收敛趋势
    axs[3].set_ylim(js.min() * 0.998, js.max() * 1.001)
    axs[3].set_ylabel("J（y 轴截断）")
    axs[3].set_xlabel("时间 (s)")
    axs[3].legend(loc="upper right", fontsize=8)

    base_t = next((s["t"] for s in samples if s["baseline"]), None)
    if base_t is not None:
        for ax in axs:
            ax.axvline(base_t, color="C2", ls="-.", lw=1.0, alpha=0.8)
        axs[0].text(base_t, axs[0].get_ylim()[1], " 基线建立",
                    color="C2", fontsize=9, va="top")

    fig.tight_layout()
    fig.savefig(out, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    print(f"图已保存: {out}")


def main():
    ap = argparse.ArgumentParser(description="ESC 仿真图")
    ap.add_argument("csv", help="ar_sim --tune 导出的 CSV")
    ap.add_argument("--trueK", type=float, required=True)
    ap.add_argument("--trueT", type=float, required=True)
    ap.add_argument("-o", "--outdir", default="assets")
    ap.add_argument("--dither", nargs=2, type=float, default=[400.0, 600.0],
                    metavar=("T0", "T1"), help="抖动细节图时间窗 (s)")
    args = ap.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    samples, settles = load_csv(args.csv)
    fig_rls(samples, outdir / "esc_sim_rls.png", args.trueK, args.trueT)
    fig_gains(samples, settles, outdir / "esc_sim_gains.png")
    fig_dither(samples, outdir / "esc_sim_dither.png", *args.dither)
    fig_overview(samples, settles, outdir / "esc_sim_overview.png")


if __name__ == "__main__":
    main()
