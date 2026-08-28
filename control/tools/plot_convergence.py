#!/usr/bin/env python3
"""plot_convergence.py — 自动调参收敛效果图

读取 ar_sim --tune 导出的 CSV（含 K̂/T̂、KpCenter/KdCenter、窗口 J），输出三张图：
  1) RLS 辨识收敛：K̂/T̂ 随时间逼近真值
  2) ESC 收敛：持久增益轨迹 + 每窗口 J 平台
  3) 全链路概览：航向/舵角/误差/实际增益/辨识估计

用法：
    python3 plot_convergence.py data.csv -o out_dir/
    python3 plot_convergence.py a.csv b.csv --trueK 0.2 0.5 --trueT 8 3 -o out_dir/

真值缺省从文件名 k<K>_t<T>（如 k0p2_t8）解析，也可用 --trueK/--trueT 显式指定。
依赖：matplotlib、numpy。
"""
import argparse
import csv
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib
if "MPLCONFIGDIR" not in __import__("os").environ:
    import os
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-cfg")
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# CJK 字体
for _cjk in ["Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Zen Hei",
             "WenQuanYi Micro Hei", "AR PL UMing CN", "AR PL UKai CN"]:
    try:
        matplotlib.font_manager.findfont(_cjk, fallback_to_default=False)
        matplotlib.rcParams["font.sans-serif"] = [_cjk] + matplotlib.rcParams["font.sans-serif"]
        break
    except Exception:
        continue
matplotlib.rcParams["axes.unicode_minus"] = False

COLS = ["index", "t", "psi", "psiRef", "rudderCmd", "rudderAct", "rate", "error",
        "Kp", "Kd", "KpCenter", "KdCenter", "Khat", "That", "identValid",
        "baseline", "Jwindow", "winIndex", "settle"]


def load_csv(path):
    samples, settles = [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            d = {c: (float(row[c]) if c != "settle" and c != "winIndex" else int(row[c]))
                 for c in COLS if c in row}
            (settles if d["settle"] else samples).append(d)
    return samples, settles


def true_from_name(path, klist, tlist, idx):
    if klist is not None and len(klist) > idx:
        return klist[idx], tlist[idx]
    m = re.search(r"k(\d+p?\d*)_t(\d+)", Path(path).stem)
    if m:
        return float(m.group(1).replace("p", ".")), float(m.group(2))
    return None, None


def fig_rls(samples, out, Ktrue, Ttrue, title):
    t = np.array([s["t"] for s in samples])
    khat = np.array([s["Khat"] for s in samples])
    that = np.array([s["That"] for s in samples])
    valid = np.array([s["identValid"] for s in samples])
    base_t = next((s["t"] for s in samples if s["baseline"]), None)

    fig, axs = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axs[0].plot(t, khat, "C0", lw=1.2, label="K_hat (RLS 估计)")
    if Ktrue:
        axs[0].axhline(Ktrue, color="C3", ls="--", lw=1.2, label=f"真值 K={Ktrue:g}")
    axs[0].set_ylabel("K_hat (1/s)")
    axs[0].legend(loc="upper right", fontsize=9)
    axs[0].grid(alpha=0.3)
    axs[0].set_title(f"{title} — RLS 辨识收敛（K_hat/T_hat）", fontsize=11)

    axs[1].plot(t, that, "C1", lw=1.2, label="T_hat (RLS 估计)")
    if Ttrue:
        axs[1].axhline(Ttrue, color="C3", ls="--", lw=1.2, label=f"真值 T={Ttrue:g}s")
    axs[1].set_ylabel("T_hat (s)")
    axs[1].set_xlabel("时间 (s)")
    axs[1].legend(loc="upper right", fontsize=9)
    axs[1].grid(alpha=0.3)
    if base_t is not None:
        for ax in axs:
            ax.axvline(base_t, color="C2", ls="-.", lw=1.0, alpha=0.8)
        axs[1].text(base_t, axs[1].get_ylim()[1], " 基线建立",
                    color="C2", fontsize=9, va="top")
    fig.tight_layout()
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"图已保存: {out}")


def fig_esc(samples, settles, out, Ktrue, Ttrue, title):
    t = np.array([s["t"] for s in samples])
    kpc = np.array([s["KpCenter"] for s in samples])
    kdc = np.array([s["KdCenter"] for s in samples])
    kp0 = kpc[0] if len(kpc) else 0.0
    kd0 = kdc[0] if len(kdc) else 0.0
    ws = np.array([s["winIndex"] for s in settles])
    js = np.array([s["Jwindow"] for s in settles])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8), sharex=True,
                                   gridspec_kw={"height_ratios": [1.6, 1]})
    ax1.plot(t, kpc, "C0", lw=1.1, label="Kp（去载波持久增益）")
    ax1.plot(t, kdc, "C1", lw=1.1, label="Kd（去载波持久增益）")
    ax1.axhline(kp0, color="C0", ls=":", lw=0.9, alpha=0.6, label=f"调度起点 Kp={kp0:.3f}")
    ax1.axhline(kd0, color="C1", ls=":", lw=0.9, alpha=0.6, label=f"调度起点 Kd={kd0:.3f}")
    for s in settles:
        ax1.axvline(s["t"], color="C2", ls="--", lw=0.7, alpha=0.45)
    ax1.set_ylabel("增益")
    ax1.legend(loc="upper right", fontsize=9, ncol=2)
    ax1.grid(alpha=0.3)
    ax1.set_title(f"{title} — ESC 增益收敛与窗口 J", fontsize=11)

    if len(ws):
        ax2.bar(ws, js, width=0.72, color="C4", alpha=0.85, label="窗口 J")
        ax2.plot(ws, js, "ko-", ms=3, lw=0.7, label="J 趋势")
        ax2.set_ylim(bottom=0)
        ax2.legend(loc="upper right", fontsize=9)
    ax2.set_xlabel("ESC 窗口序号")
    ax2.set_ylabel("ITAE J")
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"图已保存: {out}")


def fig_overview(samples, out, title):
    t = np.array([s["t"] for s in samples])
    fig, axs = plt.subplots(5, 1, figsize=(11, 11), sharex=True)
    axs[0].plot(t, [s["psi"] for s in samples], "C0", lw=1.0, label="航向 ψ")
    axs[0].plot(t, [s["psiRef"] for s in samples], "k--", lw=0.8, label="参考 ψ_ref")
    axs[0].set_ylabel("航向 (°)"); axs[0].legend(loc="upper right", fontsize=8)
    axs[0].grid(alpha=0.3); axs[0].set_title(f"{title} — 自动调参全链路", fontsize=11)

    axs[1].plot(t, [s["rudderCmd"] for s in samples], "C1", lw=1.0, label="舵指令")
    axs[1].plot(t, [s["rudderAct"] for s in samples], "C7", lw=0.7, alpha=0.7, label="实际舵角")
    axs[1].axhline(35, color="r", ls=":", lw=0.8); axs[1].axhline(-35, color="r", ls=":", lw=0.8)
    axs[1].set_ylabel("舵角 (°)"); axs[1].legend(loc="upper right", fontsize=8)
    axs[1].grid(alpha=0.3)

    axs[2].plot(t, [s["error"] for s in samples], "C3", lw=1.0)
    axs[2].axhline(0, color="k", lw=0.5)
    axs[2].set_ylabel("误差 e (°)"); axs[2].grid(alpha=0.3)

    axs[3].plot(t, [s["Kp"] for s in samples], "C4", lw=1.0, label="Kp（含载波）")
    axs[3].plot(t, [s["Kd"] for s in samples], "C5", lw=1.0, alpha=0.85, label="Kd（含载波）")
    axs[3].set_ylabel("PD 增益"); axs[3].legend(loc="upper right", fontsize=8)
    axs[3].grid(alpha=0.3)

    axs[4].plot(t, [s["Khat"] for s in samples], "C6", lw=1.1, label="K_hat")
    ax4b = axs[4].twinx()
    ax4b.plot(t, [s["That"] for s in samples], "C2", lw=1.1, label="T_hat")
    axs[4].set_ylabel("K_hat (1/s)", color="C6"); ax4b.set_ylabel("T_hat (s)", color="C2")
    axs[4].tick_params(axis="y", labelcolor="C6"); ax4b.tick_params(axis="y", labelcolor="C2")
    axs[4].set_xlabel("时间 (s)"); axs[4].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"图已保存: {out}")


def main():
    ap = argparse.ArgumentParser(description="自动调参收敛效果图")
    ap.add_argument("csv", nargs="+", help="ar_sim --tune 导出的 CSV")
    ap.add_argument("-o", "--outdir", default="docs/auto_tuning/figures",
                    help="输出目录（缺省 docs/auto_tuning/figures）")
    ap.add_argument("--trueK", nargs="+", type=float, help="真值 K 列表（与 CSV 一一对应）")
    ap.add_argument("--trueT", nargs="+", type=float, help="真值 T 列表（与 CSV 一一对应）")
    args = ap.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    for i, path in enumerate(args.csv):
        p = Path(path)
        if not p.exists():
            sys.exit(f"CSV 不存在: {p}")
        samples, settles = load_csv(p)
        kt, tt = true_from_name(p, args.trueK, args.trueT, i)
        label = p.stem.replace("tune_convergence_", "")
        stem = p.stem
        fig_rls(samples, outdir / f"{stem}_rls.png", kt, tt, label)
        fig_esc(samples, settles, outdir / f"{stem}_esc.png", kt, tt, label)
        fig_overview(samples, outdir / f"{stem}_overview.png", label)


if __name__ == "__main__":
    main()
