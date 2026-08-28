#!/usr/bin/env python3
"""plot_rls.py — RLS 在线辨识仿真效果图（四张）

数据源：control/build/sim/ar_sim 导出的 CSV（.scratch/rls_report/csv/）
  - tune_{typical,fast,slow}.csv : --tune 全链路（含 Khat/That/identValid，1 s 行）
  - trial_typical.csv            : --trial 自动 Z 形辨识试验（50 Hz 行）

输出（assets/，rls_sim_ 前缀，200 dpi，白底，网格）：
  1) rls_sim_convergence.png  Khat/That 收敛曲线（典型船），真值线 + valid 门限域
  2) rls_sim_excitation.png   δ/r/dr 时间序列 + 激励门控接受/拒绝区段着色
  3) rls_sim_zigzag.png       Z 形机动航向/舵角/转首率 + r 过零高 dr 瞬态标注
  4) rls_sim_ships.png        三船型辨识相对误差收敛对比
"""
import csv
import logging
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

# ---------------- 中文字体：显式注册 .scratch/fonts 下的 SC 字面 ----------------
FONT_DIR = Path(".scratch/fonts")
for f in ("NotoSansCJKsc-Regular.otf", "NotoSansCJKsc-Bold.otf"):
    fp = FONT_DIR / f
    if fp.exists():
        font_manager.fontManager.addfont(str(fp))
matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK SC"] + \
    matplotlib.rcParams["font.sans-serif"]
matplotlib.rcParams["axes.unicode_minus"] = False

# 缺字形告警上抛到 stderr，便于检查乱码
logging.getLogger("matplotlib").setLevel(logging.WARNING)

CSV = Path(".scratch/rls_report/csv")
OUT = Path("assets")
OUT.mkdir(exist_ok=True)

# 统一风格
DPI = 200
C_TRUE = "#C0392B"   # 真值
C_EST_K = "#1F6FB2"  # $\hat{K}$
C_EST_T = "#B26A00"  # $\hat{T}$
C_VALID = "#2E8B57"  # valid 域/置位
C_ACC = "#2E8B57"    # 门控接受
C_REJ = "#C0392B"    # 门控拒绝
C_SHIP = {"typical": "#1F6FB2", "fast": "#B26A00", "slow": "#2E8B57"}


def load_tune(name):
    """读 --tune CSV，仅取 settle=0 的 1 s 采样行。"""
    with open(CSV / name, newline="") as f:
        rows = [r for r in csv.DictReader(f) if r["settle"] == "0"]
    g = lambda c: np.array([float(r[c]) for r in rows])
    return {c: g(c) for c in ("t", "psi", "psiRef", "rudderAct", "rate",
                              "Khat", "That", "identValid", "baseline")}


def load_trial(name):
    with open(CSV / name, newline="") as f:
        rows = list(csv.DictReader(f))
    g = lambda c: np.array([float(r[c]) for r in rows])
    return {c: g(c) for c in ("t", "heading", "rudderCmd", "rudderAct", "rate")}


def style_ax(ax):
    ax.grid(alpha=0.3, lw=0.6)
    ax.set_facecolor("white")


def save(fig, name):
    fig.savefig(OUT / name, dpi=DPI, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"图已保存: {OUT / name}")


# ============ 图 1：$\hat{K}$/$\hat{T}$ 收敛曲线（典型船 K=0.2/T=8） ============
def fig_convergence():
    d = load_tune("tune_typical.csv")
    K0, T0 = 0.2, 8.0
    tmax = 150.0
    m = d["t"] <= tmax
    t, kh, th, vv = d["t"][m], d["Khat"][m], d["That"][m], d["identValid"][m]
    t_valid = next((t[i] for i in range(len(t)) if vv[i] >= 1), None)

    fig, axs = plt.subplots(2, 1, figsize=(10, 6.8), sharex=True)
    # --- K 面板 ---
    ax = axs[0]
    ax.axhspan(0.0, 2.0, color=C_VALID, alpha=0.06, zorder=0,
               label="valid 门限域 0<K≤2")
    ax.axhspan(K0 * 0.75, K0 * 1.25, color=C_TRUE, alpha=0.12, zorder=1,
               label="真值 ±25%（收敛容差带）")
    ax.axhline(K0, color=C_TRUE, ls="--", lw=1.3, label=f"真值 K={K0:g} (1/s)")
    ax.plot(t, kh, color=C_EST_K, lw=1.6, label=r"$\hat{K}$ (RLS 估计)")
    ax.set_ylim(-0.03, 0.42)
    ax.set_ylabel(r"$\hat{K}$ (1/s)")
    ax.legend(loc="upper right", fontsize=9)
    style_ax(ax)
    ax.set_title("RLS 在线辨识收敛曲线 — 典型船 K=0.2/T=8"
                 "（Z 形 ±40°/15 s 航向参考，50 Hz 采样，RLS 拍 1 s）", fontsize=11)
    # --- T 面板 ---
    ax = axs[1]
    ax.axhspan(1.0, 60.0, color=C_VALID, alpha=0.06, zorder=0,
               label="valid 门限域 1≤T≤60 s")
    ax.axhline(1.0, color=C_VALID, ls=":", lw=1.0)
    ax.axhspan(T0 * 0.75, T0 * 1.25, color=C_TRUE, alpha=0.12, zorder=1,
               label="真值 ±25%（收敛容差带）")
    ax.axhline(T0, color=C_TRUE, ls="--", lw=1.3, label=f"真值 T={T0:g} s")
    ax.plot(t, th, color=C_EST_T, lw=1.6, label=r"$\hat{T}$ (RLS 估计)")
    ax.set_ylim(-2.5, 15)
    ax.set_ylabel(r"$\hat{T}$ (s)")
    ax.set_xlabel("时间 t (s)")
    ax.legend(loc="upper right", fontsize=9)
    style_ax(ax)
    # valid 置位时刻（= 极点配置基线建立时刻）
    if t_valid is not None:
        for a in axs:
            a.axvline(t_valid, color=C_VALID, ls="-.", lw=1.2, alpha=0.9)
        axs[0].annotate(f"valid 置位 t={t_valid:.0f} s\n（同拍极点配置建立基线）",
                        xy=(t_valid, 0.30), xytext=(t_valid + 18, 0.335),
                        color=C_VALID, fontsize=9,
                        arrowprops=dict(arrowstyle="->", color=C_VALID, lw=0.9))
    fig.tight_layout()
    save(fig, "rls_sim_convergence.png")
    print(f"  valid 置位 t={t_valid:.0f}s, 末值 Khat={kh[-1]:.4f} That={th[-1]:.4f}")


# ============ 图 2：δ/r/dr 与激励门控区段 ============
def fig_excitation():
    d = load_tune("tune_typical.csv")
    tmax = 120.0
    m = d["t"] <= tmax
    t, dd, rr = d["t"][m], d["rudderAct"][m], d["rate"][m]
    # 重建 1 s 节拍差分与激励门控（门控：|δ|>3° 且 (|r|>0.5°/s 或 |dr|>0.2°/s²)）
    dr = (rr[1:] - rr[:-1]) / (t[1:] - t[:-1])
    tg = t[1:]
    ok = (np.abs(dd[1:]) > 3.0) & ((np.abs(rr[1:]) > 0.5) | (np.abs(dr) > 0.2))

    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    # 门控区段着色（接受=绿、拒绝=红，以拍为中心宽 1 s）
    for ax in axs:
        for i in range(len(tg)):
            ax.axvspan(tg[i] - 0.5, tg[i] + 0.5,
                       color=(C_ACC if ok[i] else C_REJ),
                       alpha=(0.08 if ok[i] else 0.30), lw=0, zorder=0)
        style_ax(ax)
    axs[0].plot(t, dd, color=C_EST_K, lw=1.2)
    axs[0].axhline(3.0, color="k", ls="--", lw=0.9)
    axs[0].axhline(-3.0, color="k", ls="--", lw=0.9)
    axs[0].text(119, 4.0, "δ 门限 ±3°", ha="right", fontsize=8.5)
    axs[0].set_ylabel("舵角 δ (°)")
    axs[1].plot(t, rr, color=C_EST_T, lw=1.2)
    axs[1].axhline(0.5, color="k", ls="--", lw=0.9)
    axs[1].axhline(-0.5, color="k", ls="--", lw=0.9)
    axs[1].text(119, 0.75, "r 门限 ±0.5°/s", ha="right", fontsize=8.5)
    axs[1].set_ylabel("艏摇率 r (°/s)")
    axs[2].plot(tg, dr, color="#6A3D9A", lw=1.0)
    axs[2].axhline(0.2, color="k", ls="--", lw=0.9)
    axs[2].axhline(-0.2, color="k", ls="--", lw=0.9)
    axs[2].text(119, 0.3, "dr 门限 ±0.2°/s²", ha="right", fontsize=8.5)
    axs[2].set_ylabel("dr = Δr/Δt (°/s²)")
    axs[2].set_xlabel("时间 t (s)")
    # 图例（代理）
    from matplotlib.patches import Patch
    axs[0].legend(handles=[
        Patch(color=C_ACC, alpha=0.25, label="门控生效：该拍 RLS 更新被接受"),
        Patch(color=C_REJ, alpha=0.40, label="门控拒绝：该拍被丢弃")],
        loc="upper right", fontsize=9)
    axs[0].set_title("激励门控与转向瞬态的对应 — 典型船 Z 形 ±40°/15 s"
                     "（门控：|δ|>3° 且 (|r|>0.5°/s 或 |dr|>0.2°/s²)）", fontsize=11)
    # 标注被拒绝拍
    rej_t = tg[~ok]
    if len(rej_t):
        axs[2].annotate(f"舵角过零/初段小舵角拍被拒绝（共 {len(rej_t)} 拍）",
                        xy=(rej_t[0], axs[2].get_ylim()[0] * 0.7),
                        xytext=(30, axs[2].get_ylim()[0] * 0.75),
                        color=C_REJ, fontsize=9,
                        arrowprops=dict(arrowstyle="->", color=C_REJ, lw=0.9))
    fig.tight_layout()
    save(fig, "rls_sim_excitation.png")
    print(f"  门控：接受 {int(ok.sum())} 拍 / 拒绝 {int((~ok).sum())} 拍（0–120 s）"
          f"，拒绝时刻 t={['%.0f' % x for x in rej_t[:10]]}")


# ============ 图 3：Z 形机动 + r 过零高 dr 瞬态 ============
def fig_zigzag():
    d = load_trial("trial_typical.csv")
    t, psi, dc, da, r = d["t"], d["heading"], d["rudderCmd"], d["rudderAct"], d["rate"]
    # r 过零检测（50 Hz 数据）+ 过零处 ±0.5 s 窗口 |dr|
    dt = t[1] - t[0]
    w = int(0.5 / dt)  # 半窗 0.5 s
    zc = []
    for i in range(1, len(r)):
        if r[i - 1] == 0.0 or r[i - 1] * r[i] < 0:
            lo, hi = max(0, i - w), min(len(r) - 1, i + w)
            drv = abs((r[hi] - r[lo]) / (t[hi] - t[lo]))
            zc.append((t[i], drv))
    zc_hi = [(tt, dv) for tt, dv in zc if dv > 0.2]

    fig, axs = plt.subplots(3, 1, figsize=(10, 8.2), sharex=True)
    axs[0].plot(t, psi, color=C_EST_K, lw=1.2, label="航向 ψ")
    axs[0].axhline(20, color="k", ls="--", lw=0.9)
    axs[0].axhline(-20, color="k", ls="--", lw=0.9)
    axs[0].text(174, 21, "换舵阈值 ±20°", ha="right", fontsize=8.5)
    axs[0].set_ylabel("航向 ψ (°)")
    axs[0].legend(loc="lower right", fontsize=9)
    axs[1].plot(t, dc, color="#888888", lw=0.9, label="舵指令 δ_cmd")
    axs[1].plot(t, da, color=C_EST_T, lw=1.2, label="实际舵角 δ_act")
    axs[1].axhline(20, color="k", ls="--", lw=0.9)
    axs[1].axhline(-20, color="k", ls="--", lw=0.9)
    axs[1].text(174, 22, "Z 形舵角 ±20°", ha="right", fontsize=8.5)
    axs[1].set_ylabel("舵角 δ (°)")
    axs[1].legend(loc="lower right", fontsize=9)
    axs[2].plot(t, r, color="#6A3D9A", lw=1.2, label="艏摇率 r")
    axs[2].axhline(0, color="k", lw=0.7)
    axs[2].set_ylabel("r (°/s)")
    axs[2].set_xlabel("时间 t (s)")
    axs[2].legend(loc="lower right", fontsize=9)
    # 标注高 dr 过零瞬态
    for n, (tt, dv) in enumerate(zc_hi, 1):
        for ax in axs:
            ax.axvline(tt, color=C_TRUE, ls=":", lw=0.9, alpha=0.75)
        rr = float(np.interp(tt, t, r))
        axs[2].plot(tt, rr, "v", color=C_TRUE, ms=7, zorder=5)
        axs[2].annotate(f"#{n}", xy=(tt, rr), xytext=(tt - 2.5, -0.9),
                        color=C_TRUE, fontsize=8)
    axs[2].annotate(
        f"r 过零 + 高 |dr| 瞬态（{len(zc_hi)} 处，窗口 |dr|≈"
        f"{np.mean([v for _, v in zc_hi]):.1f}°/s² > 0.2 门限）\n"
        "→ 门控保留这些拍，T 方可辨识（否则 T 欠定）",
        xy=(zc_hi[0][0], 0), xytext=(38, 2.1), color=C_TRUE, fontsize=9,
        arrowprops=dict(arrowstyle="->", color=C_TRUE, lw=0.9))
    for ax in axs:
        style_ax(ax)
    axs[0].set_title("自动 Z 形辨识试验（--trial：±20° 舵 / ±20° 航向换舵，4 循环，"
                     "典型船 K=0.2/T=8）\n"
                     r"试验 175 s 收官：$\hat{K}$=0.207 (+3.4%)，"
                     r"$\hat{T}$=8.13 (+1.6%)，valid=1，极点配置增益自动应用", fontsize=11)
    fig.tight_layout()
    save(fig, "rls_sim_zigzag.png")
    print(f"  r 过零共 {len(zc)} 次，高 dr 过零 {len(zc_hi)} 次："
          f"{['%.0f' % tt for tt, _ in zc_hi]}")


# ============ 图 4：三船型辨识误差收敛对比 ============
def fig_ships():
    ships = [("typical", "典型船 K=0.2/T=8 (leg 15s)", 0.2, 8.0),
             ("fast",    "快船 K=0.5/T=3 (leg 15s)",  0.5, 3.0),
             ("slow",    "慢船 K=0.1/T=20 (leg 60s)", 0.1, 20.0)]
    tmax = 300.0
    fig, axs = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for key, label, K0, T0 in ships:
        d = load_tune(f"tune_{key}.csv")
        m = d["t"] <= tmax
        t = d["t"][m]
        ek = (d["Khat"][m] - K0) / K0 * 100.0
        et = (d["That"][m] - T0) / T0 * 100.0
        tv = next((d["t"][i] for i in range(len(d["t"]))
                   if d["identValid"][i] >= 1), None)
        lab = f"{label}：t=300s 处 {ek[-1]:+.1f}%/{et[-1]:+.1f}%，valid@{tv:.0f}s"
        axs[0].plot(t, ek, color=C_SHIP[key], lw=1.5, label=lab)
        axs[1].plot(t, et, color=C_SHIP[key], lw=1.5, label=lab)
    for ax in axs:
        ax.axhline(0, color="k", lw=0.8)
        ax.axhline(25, color=C_TRUE, ls="--", lw=1.0)
        ax.axhline(-25, color=C_TRUE, ls="--", lw=1.0)
        ax.axhspan(-25, 25, color=C_TRUE, alpha=0.06, zorder=0)
        ax.text(298, 26.5, "验收容差 ±25%", ha="right", color=C_TRUE, fontsize=8.5)
        style_ax(ax)
        ax.legend(loc="upper right", fontsize=9)
    axs[0].set_ylabel(r"$\hat{K}$ 相对误差 (%)")
    axs[1].set_ylabel(r"$\hat{T}$ 相对误差 (%)")
    axs[1].set_xlabel("时间 t (s)")
    axs[0].set_ylim(-30, 40)
    axs[1].set_ylim(-110, 40)
    axs[0].set_title("三船型 RLS 辨识相对误差收敛对比（相对真值，%）\n"
                     "慢船 Z 形参考翻转周期放大至 60 s（≥2T），避免仅观测初始瞬态",
                     fontsize=11)
    fig.tight_layout()
    save(fig, "rls_sim_ships.png")


if __name__ == "__main__":
    fig_convergence()
    fig_excitation()
    fig_zigzag()
    fig_ships()
    print("全部完成")
