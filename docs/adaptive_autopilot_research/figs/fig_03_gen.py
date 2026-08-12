# -*- coding: utf-8 -*-
"""生成 03 章新增插图：航速标度律推导图 + 海况估计原理图。"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["AR PL UMing CN", "Noto Sans CJK SC", "WenQuanYi Zen Hei"]
plt.rcParams["axes.unicode_minus"] = False

OUT = "/home/jim/Desktop/auto_rudder/docs/adaptive_autopilot_research/figs"

# ---------------- 图 1：K/T 航速标度律与 PD 增益调度 ----------------
L = 30.0
Kp_nd, Tp_nd = 1.0, 1.0
U0 = 25 * 0.5144
wn, zeta = 0.3, 1.0

U = np.linspace(2.5, 14.0, 300)
K = Kp_nd * U / L
T = Tp_nd * L / U
Kp_gain = wn**2 * T / K
Kd_gain = (2 * zeta * wn * T - 1) / K
Kp_ref = wn**2 * (Tp_nd / Kp_nd) * (L / U0) ** 2

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))

l1, = ax1.plot(U * 1.9438, K, "b-", lw=2, label=r"$K=K'\,U/L$")
ax1.set_xlabel("船速 U [kn]")
ax1.set_ylabel(r"回转性指数 $K$ [1/(s·rad)]", color="b")
ax1.tick_params(axis="y", labelcolor="b")
ax1.set_ylim(0, 0.5)
ax1b = ax1.twinx()
l2, = ax1b.plot(U * 1.9438, T, "r--", lw=2, label=r"$T=T'\,L/U$")
ax1b.set_ylabel(r"追随性时间常数 $T$ [s]", color="r")
ax1b.tick_params(axis="y", labelcolor="r")
ax1b.set_ylim(0, 13)
ax1.axvline(U0 * 1.9438, color="gray", ls=":", lw=1)
ax1.text(U0 * 1.9438 - 0.3, 0.44, "设计航速 25 kn", fontsize=9, color="gray", ha="right")
ax1.set_title("(a) K/T 航速标度律（$K'=T'=1$, $L=30$ m）")
ax1.legend(handles=[l1, l2], loc="center right", fontsize=10)
ax1.grid(alpha=0.3)

ax2.plot(U * 1.9438, Kp_gain, "b-", lw=2, label=r"$K_p=\omega_n^2 T/K$（极点配置）")
ax2.plot(U * 1.9438, Kd_gain, "g-.", lw=2, label=r"$K_d=(2\zeta\omega_n T-1)/K$")
ax2.plot(U * 1.9438, Kp_ref * (U0 / U) ** 2, "r--", lw=1.5,
         label=r"$K_p(U_0)\cdot(U_0/U)^2$（Källström 调度律）")
ax2.axhline(3 * Kp_ref, color="k", ls=":", lw=1)
ax2.text(15.5, 3 * Kp_ref + 0.15, "低速增益上限（防舵角饱和）", fontsize=9)
ax2.axvline(U0 * 1.9438, color="gray", ls=":", lw=1)
ax2.set_xlabel("船速 U [kn]")
ax2.set_ylabel("PD 增益")
ax2.set_ylim(0, 8)
ax2.set_title(r"(b) PD 增益随航速调度（$\omega_n=0.3$ rad/s, $\zeta=1$）")
ax2.legend(fontsize=9, loc="upper right")
ax2.grid(alpha=0.3)

fig.tight_layout()
fig.savefig(f"{OUT}/fig_03_speed_scaling.png", dpi=160)
plt.close(fig)

# ---------------- 图 2：基于航向谱的海况估计 ----------------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))

w = np.linspace(0.01, 2.0, 800)
def bret(w, hs, wp=0.6):
    return 5.0 / 16.0 * (0.7 * hs) ** 2 * wp**4 / w**5 * np.exp(-5 / 4 * (wp / w) ** 4)
low = 0.4 * np.exp(-((w - 0.12) / 0.10) ** 2)
S2 = low + bret(w, 0.5)
S4 = low + bret(w, 2.0)
S6 = low + bret(w, 4.0)

l1, = ax1.plot(w, S2, "b", lw=1.8, label="海况 SS2（Hs=0.5 m）")
l2, = ax1.plot(w, S4, "g", lw=1.8, label="海况 SS4（Hs=2.0 m）")
l3, = ax1.plot(w, S6, "r", lw=1.8, label="海况 SS6（Hs=4.0 m）")
ax1.set_ylim(0, 1.25)
ax1.axvspan(0.3, 1.3, color="orange", alpha=0.15)
ax1.annotate("波频带 0.05–0.2 Hz\n带通 + 滑动 RMS → 海况估计",
             xy=(0.8, 0.55), xytext=(0.30, 0.75), textcoords=("data", "axes fraction"),
             fontsize=9, color="darkorange", ha="center",
             arrowprops=dict(arrowstyle="->", color="darkorange", lw=1))
ax1.set_xlabel(r"频率 $\omega$ [rad/s]")
ax1.set_ylabel(r"艏摇功率谱密度 $S(\omega)$（示意）")
ax1.set_title("(a) 波频带能量随海况单调增长")
ax1.legend(handles=[l1, l2, l3], fontsize=9, loc="upper right")
ax1.grid(alpha=0.3)

ss = np.arange(0, 8)
rms = np.array([0.05, 0.12, 0.3, 0.6, 1.1, 1.8, 2.8, 4.2])
lam = np.array([0.1, 0.15, 0.3, 0.6, 1.2, 2.0, 3.0, 4.0])
l4, = ax2.step(ss, rms, where="mid", color="b", lw=2, label="波频带艏摇 RMS [deg]")
ax2.set_xlabel("估计海况等级（sea state）")
ax2.set_ylabel("波频带艏摇 RMS [deg]", color="b")
ax2.tick_params(axis="y", labelcolor="b")
ax2.set_ylim(0, 4.8)
ax2b = ax2.twinx()
l5, = ax2b.step(ss, lam, where="mid", color="r", ls="--", lw=2, label=r"操舵权重 $\lambda$")
ax2b.set_ylabel(r"权重 $\lambda$（$J=\int(\psi_e^2+\lambda\delta^2)dt$）", color="r")
ax2b.tick_params(axis="y", labelcolor="r")
ax2b.set_ylim(0, 5.0)
ax2b.text(0.15, 4.15, r"$\lambda=4$：高海况经济操舵", fontsize=9, color="r")
ax2b.text(4.6, 0.28, r"$\lambda=0.1$：精确操舵", fontsize=9, color="r")
ax2.set_title("(b) 海况估计 → 性能权重调度映射（示意）")
ax2.legend(handles=[l4, l5], loc="upper left", fontsize=9)
ax2.grid(alpha=0.3)

fig.tight_layout()
fig.savefig(f"{OUT}/fig_03_seastate_estimation.png", dpi=160)
plt.close(fig)

print("done")
