#!/usr/bin/env python3
# 生成《ESC 综述导读（深入浅出）》的新增配图到 assets/
# 复用项目既有 matplotlib 风格；python3 gen_guide_figures.py 即可重新生成
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

from matplotlib import font_manager
for _f in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
           "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"):
    try:
        font_manager.fontManager.addfont(_f)
    except Exception:
        pass
_cjk = font_manager.FontProperties(
    fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc").get_name()
plt.rcParams["font.family"] = [_cjk, "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 150

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "assets")
OUT = os.path.abspath(OUT)
os.makedirs(OUT, exist_ok=True)

C_BLUE = "#2b6cb0"; C_ORANGE = "#dd6b20"; C_GREEN = "#2f855a"
C_RED = "#c53030"; C_GRAY = "#718096"; C_PURPLE = "#6b46c1"


def box(ax, x, y, w, h, text, fc="#ebf4ff", ec=C_BLUE, fs=10, bold=False, tc="black"):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                                boxstyle="round,pad=0.02,rounding_size=0.03",
                                fc=fc, ec=ec, lw=1.5))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fs, color=tc, fontweight="bold" if bold else "normal")


def arrow(ax, x1, y1, x2, y2, text="", color=C_GRAY, fs=9, lw=1.5, toff=(0, 0.06), rad=0.0):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                                 color=color, lw=lw, mutation_scale=14,
                                 shrinkA=2, shrinkB=2,
                                 connectionstyle=f"arc3,rad={rad}"))
    if text:
        ax.text((x1 + x2) / 2 + toff[0], (y1 + y2) / 2 + toff[1], text,
                ha="center", va="bottom", fontsize=fs, color=color)


# ============ 图1：经典 ESC 回路（中文标注框图） ============
def fig_loop_cn():
    fig, ax = plt.subplots(figsize=(11.5, 4.6))
    ax.set_xlim(0, 11.5); ax.set_ylim(0, 4.6); ax.axis("off")

    # 上排：信号主链路（左→右）
    box(ax, 0.25, 2.6, 1.9, 1.2, "① 抖动注入\nθ = θ̂ + a·sinωt\n（探头）", fc="#fffaf0", ec=C_ORANGE, fs=10)
    box(ax, 2.75, 2.6, 1.9, 1.2, "② 未知对象\ny = h(x)\n（山）", fc="#f7fafc", ec=C_GRAY, fs=10, bold=True)
    box(ax, 5.25, 2.6, 1.9, 1.2, "③ 高通滤波\n去掉“海拔”直流\n（只留起伏）", fc="#ebf4ff", ec=C_BLUE, fs=10)
    box(ax, 7.75, 2.6, 1.6, 1.2, "④ 乘回载波\n× sinωt\n（相关检测）", fc="#ebf4ff", ec=C_BLUE, fs=10)
    box(ax, 9.75, 2.6, 1.55, 1.2, "⑤ 低通/平均\n⇒ 梯度估计\nĝ ∝ h′(θ̂)", fc="#ebf4ff", ec=C_BLUE, fs=10)

    arrow(ax, 2.15, 3.2, 2.75, 3.2)
    arrow(ax, 4.65, 3.2, 5.25, 3.2, text="y(t)", color=C_GRAY)
    arrow(ax, 7.15, 3.2, 7.75, 3.2)
    arrow(ax, 9.35, 3.2, 9.75, 3.2)

    # 下排：积分器 + 回授
    box(ax, 7.75, 0.5, 1.6, 1.1, "⑥ 积分器\nθ̂ ← θ̂ − k·ĝ\n（迈步）", fc="#faf5ff", ec=C_PURPLE, fs=10)
    arrow(ax, 10.5, 2.6, 8.55, 1.6, color=C_PURPLE)
    # 载波支路：从注入端到乘法器（走下方 y=0.82）
    arrow(ax, 1.2, 2.6, 1.2, 0.82, color=C_ORANGE)
    ax.text(0.5, 1.75, "载波 sinωt", fontsize=9, color=C_ORANGE)
    arrow(ax, 1.2, 0.82, 8.55, 0.82, color=C_ORANGE)
    arrow(ax, 8.55, 0.82, 8.55, 2.6, color=C_ORANGE)
    # 积分器回授到注入端（走中间 y=1.35，与载波分开）
    arrow(ax, 7.75, 1.35, 1.9, 1.35, color=C_PURPLE)
    ax.text(4.8, 1.48, "θ̂（当前估计，缓慢移动）", fontsize=9, color=C_PURPLE, ha="center")
    arrow(ax, 1.9, 1.35, 1.55, 2.55, color=C_PURPLE, rad=-0.2)

    ax.text(5.75, 4.25, "经典 ESC 信号回路：探头 → 山 → 滤波 → 相关 → 平均 → 迈步（再回到起点）",
            ha="center", fontsize=12, fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "esc_loop_cn.png"), bbox_inches="tight")
    plt.close(fig)


# ============ 图2：实用稳定性（收敛到邻域并持续小振荡） ============
def fig_practical_stability():
    rng = np.random.default_rng(7)
    dt = 0.02
    T = 600.0
    t = np.arange(0, T, dt)
    omega = 0.3          # 载波 (rad/s)，为展示效果取较大
    a = 0.12             # 抖动幅值
    k_int = 0.6          # 积分增益
    th_star = 1.3

    # 经典 ESC 简化仿真：静态 map J=(θ-θ*)²，HPF 近似为去慢变均值，解调+LPF+积分
    th_hat = np.zeros_like(t); th_hat[0] = 2.2
    xi = 0.0; eta = 0.0
    wl, wh = 0.15, 0.08
    th_act = np.zeros_like(t); J = np.zeros_like(t); g_est = np.zeros_like(t)
    for i in range(1, len(t)):
        th_act[i] = th_hat[i-1] + a * np.sin(omega * t[i])
        J[i] = (th_act[i] - th_star) ** 2 + 0.02 * rng.standard_normal()
        eta += dt * (-wh * eta + wh * J[i])          # LPF 估直流
        hp = J[i] - eta                               # HPF 输出（去直流）
        xi += dt * (-wl * xi + wl * hp * np.sin(omega * t[i]))  # 解调 + LPF
        g_est[i] = xi
        th_hat[i] = th_hat[i-1] - k_int * xi * dt     # 最小化：负梯度方向

    fig, axs = plt.subplots(2, 2, figsize=(11.5, 6.4),
                            gridspec_kw={"width_ratios": [2.2, 1.0]})
    ax = axs[0, 0]
    ax.plot(t, th_act, color=C_ORANGE, lw=0.8, label=r"实际参数 $\theta(t)=\hat\theta+a\sin\omega t$")
    ax.plot(t, th_hat, color=C_BLUE, lw=1.8, label=r"估计中心 $\hat\theta(t)$")
    ax.axhline(th_star, color=C_GREEN, ls="--", lw=1.2)
    ax.axhspan(th_star - a, th_star + a, color=C_GREEN, alpha=0.10)
    ax.text(300, th_star + a + 0.08, "实用稳定邻域 $O(a)$", color=C_GREEN, fontsize=10)
    ax.text(300, th_star - a - 0.16, r"未知最优 $\theta^*$（绿色虚线）", color=C_GREEN, fontsize=10)
    ax.set_xlabel("时间 t (s)"); ax.set_ylabel(r"参数 $\theta$")
    ax.set_title("① 全程：指数式接近最优点，然后停留在邻域内", fontsize=11)
    ax.legend(loc="lower right", fontsize=9); ax.grid(alpha=0.3)

    ax = axs[0, 1]
    m = t > 480
    ax.plot(t[m], th_act[m], color=C_ORANGE, lw=1.2)
    ax.axhline(th_star, color=C_GREEN, ls="--", lw=1.2)
    ax.axhspan(th_star - a, th_star + a, color=C_GREEN, alpha=0.10)
    ax.set_title("② 放大：永远停不下来的小振荡", fontsize=11)
    ax.set_xlabel("t (s)"); ax.grid(alpha=0.3)

    ax = axs[1, 0]
    ax.plot(t, J, color=C_GRAY, lw=0.5, alpha=0.7, label="含噪代价 J(t)")
    # 滑窗均值
    w = int(20 / dt)
    Jm = np.convolve(J, np.ones(w) / w, mode="same")
    ax.plot(t, Jm, color=C_RED, lw=1.6, label="J 的滑动平均（20 s 窗）")
    ax.set_xlabel("时间 t (s)"); ax.set_ylabel("代价 J")
    ax.set_title("③ 代价随之下降并稳定在小值（残余振荡是 ESC 的固有代价）", fontsize=11)
    ax.legend(fontsize=9); ax.grid(alpha=0.3)

    ax = axs[1, 1]
    ax.plot(t, 2 * g_est / a, color=C_PURPLE, lw=1.0)
    ax.axhline(0, color=C_GRAY, lw=0.8)
    ax.set_title("④ 梯度估计 ĝ：收敛后绕 0 波动", fontsize=11)
    ax.set_xlabel("t (s)"); ax.grid(alpha=0.3)

    fig.suptitle("ESC 的“实用稳定性”：不收敛到一个点，而是收敛到最优点的小邻域并持续微振",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(os.path.join(OUT, "esc_practical_stability.png"), bbox_inches="tight")
    plt.close(fig)


fig_loop_cn()
fig_practical_stability()
print("done ->", OUT)
