#!/usr/bin/env python3
# 生成《自动调参流程图文详解》的全部配图到 docs/auto_tuning/figures/
# 纯 matplotlib，无需额外依赖；python3 gen_figures.py 即可重新生成
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

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figures")
os.makedirs(OUT, exist_ok=True)

C_BLUE = "#2b6cb0"; C_ORANGE = "#dd6b20"; C_GREEN = "#2f855a"
C_RED = "#c53030"; C_GRAY = "#718096"; C_PURPLE = "#6b46c1"


def box(ax, x, y, w, h, text, fc="#ebf4ff", ec=C_BLUE, fs=10, bold=False, tc="black"):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                                boxstyle="round,pad=0.02,rounding_size=0.03",
                                fc=fc, ec=ec, lw=1.5))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fs, color=tc, fontweight="bold" if bold else "normal")


def arrow(ax, x1, y1, x2, y2, text="", color=C_GRAY, fs=9, style="-|>", lw=1.5, toff=(0, 0.04)):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle=style,
                                 color=color, lw=lw, mutation_scale=14,
                                 shrinkA=2, shrinkB=2))
    if text:
        ax.text((x1 + x2) / 2 + toff[0], (y1 + y2) / 2 + toff[1], text,
                ha="center", va="bottom", fontsize=fs, color=color)


# ============ 图1：三层架构 ============
def fig1():
    fig, ax = plt.subplots(figsize=(11, 6.2))
    ax.set_xlim(0, 11); ax.set_ylim(0, 6.2); ax.axis("off")

    box(ax, 0.3, 4.6, 2.2, 1.1, "层1 增益调度表\nGainSchedule\n(船速/海况查表)", fc="#f0fff4", ec=C_GREEN)
    box(ax, 3.4, 4.6, 2.6, 1.1, "层2 RLS 在线辨识\n+ 极点配置\n(建基线 Kp,Kd)", fc="#ebf4ff", ec=C_BLUE, bold=False)
    box(ax, 6.9, 4.6, 2.6, 1.1, "层3 ESC 极值搜索\n(基线附近连续微调)", fc="#faf5ff", ec=C_PURPLE)
    box(ax, 3.4, 2.4, 2.6, 1.0, "PD 控制器\nδ = Kp·e + Kd·ė", fc="#fffaf0", ec=C_ORANGE, bold=True)
    box(ax, 6.9, 2.4, 2.6, 1.0, "船舶\nNomoto: T·ṙ+r = K·δ", fc="#f7fafc", ec=C_GRAY)
    box(ax, 3.4, 0.5, 2.6, 1.0, "Nomoto 辨识器\nRLS: θ=[K,T]", fc="#ebf4ff", ec=C_BLUE)

    ax.add_patch(FancyBboxPatch((3.2, 4.4), 6.5, 1.5,
                                boxstyle="round,pad=0.02,rounding_size=0.05",
                                fc="none", ec="#a0aec0", lw=1.2, ls="--"))
    ax.text(6.45, 5.75, "AutoTuner::tick() 每拍编排", fontsize=10, color=C_GRAY, ha="center")

    arrow(ax, 2.5, 5.15, 3.4, 5.15, "名义增益\n(兜底/初值)", C_GREEN)
    arrow(ax, 6.0, 5.15, 6.9, 5.15, "基线")
    arrow(ax, 4.7, 4.6, 4.7, 3.4, "Kp,Kd\n(含±5%载波)", C_PURPLE, toff=(0.9, -0.05))
    arrow(ax, 6.0, 2.9, 6.9, 2.9, "舵角 δ", C_ORANGE)
    arrow(ax, 8.2, 2.4, 8.2, 1.0, "航向ψ / 转向率r", C_GRAY, toff=(1.3, 0))
    arrow(ax, 8.2, 1.0, 6.0, 1.0)
    arrow(ax, 4.7, 1.5, 4.7, 2.4, "误差 e", C_GRAY)
    arrow(ax, 3.4, 1.0, 1.4, 1.0, "", C_BLUE)
    ax.annotate("", xy=(1.4, 4.6), xytext=(1.4, 1.0),
                arrowprops=dict(arrowstyle="-|>", color=C_BLUE, lw=1.5))
    ax.text(0.55, 2.9, "δ, r 每拍喂入", fontsize=9, color=C_BLUE, ha="center", rotation=90)
    ax.annotate("", xy=(3.4, 5.15), xytext=(1.4, 2.5),
                arrowprops=dict(arrowstyle="-|>", color=C_BLUE, lw=1.5,
                                connectionstyle="arc3,rad=-0.25"))
    ax.text(1.6, 3.6, "K,T 估计\n(漂移>25%重基线)", fontsize=9, color=C_BLUE, ha="center")
    ax.set_title("三层递进式自动调参架构", fontsize=14, pad=12)
    fig.tight_layout(); fig.savefig(f"{OUT}/fig1_architecture.png", bbox_inches="tight"); plt.close(fig)


# ============ 图2：tick() 每拍流程 ============
def fig2():
    fig, ax = plt.subplots(figsize=(8.5, 10.5))
    ax.set_xlim(0, 8.5); ax.set_ylim(0, 10.5); ax.axis("off")
    W = 4.6; X = (8.5 - W) / 2

    steps = [
        ("速度门控: speed ≤ 15 kn ?", "#fff5f5", C_RED),
        ("基线未建立 → 跟随调度表名义增益", "#f0fff4", C_GREEN),
        ("辨识评估: excited & valid →\n漂移>25%? → 极点配置+稳定门 → 重基线", "#ebf4ff", C_BLUE),
        ("性能样本累积: |e|,|δ|,|dδ| 入环形窗\n振荡检测(参考稳定30s后) + ESC每拍解调", "#faf5ff", C_PURPLE),
        ("窗口结算(每60s): 梯度归一化→LPF→\n积分器→±10%限幅→稳定检查→接受/回退", "#faf5ff", C_PURPLE),
        ("ESC 扰动注入: Kp×(1+a·sinωt)\nKd×(1+a·cosωt)", "#fffaf0", C_ORANGE),
        ("返回本拍增益 → PD 闭环", "#f7fafc", C_GRAY),
    ]
    H = 0.95; GAP = 0.42
    y = 10.5 - H
    for i, (txt, fc, ec) in enumerate(steps):
        box(ax, X, y, W, H, txt, fc=fc, ec=ec, fs=10)
        if i < len(steps) - 1:
            arrow(ax, 8.5 / 2, y, 8.5 / 2, y - GAP)
        y -= H + GAP
    ax.set_title("AutoTuner::tick() 每拍（50 Hz）内部流程", fontsize=14, pad=10)
    fig.tight_layout(); fig.savefig(f"{OUT}/fig2_tick_flow.png", bbox_inches="tight"); plt.close(fig)


# ============ 图3：ESC 信号链 ============
def fig3():
    fig, ax = plt.subplots(figsize=(12, 5.6))
    ax.set_xlim(0, 12); ax.set_ylim(0, 5.6); ax.axis("off")

    box(ax, 0.3, 3.2, 1.7, 1.0, "基线增益\nKp*, Kd*", fc="#ebf4ff", ec=C_BLUE)
    box(ax, 2.6, 3.2, 1.7, 1.0, "×(1+a·sinωt)\n扰动注入", fc="#fffaf0", ec=C_ORANGE)
    box(ax, 4.9, 3.2, 1.7, 1.0, "闭环系统\n(PD+船舶)", fc="#f7fafc", ec=C_GRAY)
    box(ax, 7.2, 3.2, 1.7, 1.0, "成本增量\nc_k (ITAE)", fc="#f7fafc", ec=C_GRAY)
    box(ax, 9.5, 3.2, 1.7, 1.0, "HPF τ=5s\n去直流", fc="#faf5ff", ec=C_PURPLE)

    box(ax, 9.5, 1.0, 1.7, 1.0, "× sin(ωt)\n× cos(ωt)\n相关解调", fc="#faf5ff", ec=C_PURPLE)
    box(ax, 7.2, 1.0, 1.7, 1.0, "窗末归一化\n+ LPF τ=10s", fc="#faf5ff", ec=C_PURPLE)
    box(ax, 4.9, 1.0, 1.7, 1.0, "积分器\nK -= k·ĝ", fc="#faf5ff", ec=C_PURPLE)
    box(ax, 2.6, 1.0, 1.7, 1.0, "稳定门\n±10%限幅/回退", fc="#fff5f5", ec=C_RED)

    arrow(ax, 2.0, 3.7, 2.6, 3.7)
    arrow(ax, 4.3, 3.7, 4.9, 3.7, "增益+载波", C_ORANGE, toff=(0, 0.06))
    arrow(ax, 6.6, 3.7, 7.2, 3.7, "e, δ", C_GRAY)
    arrow(ax, 8.9, 3.7, 9.5, 3.7)
    arrow(ax, 10.35, 3.2, 10.35, 2.0, "每拍", C_PURPLE, toff=(0.3, 0))
    arrow(ax, 9.5, 1.5, 8.9, 1.5)
    arrow(ax, 7.2, 1.5, 6.6, 1.5, "每60s一次", C_PURPLE, toff=(0, 0.06))
    arrow(ax, 4.9, 1.5, 4.3, 1.5)
    arrow(ax, 3.45, 2.0, 3.45, 3.2, "更新基线", C_RED, toff=(0.45, 0))
    ax.text(6.0, 0.35, "sin 载波 → 解调出 ∂J/∂Kp 方向；cos 载波（正交）→ ∂J/∂Kd，双参数同时调互不串扰",
            fontsize=10, color=C_GRAY, ha="center")
    ax.set_title("层3 ESC 极值搜索：标准三环结构（HPF → 解调 → LPF → 积分器）", fontsize=13, pad=10)
    fig.tight_layout(); fig.savefig(f"{OUT}/fig3_esc_chain.png", bbox_inches="tight"); plt.close(fig)


# ============ 图4：ESC 数值演示（二次型成本面上的真实寻优） ============
def fig4():
    rng = np.random.default_rng(7)
    dt, win_sec = 0.02, 60.0
    a, w, step = 0.05, 0.5, 0.02   # 演示用载波频率高于工程值(0.05 rad/s)，便于单窗内含多个周期
    tau_hpf, tau_lpf = 5.0, 10.0
    win_beats = int(win_sec / dt)

    def cost(kp):  # 未知最优在 kp*=1.3；尺度放大便于演示
        return 500.0 * (kp - 1.3) ** 2 + 75.0

    kp = 2.0
    kp_hist, g_hist, j_hist = [kp], [], []
    phase = 0.0
    hpf_y, hpf_c_prev, demod_sum, grad = 0.0, 0.0, 0.0, 0.0
    t_wave, kp_wave = [], []
    n_win = 12
    for wn_ in range(n_win):
        demod_sum, cnt = 0.0, 0
        for k in range(win_beats):
            t = (wn_ * win_beats + k) * dt
            kp_d = kp * (1 + a * np.sin(phase))
            c = cost(kp_d) * dt + rng.normal(0, 5e-3)  # 成本增量+测量噪声
            al_h = tau_hpf / (tau_hpf + dt)
            hpf_y = al_h * (hpf_y + c - hpf_c_prev)
            hpf_c_prev = c
            demod_sum += hpf_y * np.sin(phase)
            cnt += 1
            phase += w * dt
            t_wave.append(t); kp_wave.append(kp_d)
        g = 2.0 * demod_sum / (a * cnt)
        al_l = win_sec / (tau_lpf + win_sec)
        grad = (1 - al_l) * grad + al_l * g
        kp_new = kp - step * grad
        dmax = max(abs(kp) * 0.10, 1e-3)
        kp = kp + np.clip(kp_new - kp, -dmax, dmax)
        kp_hist.append(kp); g_hist.append(grad); j_hist.append(cost(kp))

    fig, axs = plt.subplots(2, 2, figsize=(11, 7.5))
    ks = np.linspace(0.6, 2.4, 200)
    axs[0, 0].plot(ks, 500.0 * (ks - 1.3) ** 2 + 75.0, color=C_GRAY, lw=2, label="真实成本 J(Kp)（对调参器不可见）")
    axs[0, 0].plot(kp_hist[1:], j_hist, "o-", color=C_ORANGE, ms=6, lw=1.5, label="ESC 轨迹（每窗一点）")
    axs[0, 0].axvline(1.3, color=C_GREEN, ls="--", lw=1)
    axs[0, 0].text(1.33, 400, "未知最优 Kp*=1.3", color=C_GREEN, fontsize=9)
    axs[0, 0].set_xlabel("Kp"); axs[0, 0].set_ylabel("J"); axs[0, 0].legend(fontsize=8, loc="upper right")
    axs[0, 0].set_title("① 无模型寻优：沿估测梯度爬向谷底")

    axs[0, 1].plot(np.arange(len(kp_hist)) * 60, kp_hist, "s-", color=C_BLUE, lw=1.8)
    axs[0, 1].axhline(1.3, color=C_GREEN, ls="--", lw=1)
    axs[0, 1].set_xlabel("时间 (s)"); axs[0, 1].set_ylabel("Kp 基线值")
    axs[0, 1].set_title("② 增益收敛（每 60 s 窗口更新一次，步长≤10%）")

    tw = np.array(t_wave[:3000]); kw = np.array(kp_wave[:3000])
    axs[1, 0].plot(tw, kw, color=C_ORANGE, lw=0.8)
    axs[1, 0].set_xlabel("时间 (s)"); axs[1, 0].set_ylabel("实际施加的 Kp")
    axs[1, 0].set_title("③ 窗口内增益载波 Kp·(1+a·sinωt)，a=5%")

    axs[1, 1].plot(np.arange(1, len(g_hist) + 1) * 60, g_hist, "o-", color=C_PURPLE, lw=1.8)
    axs[1, 1].axhline(0, color=C_GRAY, lw=0.8)
    axs[1, 1].set_xlabel("时间 (s)"); axs[1, 1].set_ylabel("梯度估计 ĝ")
    axs[1, 1].set_title("④ 解调+LPF 后的梯度估计：Kp>1.3 时 ĝ>0 → 减 Kp")
    for a_ in axs.flat: a_.grid(alpha=0.3)
    fig.suptitle("ESC 数值演示：从 Kp=2.0 自收敛到未知最优 1.3（复现 auto_tuner.hpp 的算法链）", fontsize=13)
    fig.tight_layout(); fig.savefig(f"{OUT}/fig4_esc_demo.png", bbox_inches="tight"); plt.close(fig)


# ============ 图5：Z 形辨识试验 + RLS 收敛 ============
def fig5():
    K_true, T_true = 0.8, 12.0
    dt = 0.02
    t_end = 900.0
    n = int(t_end / dt)
    dz, phiz = 20.0, 20.0
    rudder = np.zeros(n); heading = np.zeros(n); rate = np.zeros(n)
    # 生成 Z 形舵令：航向累计变化达 ±20° 反舵
    d, dsign, cum = dz, 1, 0.0   # 首段即打 +dz 满舵
    psi, r = 0.0, 0.0
    for k in range(n):
        rudder[k] = d
        r += dt * (K_true * d - r) / T_true
        psi += r * dt
        heading[k] = psi; rate[k] = r
        cum += r * dt
        if cum * (dsign * phiz) >= phiz * phiz:
            dsign *= -1; d = dsign * dz; cum = 0.0

    # 按工程实现做 RLS（1s 拍、激励门控、归一化）
    lam, beat = 0.98, 1.0
    d_scale, dr_scale = 50.0, 1.0
    theta = np.zeros(2); P = np.eye(2)
    prev_r, elapsed, first = 0.0, 0.0, True
    t_beat, k_hat, t_hat = [], [], []
    for k in range(n):
        elapsed += dt
        if elapsed < beat: continue
        bdt = elapsed; elapsed = 0.0
        dr = 0.0 if first else (rate[k] - prev_r) / bdt
        first = False
        excited = abs(rudder[k]) > 3.0 and (abs(rate[k]) > 0.5 or abs(dr) > 0.2)
        if excited:
            phi = np.array([rudder[k] / d_scale, -dr / dr_scale])
            denom = lam + phi @ P @ phi
            g = P @ phi / denom
            err = rate[k] - phi @ theta
            theta += g * err
            P = (P - np.outer(g, phi) @ P) / lam
            P[0, 0] = max(P[0, 0], 1e-3); P[1, 1] = max(P[1, 1], 1e-3)
            t_beat.append(k * dt); k_hat.append(theta[0] / d_scale); t_hat.append(theta[1] / dr_scale)
        prev_r = rate[k]

    t = np.arange(n) * dt
    fig, axs = plt.subplots(3, 1, figsize=(10.5, 8.5), sharex=False)
    axs[0].plot(t, rudder, color=C_ORANGE, lw=1.2, label="舵角 δ (±20°)")
    axs[0].set_ylabel("舵角 (°)"); axs[0].legend(loc="upper right", fontsize=9)
    axs[0].set_title("① P14 自动 Z 形辨识试验：ManeuverSequencer 生成 ±20°/±20° Z 形舵令")
    axs[1].plot(t, heading, color=C_BLUE, lw=1.2, label="航向 ψ 响应（Nomoto K=0.8, T=12s）")
    axs[1].set_ylabel("航向 (°)"); axs[1].legend(loc="upper right", fontsize=9)
    axs[1].set_title("② 船舶锯齿形航向响应——转向反向瞬态携带 T 的信息")
    axs[2].plot(t_beat, k_hat, color=C_GREEN, lw=1.8, label="K 估计（真值 0.8）")
    axs[2].plot(t_beat, np.array(t_hat) / 20.0, color=C_PURPLE, lw=1.8, label="T 估计/20（真值 12s）")
    axs[2].axhline(0.8, color=C_GREEN, ls="--", lw=0.8); axs[2].axhline(0.6, color=C_PURPLE, ls="--", lw=0.8)
    axs[2].set_xlabel("时间 (s)"); axs[2].set_ylabel("估计值"); axs[2].legend(fontsize=9)
    axs[2].set_title("③ RLS 在线收敛过程（λ=0.98 遗忘因子，逐步逼近真值 K=0.8 / T=12s）→ 收官 establishBaseline()")
    for a_ in axs: a_.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(f"{OUT}/fig5_zigzag_rls.png", bbox_inches="tight"); plt.close(fig)


# ============ 图6：安全机制 / 生命周期 ============
def fig6():
    fig, ax = plt.subplots(figsize=(11, 6.0))
    ax.set_xlim(0, 11); ax.set_ylim(0, 6.0); ax.axis("off")

    box(ax, 0.4, 3.6, 2.0, 1.0, "跟随调度表\n(基线未建立)", fc="#f0fff4", ec=C_GREEN)
    box(ax, 3.4, 3.6, 2.2, 1.0, "基线运行\nESC 窗口微调\n(60s/窗)", fc="#ebf4ff", ec=C_BLUE, bold=False)
    box(ax, 6.6, 3.6, 2.0, 1.0, "显式重基线\n(K/T 漂移>25%)", fc="#fffaf0", ec=C_ORANGE)
    box(ax, 8.9, 1.0, 1.8, 1.0, "挂起 suspend()\n(MANUAL/HOLD)", fc="#f7fafc", ec=C_GRAY)
    box(ax, 3.4, 1.0, 2.2, 1.0, "回退 revert()\n(稳定门不通过)", fc="#fff5f5", ec=C_RED)

    arrow(ax, 2.4, 4.1, 3.4, 4.1, "establishBaseline()\n或辨识评估通过", C_BLUE, toff=(-0.1, 0.08))
    arrow(ax, 5.6, 4.35, 6.6, 4.35, "漂移超阈值", C_ORANGE, toff=(0, 0.05))
    arrow(ax, 7.6, 3.6, 5.6, 3.75, "极点配置+稳定门\n→ 清 ESC 积分器", C_ORANGE, toff=(-0.4, -0.55))
    arrow(ax, 4.5, 3.6, 4.5, 2.0, "窗口结算\n稳定门失败", C_RED, toff=(0.6, 0))
    arrow(ax, 4.5, 2.0, 4.5, 3.6, "恢复基线增益", C_RED, toff=(-1.15, 0))
    arrow(ax, 8.6, 3.9, 9.8, 2.0, "切出自动模式", C_GRAY, toff=(0.35, 0.35))
    arrow(ax, 9.8, 2.0, 8.6, 3.8, "切回：基线保留\n测量状态已清", C_GRAY, toff=(-1.2, -0.75))

    ax.text(5.5, 0.35, "四道防线：① ζ≥0.6 / ωn≤0.5 / osc≤5° 稳定门  ② 失败回退基线  ③ 振荡检测(参考稳定30s后)  ④ 高速>15kn 去激活",
            fontsize=10, color=C_RED, ha="center")
    ax.set_title("基线生命周期与安全回退机制", fontsize=14, pad=10)
    fig.tight_layout(); fig.savefig(f"{OUT}/fig6_lifecycle.png", bbox_inches="tight"); plt.close(fig)


fig1(); fig2(); fig3(); fig4(); fig5(); fig6()
print("figures written to", OUT)
