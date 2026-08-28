#!/usr/bin/env python3
# 生成《辨识流程原理推导与工作流程》的全部配图到 docs/auto_tuning/figures/
# 纯 matplotlib，无需额外依赖；python3 gen_ident_figures.py 即可重新生成
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

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "figures")
DATA = os.path.join(HERE, "data")
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


def load_tune_csv(name):
    """读取 --tune 导出的 CSV（样本行）。"""
    path = os.path.join(DATA, name)
    import csv
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if r["settle"] == "1":
                continue
            rows.append({k: float(r[k]) for k in
                         ["t", "psi", "psiRef", "rudderCmd", "rudderAct",
                          "rate", "error", "Kp", "Kd", "KpCenter", "KdCenter",
                          "Khat", "That", "Jwindow"]})
    return rows


# ============ 图1：辨识在自动舵中的位置与数据流 ============
def fig1():
    fig, ax = plt.subplots(figsize=(11, 5.6))
    ax.set_xlim(0, 11); ax.set_ylim(0, 5.6); ax.axis("off")

    box(ax, 0.2, 3.9, 2.3, 1.1, "增益调度表\n(船速/海况查表)", fc="#f0fff4", ec=C_GREEN, fs=9)
    box(ax, 3.1, 3.9, 2.5, 1.1, "AutoTuner\n极点配置+ESC+稳定门", fc="#faf5ff", ec=C_PURPLE, fs=9)
    box(ax, 6.2, 3.9, 2.3, 1.1, "PD 控制器\nδ = Kp·e + Kd·ė", fc="#fffaf0", ec=C_ORANGE, fs=9)
    box(ax, 9.0, 3.9, 1.8, 1.1, "船舶\nNomoto 模型", fc="#f7fafc", ec=C_GRAY, fs=9)
    box(ax, 0.2, 1.2, 2.3, 1.1, "传感器\nAHRS ψ/r、舵角 δ", fc="#f7fafc", ec=C_GRAY, fs=9)
    box(ax, 3.1, 1.2, 2.5, 1.1, "Nomoto 辨识器\nRLS: θ=[K, T]ᵀ", fc="#ebf4ff", ec=C_BLUE, bold=True, fs=9)

    # 主链：调度 → AutoTuner → PD → 船舶（从左到右）
    arrow(ax, 2.5, 4.45, 3.1, 4.45, "名义增益", fs=8.5)
    arrow(ax, 5.6, 4.45, 6.2, 4.45, "增益(Kp,Kd)", fs=8.5)
    arrow(ax, 8.5, 4.45, 9.0, 4.45, "舵指令 δ", fs=8.5)
    # 反馈：船舶 → 传感器（沿底部绕行，不穿任何框）
    arrow(ax, 9.9, 3.9, 9.9, 0.45, "量测 ψ/r/δ", fs=8.5, toff=(0.12, 0))
    arrow(ax, 9.9, 0.45, 1.35, 0.45, "", fs=8.5)
    arrow(ax, 1.35, 0.45, 1.35, 1.2, "", fs=8.5)
    # 传感器 → 辨识器 → AutoTuner
    arrow(ax, 2.5, 1.75, 3.1, 1.75, "δ, r（50 Hz）", fs=8.5)
    arrow(ax, 4.35, 2.3, 4.35, 3.9, "辨识快照 K_hat/T_hat", fs=8.5, toff=(0.12, 0))

    ax.text(5.5, 5.25, "辨识流程在自动舵中的位置：独立模块 + 只读快照，自降频在线估计 K/T",
            ha="center", fontsize=11, color="#1a202c", fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "ident_fig1_position.png"), bbox_inches="tight")
    plt.close(fig)


# ============ 图2：Nomoto 模型与回归化 ============
def fig2():
    fig, ax = plt.subplots(figsize=(10.5, 5.2))
    ax.set_xlim(0, 10.5); ax.set_ylim(0, 5.2); ax.axis("off")

    box(ax, 0.8, 4.0, 4.0, 1.1, "船舶转向动力学\nT·ṙ + r = K·δ", fc="#f7fafc", ec=C_GRAY, fs=10, bold=True)
    box(ax, 5.8, 4.0, 4.0, 1.1, "后向欧拉离散\nr_k = K·δ_k − T·(r_k−r_{k−1})/Δt", fc="#ebf4ff", ec=C_BLUE, fs=10)
    box(ax, 0.8, 1.5, 4.0, 1.4, "线性回归 y = φᵀ·θ\ny=r_k,  φ=[δ_k, −dr_k],  θ=[K, T]", fc="#f0fff4", ec=C_GREEN, fs=10)
    box(ax, 5.8, 1.5, 4.0, 1.4, "RLS 递推最小二乘\n增益/新息/协方差三件套\n带遗忘因子 λ", fc="#faf5ff", ec=C_PURPLE, fs=10)

    arrow(ax, 4.8, 4.55, 5.8, 4.55, "微分方程 → 代数方程", fs=8.5)
    arrow(ax, 5.8, 4.0, 4.8, 2.9, "离散化 → 回归", fs=8.5)
    arrow(ax, 4.8, 2.2, 5.8, 2.2, "递推求解", fs=8.5)
    ax.text(5.25, 0.55, "激励门控：|δ|>3° 且 (|r|>0.5°/s 或 |dr|>0.2°/s²) 才更新",
            ha="center", fontsize=9, color=C_RED)

    ax.text(5.25, 4.9, "辨识问题的两步转换：先把一阶动力学写成线性回归，再用 RLS 在线求解",
            ha="center", fontsize=11, color="#1a202c", fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "ident_fig2_regression.png"), bbox_inches="tight")
    plt.close(fig)


# ============ 图3：Z 形试验 + RLS 收敛（真实仿真数据） ============
def fig3():
    rows = load_tune_csv("tune_convergence_k0p2_t8.csv")
    t = np.array([r["t"] for r in rows])
    psi = np.array([r["psi"] for r in rows])
    ref = np.array([r["psiRef"] for r in rows])
    delta = np.array([r["rudderAct"] for r in rows])
    rate = np.array([r["rate"] for r in rows])
    khat = np.array([r["Khat"] for r in rows])
    that = np.array([r["That"] for r in rows])

    fig = plt.figure(figsize=(11, 10))
    gs = fig.add_gridspec(4, 1, hspace=0.45, left=0.08, right=0.96, top=0.93, bottom=0.07)
    ax1 = fig.add_subplot(gs[0])
    m = t <= 240
    ax1.plot(t[m], ref[m], "k--", lw=1.0, label="参考 ψ_ref（Z 形 ±40°）")
    ax1.plot(t[m], psi[m], "C0", lw=1.2, label="航向 ψ")
    ax1.set_ylabel("航向 (°)"); ax1.legend(loc="upper right", fontsize=9); ax1.grid(alpha=0.3)
    ax1.set_title("Z 形激励下的船舶响应（K=0.2/T=8，10 kn，前 240 s）", fontsize=11)

    ax2 = fig.add_subplot(gs[1])
    ax2.plot(t[m], delta[m], "C1", lw=1.1, label="实际舵角 δ")
    ax2.plot(t[m], rate[m], "C2", lw=1.0, alpha=0.8, label="转向率 r")
    ax2.set_ylabel("δ (°) / r (°/s)"); ax2.legend(loc="upper right", fontsize=9); ax2.grid(alpha=0.3)

    ax3 = fig.add_subplot(gs[2])
    ax3.plot(t, khat, "C0", lw=1.2, label="K_hat (RLS)")
    ax3.axhline(0.2, color="C3", ls="--", lw=1.2, label="真值 K=0.2")
    ax3.set_ylabel("K_hat (1/s)"); ax3.legend(loc="upper right", fontsize=9); ax3.grid(alpha=0.3)
    ax3.set_title("RLS 在线收敛（K_hat/T_hat 逐步逼近真值）", fontsize=11)

    ax4 = fig.add_subplot(gs[3])
    ax4.plot(t, that, "C1", lw=1.2, label="T_hat (RLS)")
    ax4.axhline(8.0, color="C3", ls="--", lw=1.2, label="真值 T=8 s")
    ax4.set_ylabel("T_hat (s)"); ax4.set_xlabel("时间 (s)")
    ax4.legend(loc="upper right", fontsize=9); ax4.grid(alpha=0.3)

    fig.savefig(os.path.join(OUT, "ident_fig3_zigzag_rls.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


# ============ 图4：自降频 + 激励门控 ============
def fig4():
    fig, ax = plt.subplots(figsize=(10.8, 5.6))
    ax.set_xlim(0, 10.8); ax.set_ylim(0, 5.6); ax.axis("off")

    box(ax, 0.4, 4.5, 3.0, 1.1, "主循环 50 Hz\nupdate(δ, r, dtNow=0.02s)", fc="#fffaf0", ec=C_ORANGE, fs=9)
    box(ax, 4.2, 4.5, 2.6, 1.1, "入口防御\n非有限/超限/dt≤0 丢弃", fc="#fff5f5", ec=C_RED, fs=9)
    box(ax, 7.6, 4.5, 2.8, 1.1, "累计 elapsed\n满 beatSec=1 s 一拍", fc="#f0fff4", ec=C_GREEN, fs=9)
    box(ax, 0.4, 2.5, 4.8, 1.2, "激励门控：|δ|>3° 且 (|r|>0.5°/s 或 |dr|>0.2°/s²)\n不满足 → 仅记录 prevR，跳过 θ/P（防协方差爆炸）", fc="#f0fff4", ec=C_GREEN, fs=9)
    box(ax, 5.8, 2.5, 2.6, 1.2, "坏数据：\n丢弃本拍（不写 prevR）", fc="#fff5f5", ec=C_RED, fs=9)
    box(ax, 0.4, 0.7, 4.8, 1.1, "RLS 拍：φ → 增益 K_k → θ ← θ+K·e\n→ P 更新（对角元保底 1e-3）", fc="#faf5ff", ec=C_PURPLE, fs=9)
    box(ax, 6.0, 0.7, 4.4, 1.1, "物理合理性 valid()：\nK_hat∈(0,2]、T_hat∈[1,60]s", fc="#ebf4ff", ec=C_BLUE, fs=9)

    arrow(ax, 3.4, 5.05, 4.2, 5.05, "每拍", fs=8.5)
    arrow(ax, 6.8, 5.05, 7.6, 5.05, "合法", fs=8.5)
    arrow(ax, 7.6, 4.5, 5.2, 3.7, "满 1 s", fs=8.5)
    arrow(ax, 5.5, 4.5, 5.5, 3.1, "非法", fs=8.5, color=C_RED, toff=(0.12, 0))
    arrow(ax, 5.5, 3.1, 5.8, 3.1, "", fs=8.5, color=C_RED)
    arrow(ax, 2.7, 2.5, 2.7, 1.8, "满足 → RLS 拍", fs=8.5, toff=(0.1, 0))
    arrow(ax, 5.2, 1.25, 6.0, 1.25, "快照", fs=8.5)

    ax.text(5.4, 5.3, "自降频与数据质量门控：调用频率无关、差分跨真实拍长、无信息样本不污染估计",
            ha="center", fontsize=11, color="#1a202c", fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "ident_fig4_gating.png"), bbox_inches="tight")
    plt.close(fig)


# ============ 图5：RLS 单拍递推流程 ============
def fig5():
    fig, ax = plt.subplots(figsize=(11, 6.8))
    ax.set_xlim(0, 11); ax.set_ylim(0, 6.8); ax.axis("off")

    # 三行三列蛇形布局：行内从左到右，换行经斜线连接到下一行最左框
    box(ax, 0.4, 5.0, 2.5, 1.0, "reset()\nθ=[0,0], P=p0·I", fc="#f7fafc", ec=C_GRAY, fs=9)
    box(ax, 3.6, 5.0, 2.5, 1.0, "update(δ,r,dtNow)\n入口防御：非法/超限丢弃", fc="#fff5f5", ec=C_RED, fs=9)
    box(ax, 6.8, 5.0, 2.5, 1.0, "累计 elapsed\n满 beatSec=1 s", fc="#fffaf0", ec=C_ORANGE, fs=9)
    box(ax, 0.4, 3.0, 2.5, 1.0, "激励门控\n不满足→仅记录 prevR", fc="#f0fff4", ec=C_GREEN, fs=9)
    box(ax, 3.6, 3.0, 2.5, 1.0, "构造回归量\nφ=[δ/50, −dr/1]ᵀ, y=r", fc="#ebf4ff", ec=C_BLUE, fs=9)
    box(ax, 6.8, 3.0, 2.5, 1.0, "增益与残差\nK_k=Pφ/(λ+φᵀPφ), e=y−φᵀθ", fc="#ebf4ff", ec=C_BLUE, fs=9)
    box(ax, 0.4, 1.0, 2.5, 1.0, "更新 θ\nθ ← θ + K_k·e", fc="#faf5ff", ec=C_PURPLE, fs=9)
    box(ax, 3.6, 1.0, 2.5, 1.0, "更新 P\nP ← (P−K_kφᵀP)/λ + floor", fc="#faf5ff", ec=C_PURPLE, fs=9)
    box(ax, 6.8, 1.0, 2.5, 1.0, "快照输出\nK_hat/T_hat/excited/valid", fc="#f0fff4", ec=C_GREEN, fs=9)

    arrow(ax, 2.9, 5.5, 3.6, 5.5, "建对象", fs=8.5)
    arrow(ax, 6.1, 5.5, 6.8, 5.5, "每拍", fs=8.5)
    arrow(ax, 6.8, 5.0, 2.9, 4.0, "满 1 s", fs=8.5)
    arrow(ax, 2.9, 4.0, 3.6, 4.0, "", fs=8.5)
    arrow(ax, 6.1, 4.0, 6.8, 4.0, "", fs=8.5)
    arrow(ax, 6.8, 3.0, 2.9, 2.0, "递推", fs=8.5)
    arrow(ax, 2.9, 2.0, 3.6, 2.0, "", fs=8.5)
    arrow(ax, 6.1, 2.0, 6.8, 2.0, "", fs=8.5)
    arrow(ax, 2.9, 1.5, 3.6, 1.5, "", fs=8.5)
    arrow(ax, 6.1, 1.5, 6.8, 1.5, "", fs=8.5)

    ax.text(5.5, 6.4, "RLS 单拍递推：一个完整更新周期内的数据流与状态机",
            ha="center", fontsize=11, color="#1a202c", fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "ident_fig5_rls_flow.png"), bbox_inches="tight")
    plt.close(fig)


# ============ 图6：辨识 → 应用工作流 ============
def fig6():
    fig, ax = plt.subplots(figsize=(11.5, 5.8))
    ax.set_xlim(-0.4, 11.5); ax.set_ylim(0, 5.8); ax.axis("off")

    box(ax, 0.3, 4.5, 2.4, 1.1, "每拍喂数据\nidentifier.update(δ,r,dt)", fc="#fffaf0", ec=C_ORANGE, fs=9)
    box(ax, 3.3, 4.5, 2.4, 1.1, "RLS 满拍更新\nK_hat/T_hat", fc="#ebf4ff", ec=C_BLUE, fs=9)
    box(ax, 6.3, 4.5, 2.4, 1.1, "只读快照 IdSnapshot\n{excited,valid,K,T}", fc="#ebf4ff", ec=C_BLUE, fs=9)
    box(ax, 9.3, 4.5, 1.9, 1.1, "tick()\n周期评估", fc="#faf5ff", ec=C_PURPLE, fs=9)

    box(ax, 0.3, 2.0, 2.4, 1.1, "漂移门（P12）\n相对基线 ≤25% 不动", fc="#f0fff4", ec=C_GREEN, fs=9)
    box(ax, 3.3, 2.0, 2.4, 1.1, "极点配置\nKp=ωn²T/K, Kd=(2ζωnT−1)/K", fc="#faf5ff", ec=C_PURPLE, fs=9)
    box(ax, 6.3, 2.0, 2.4, 1.1, "稳定门\nζ≥0.6, ωn≤0.5, osc≤5°", fc="#f0fff4", ec=C_GREEN, fs=9)
    box(ax, 9.3, 2.0, 1.9, 1.1, "建立/重锚\n基线+清ESC", fc="#faf5ff", ec=C_PURPLE, fs=9)

    box(ax, 0.3, 0.3, 5.0, 1.0, "Z 形辨识试验（P14）：机动中喂辨识器 → 收官 establishBaseline()", fc="#f0fff4", ec=C_GREEN, fs=9)

    arrow(ax, 2.7, 5.05, 3.3, 5.05, "", fs=8.5)
    arrow(ax, 5.7, 5.05, 6.3, 5.05, "", fs=8.5)
    arrow(ax, 8.7, 5.05, 9.3, 5.05, "每拍", fs=8.5)
    # tick → 漂移门：肘形绕行（先下、再左、后落框），不穿越第二行框
    arrow(ax, 10.25, 4.5, 10.25, 3.55, "有效快照", fs=8.5, toff=(0.12, 0))
    arrow(ax, 10.25, 3.55, 1.5, 3.55, "漂移超阈值", fs=8.5)
    arrow(ax, 1.5, 3.55, 1.5, 3.1, "", fs=8.5)
    # 第二行：漂移门 → 极点配置 → 稳定门 → 建立/重锚（从左到右）
    arrow(ax, 2.7, 2.55, 3.3, 2.55, "", fs=8.5)
    arrow(ax, 5.7, 2.55, 6.3, 2.55, "", fs=8.5)
    arrow(ax, 8.7, 2.55, 9.3, 2.55, "通过", fs=8.5)
    # Z 形试验 → 每拍喂数据：沿左边缘绕行
    arrow(ax, 0.8, 1.3, 0.08, 1.3, "", fs=8.5)
    arrow(ax, 0.08, 1.3, 0.08, 5.05, "Z 形试验", fs=8.5, toff=(-0.18, 0))
    arrow(ax, 0.08, 5.05, 0.3, 5.05, "", fs=8.5)

    ax.text(5.75, 5.5, "辨识结果的两种应用路径：tick 周期评估（漂移门）与 Z 形试验收官（一次性建基线）",
            ha="center", fontsize=11, color="#1a202c", fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "ident_fig6_workflow.png"), bbox_inches="tight")
    plt.close(fig)


# ============ 图7：慢船激励适配对比 ============
def fig7():
    r15 = load_tune_csv("tune_convergence_k0p1_t20.csv")
    r60 = load_tune_csv("tune_convergence_k0p1_t20_leg60.csv")
    fig, axs = plt.subplots(2, 1, figsize=(11, 6.6), sharex=True)
    for rows, lab, c in [(r15, "leg=15 s（默认）", C_RED), (r60, "leg=60 s（≥2T 适配）", C_GREEN)]:
        t = np.array([r["t"] for r in rows])
        khat = np.array([r["Khat"] for r in rows])
        that = np.array([r["That"] for r in rows])
        axs[0].plot(t, khat, c, lw=1.1, label=lab)
        axs[1].plot(t, that, c, lw=1.1, label=lab)
    axs[0].axhline(0.1, color="k", ls="--", lw=1.0, label="真值 K=0.1")
    axs[1].axhline(20.0, color="k", ls="--", lw=1.0, label="真值 T=20 s")
    axs[0].set_ylabel("K_hat (1/s)"); axs[0].legend(loc="upper right", fontsize=9); axs[0].grid(alpha=0.3)
    axs[1].set_ylabel("T_hat (s)"); axs[1].set_xlabel("时间 (s)")
    axs[1].legend(loc="upper right", fontsize=9); axs[1].grid(alpha=0.3)
    axs[0].set_title("慢船（K=0.1/T=20）激励适配对比：翻转周期必须 ≥ 2T 才可辨识",
                     fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "ident_fig7_slow_ship.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    fig1(); fig2(); fig3(); fig4(); fig5(); fig6(); fig7()
    print("已生成 ident_fig1~fig7 到", OUT)
