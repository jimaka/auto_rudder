# -*- coding: utf-8 -*-
"""生成第 04 章（模糊自适应控制自动舵）示意图"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

plt.rcParams['font.sans-serif'] = ['AR PL UMing CN', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei']
plt.rcParams['axes.unicode_minus'] = False

FIGDIR = "/home/jim/Desktop/auto_rudder/docs/adaptive_autopilot_research/figs"

# ------------------------------------------------------------------
# 图 1：模糊自适应 PID 船舶自动舵结构框图
# ------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(11.5, 5.2), dpi=150)
ax.set_xlim(0, 115); ax.set_ylim(0, 52); ax.axis("off")


def box(x, y, w, h, text, fc="#eef4fb", ec="#33608a", fs=11):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                 boxstyle="round,pad=0.4,rounding_size=1.2",
                 fc=fc, ec=ec, lw=1.4))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs)


def arrow(x1, y1, x2, y2, text="", tdx=0, tdy=1.6, color="#333333"):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                 mutation_scale=14, lw=1.4, color=color))
    if text:
        ax.text((x1 + x2) / 2 + tdx, (y1 + y2) / 2 + tdy, text,
                ha="center", va="center", fontsize=10, color=color)


# 主回路
ax.text(4, 26, "$\\psi_d$", fontsize=13, ha="center", va="center")
ax.text(4.2, 20.5, "航向设定", fontsize=9, ha="center", color="#555555")
# 求和点
circ = plt.Circle((10, 26), 1.7, fc="white", ec="#333333", lw=1.4)
ax.add_patch(circ)
ax.text(10, 26, "$-$", fontsize=14, ha="center", va="center")
arrow(5.6, 26, 8.2, 26)

box(15, 22, 16, 8, "微分 / 滤波\n$e,\\ \\dot e$（或转向率 $r$）", fc="#f7f7f7")
arrow(11.7, 26, 15, 26, "航向误差 $e$", tdy=1.8)

box(37, 22, 15, 8, "PID 控制器\n$K_p,K_i,K_d$", fc="#f2fbf2", ec="#2e7d32")
arrow(31, 26, 37, 26)

box(58, 22, 12, 8, "舵机伺服\n（饱和限幅）", fc="#fdf6ec", ec="#b8860b")
arrow(52, 26, 58, 26, "指令舵角 $\\delta_c$", tdy=1.8)

box(76, 22, 15, 8, "船舶操舵响应\nNomoto: $T\\dot r+r=K\\delta$", fc="#f7f7f7")
arrow(70, 26, 76, 26, "实际舵角 $\\delta$", tdy=1.8)
arrow(91, 26, 106, 26, "航向 $\\psi$", tdy=1.8)

# 反馈
ax.plot([106, 106, 10, 10], [26, 6, 6, 24.3], color="#333333", lw=1.4)
ax.add_patch(FancyArrowPatch((10, 6.8), (10, 24.2), arrowstyle="-|>",
             mutation_scale=14, lw=1.4, color="#333333"))

# 波浪干扰
arrow(83.5, 44, 83.5, 30.5, "风浪流干扰", tdx=0, tdy=2.2, color="#a03030")

# 模糊推理上层
box(30, 40, 24, 8, "模糊推理机\n规则库 $R_l$: IF $e$ is $A_l$ AND $\\dot e$ is $B_l$\nTHEN $\\Delta K$ is $C_l$", fc="#fdf0f5", ec="#a04070", fs=10)
arrow(23, 30, 34, 40, "$e,\\dot e$", tdx=-4.5, tdy=-1.2, color="#a04070")
arrow(44, 40, 44, 30.5, "$\\Delta K_p,\\Delta K_i,\\Delta K_d$", tdx=11.5, tdy=0, color="#a04070")
ax.text(40, 49.8, "模糊增益自整定（监督层）：$K_p=K_{p0}+\\Delta K_p$ …",
        fontsize=10.5, color="#a04070", ha="left")

ax.set_title("模糊自整定 PID 船舶自动舵结构框图", fontsize=13, pad=8)
fig.tight_layout()
fig.savefig(f"{FIGDIR}/fig_04_fuzzy_autopilot_structure.png", bbox_inches="tight")
plt.close(fig)

# ------------------------------------------------------------------
# 图 2：模糊规则隶属度函数 + ΔKp 控制面
# ------------------------------------------------------------------
labels = ["NB", "NM", "NS", "ZO", "PS", "PM", "PB"]
centers = np.array([-3, -2, -1, 0, 1, 2, 3], dtype=float)

# 经典 ΔKp 规则表（行: e, 列: ec），数值为输出隶属度中心
rule_table = np.array([
    [3, 3, 2, 2, 1, 0, 0],
    [3, 3, 2, 1, 1, 0, -1],
    [2, 2, 2, 1, 0, -1, -1],
    [2, 2, 1, 0, -1, -2, -2],
    [1, 1, 0, -1, -1, -2, -2],
    [1, 0, -1, -2, -2, -2, -3],
    [0, 0, -1, -2, -2, -3, -3],
], dtype=float)


def tri(x, c):
    """以 c 为中心、半宽 1 的三角隶属度"""
    return np.clip(1.0 - np.abs(x - c), 0.0, 1.0)


def fuzzy_out(e, ec):
    num, den = 0.0, 0.0
    mu_e = np.array([tri(e, c) for c in centers])
    mu_c = np.array([tri(ec, c) for c in centers])
    for i in range(7):
        for j in range(7):
            w = min(mu_e[i], mu_c[j])
            if w > 0:
                num += w * rule_table[i, j]
                den += w
    return num / den if den > 0 else 0.0


x = np.linspace(-3, 3, 121)
X, Y = np.meshgrid(x, x)
Z = np.vectorize(fuzzy_out)(X, Y)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.8), dpi=150,
                               gridspec_kw={"width_ratios": [1, 1.15]})

# 左：隶属度函数
for lab, c in zip(labels, centers):
    ax1.plot(x, tri(x, c), lw=1.5)
    ax1.text(c, 1.06, lab, ha="center", fontsize=9)
ax1.set_xlabel("归一化航向误差 $e$ / 误差变化率 $\\dot e$")
ax1.set_ylabel("隶属度 $\\mu$")
ax1.set_title("(a) 输入变量隶属度函数（七级语言值）")
ax1.set_xlim(-3.4, 3.4); ax1.set_ylim(-0.05, 1.25)
ax1.grid(alpha=0.3)

# 右：控制面
cf = ax2.contourf(X, Y, Z, levels=24, cmap="RdYlBu_r")
cl = ax2.contour(X, Y, Z, levels=[-2, -1, 0, 1, 2], colors="k",
                 linewidths=0.6, alpha=0.5)
ax2.clabel(cl, fmt="%.0f", fontsize=8)
cb = fig.colorbar(cf, ax=ax2, pad=0.02)
cb.set_label("$\\Delta K_p$（归一化）")
ax2.set_xlabel("航向误差 $e$（归一化）")
ax2.set_ylabel("误差变化率 $\\dot e$（归一化）")
ax2.set_title("(b) 模糊规则库生成的 $\\Delta K_p$ 非线性控制面")

fig.suptitle("模糊增益自整定：由规则库到连续非线性增益曲面", fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.94])
fig.savefig(f"{FIGDIR}/fig_04_fuzzy_rule_surface.png", bbox_inches="tight")
plt.close(fig)

print("OK")
