# -*- coding: utf-8 -*-
"""生成《MPC与船舶节能算法调研文档》配图"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np

plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Droid Sans Fallback"]
plt.rcParams["axes.unicode_minus"] = False

OUT = "docs/ship_energy_research/figures"
import os
os.makedirs(OUT, exist_ok=True)

C_MPC = "#1f77b4"
C_PID = "#d62728"

# ---------------- 图1: MPC vs PID 阶跃航向响应 ----------------
t = np.linspace(0, 30, 600)
# 参考 Jannaty 2023: MPC ~4s 到达, PID ~15s, PID 有明显超调与振荡
def step_resp(t, wn, zeta):
    wd = wn * np.sqrt(1 - zeta**2)
    return 1 - np.exp(-zeta * wn * t) * (np.cos(wd * t) + zeta * wn / wd * np.sin(wd * t))

y_mpc = step_resp(t, 0.9, 0.95)
y_pid = step_resp(t, 0.42, 0.30)

fig, ax = plt.subplots(figsize=(9, 5))
ax.axhline(1.0, color="gray", ls="--", lw=1, label="目标航向指令")
ax.plot(t, y_pid, color=C_PID, lw=2.2, label="传统 PID/PD 自动舵（调节时间 ≈15 s，超调振荡明显）")
ax.plot(t, y_mpc, color=C_MPC, lw=2.2, label="MPC 控制（调节时间 ≈4 s，几乎无超调）")
ax.set_xlabel("时间 / s"); ax.set_ylabel("航向响应（归一化）")
ax.set_title("图1  船舶航向阶跃响应：MPC vs PID（据 Jannaty et al., 2023 仿真结果示意）")
ax.legend(loc="lower right", fontsize=9)
ax.grid(alpha=0.3); ax.set_ylim(-0.1, 1.6)
fig.tight_layout(); fig.savefig(f"{OUT}/fig1_step_response.png", dpi=150); plt.close(fig)

# ---------------- 图2: MPC 滚动优化原理 ----------------
fig, ax = plt.subplots(figsize=(9, 4.5))
tt = np.linspace(0, 10, 400)
ref = np.where(tt >= 2, 1.0, 0.0)
pred = np.zeros_like(tt)
mask = tt >= 2
pred[mask] = 1 - np.exp(-(tt[mask] - 2) / 0.8)
ax.plot(tt, ref, "k--", lw=1.5, label="未来参考航向 r(t)")
ax.plot(tt[tt <= 4.5], pred[tt <= 4.5], color="gray", lw=2.5, label="实际已执行轨迹")
tm = 4.5
ax.plot(tt[tt >= tm], pred[tt >= tm], color=C_MPC, lw=2.5, label="预测模型输出 y_hat(t+k|t)")
ax.axvspan(tm, 9.0, color=C_MPC, alpha=0.10)
ax.annotate("预测时域 $N_p$\n（对未来滚动优化）", xy=(6.8, 0.45), fontsize=11, color=C_MPC, ha="center")
ax.axvline(tm, color="k", ls=":", lw=1)
ax.annotate("当前时刻 t\n（只执行第一个控制量）", xy=(tm, -0.35), fontsize=10, ha="center")
ax.set_xlabel("时间"); ax.set_ylabel("航向")
ax.set_title("图2  MPC 滚动优化（Receding Horizon）原理：利用模型预测未来并处理舵角/舵速约束")
ax.legend(loc="upper left", fontsize=9)
ax.grid(alpha=0.3); ax.set_ylim(-0.6, 1.3)
fig.tight_layout(); fig.savefig(f"{OUT}/fig2_mpc_principle.png", dpi=150); plt.close(fig)

# ---------------- 图3: 调试难度雷达图 ----------------
labels = ["模型依赖程度", "需整定参数数量", "整定规则成熟度", "实时计算负担", "约束处理能力", "抗风浪干扰能力"]
# 分值 1-5，越高表示该维度"负担/能力"越大；前四项为负担（越大越难），后两项为能力（越大越好）
mpc = [5, 4, 2, 4, 5, 5]
pid = [1, 2, 5, 1, 2, 2]
ang = np.linspace(0, 2 * np.pi, len(labels), endpoint=False).tolist()
mpc += mpc[:1]; pid += pid[:1]; ang += ang[:1]
fig, ax = plt.subplots(figsize=(7.5, 6.5), subplot_kw=dict(polar=True))
ax.plot(ang, mpc, color=C_MPC, lw=2, label="MPC")
ax.fill(ang, mpc, color=C_MPC, alpha=0.2)
ax.plot(ang, pid, color=C_PID, lw=2, label="PID/PD")
ax.fill(ang, pid, color=C_PID, alpha=0.2)
ax.set_xticks(ang[:-1]); ax.set_xticklabels(labels, fontsize=10)
ax.set_ylim(0, 5); ax.set_yticks([1, 2, 3, 4, 5]); ax.set_yticklabels(["1", "2", "3", "4", "5"], fontsize=8)
ax.set_title("图3  MPC vs PID/PD 工程特性雷达图（1–5 分）\n前四项为负担维度（越大越难），后两项为能力维度（越大越好）", fontsize=11, pad=22)
ax.legend(loc="lower right", bbox_to_anchor=(1.15, -0.05))
fig.tight_layout(); fig.savefig(f"{OUT}/fig3_radar.png", dpi=150); plt.close(fig)

# ---------------- 图4: 节能措施量级对比 ----------------
measures = ["减速航行\n(slow steaming)\n[IMO/GloMEEP]", "气象航线优化\n[IMO MEPC58\n/StormGeo]", "航速+纵倾\n联合优化\n[ClassNK-NAPA\n实船试航]", "纵倾优化\n[ClassNK/Eniram\n实船]", "自动舵调整\n(减少舵动作)\n[IMO GloMEEP\n/ABS]"]
lo = np.array([10, 2, 2, 1, 0.25])
hi = np.array([50, 5, 4, 5, 1.25])
mid = (lo + hi) / 2
fig, ax = plt.subplots(figsize=(9.5, 5.5))
bars = ax.bar(measures, mid, yerr=[mid - lo, hi - mid], capsize=6,
              color=["#2ca02c", "#1f77b4", "#17becf", "#ff7f0e", "#9467bd"],
              edgecolor="k", error_kw=dict(lw=1.5))
for b, l, h in zip(bars, lo, hi):
    ax.text(b.get_x() + b.get_width()/2, h + 1.2, f"{l:g}–{h:g}%", ha="center", fontsize=10, fontweight="bold")
ax.tick_params(axis="x", labelsize=8.5)
ax.set_ylabel("主机油耗削减潜力 / %")
ax.set_title("图4  船舶节能措施的节油量级对比（区间上下限，来源：IMO / 船级社 / 实船试航）")
ax.set_ylim(0, 56)
ax.grid(axis="y", alpha=0.3)
fig.tight_layout(); fig.savefig(f"{OUT}/fig4_energy_measures.png", dpi=150); plt.close(fig)

# ---------------- 图5: 工业部署案例节油率 ----------------
cases = ["Yara FuelOpt\n(Teekay 油轮)", "ABB OCTOPUS\n(1000+ 船)", "Kongsberg\nEcoAdvisor (DP)", "NAPA 航程优化\n(Marubeni)", "DeepSea+Nabtesco\n(汽车船)", "ClassNK-NAPA\n(K Line 箱船)", "Anschütz 自动舵\n(双舵 toe angle)"]
vals = [4, 9, 9, 7.1, 3.4, 3.9, 5]
notes = ["实测 3–5%", "官方 ≤9%", "DP 工况 8–10%", "实测 ≤7.1%", "实测 3.4%", "试航 3.8–3.9%", "宣称 ≤5%"]
fig, ax = plt.subplots(figsize=(10, 5.2))
bars = ax.barh(cases[::-1], vals[::-1], color="#1f77b4", edgecolor="k")
for b, v, n in zip(bars, vals[::-1], notes[::-1]):
    ax.text(v + 0.15, b.get_y() + b.get_height()/2, n, va="center", fontsize=9)
ax.set_xlabel("宣称/实测节油率 / %")
ax.set_title("图5  船舶节能产品工业部署案例的节油率（厂商口径与第三方实测混合，引用时注意营销偏差）")
ax.set_xlim(0, 11.5)
ax.grid(axis="x", alpha=0.3)
fig.tight_layout(); fig.savefig(f"{OUT}/fig5_deploy_cases.png", dpi=150); plt.close(fig)

# ---------------- 图6: 分层节能架构 ----------------
fig, ax = plt.subplots(figsize=(10, 6))
ax.axis("off")
layers = [
    ("决策层（航次级）", "气象航线优化 / 航速剖面优化 / 到达时间管理（JIT）", "#2ca02c", "节油 2–5%（最大杠杆在减速航行 10–50%）"),
    ("操作层（工况级）", "纵倾优化 / 主机负荷点优化 / 混合动力能量管理（MPC-EMS）", "#17becf", "节油 1–5%，EMS 仿真最高 17%"),
    ("控制层（舵机级）", "自动舵航向/航迹控制：PID / 自适应 / MPC —— 减少无效操舵、降低舵致阻力", "#1f77b4", "节油 0.25–1.25%（双舵 toe angle 特例 ≤5%）"),
    ("执行层", "舵机 / 主机 / 可调螺距桨 / 储能系统", "#7f7f7f", "执行机构与传感器"),
]
y = 0.86
for name, desc, color, note in layers:
    box = FancyBboxPatch((0.05, y - 0.13), 0.66, 0.14, boxstyle="round,pad=0.012",
                         fc=color, ec="k", alpha=0.85)
    ax.add_patch(box)
    ax.text(0.075, y - 0.045, name, fontsize=12, fontweight="bold", color="white")
    ax.text(0.075, y - 0.095, desc, fontsize=9.5, color="white")
    ax.text(0.735, y - 0.06, note, fontsize=9.5, va="center")
    y -= 0.20
for yy in [0.72, 0.52, 0.32]:
    ax.add_patch(FancyArrowPatch((0.38, yy), (0.38, yy - 0.045), arrowstyle="-|>", mutation_scale=18, color="k"))
ax.set_xlim(0, 1); ax.set_ylim(0, 1)
ax.set_title("图6  船舶节能的分层架构：节能大头在决策/操作层，自动舵控制层贡献小但几乎零成本", fontsize=12)
fig.tight_layout(); fig.savefig(f"{OUT}/fig6_architecture.png", dpi=150); plt.close(fig)

print("done:", sorted(os.listdir(OUT)))
