# -*- coding: utf-8 -*-
"""生成 11 章配图：自适应/学习型 MPC 架构框图 + 嵌入式计算预算示意图"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

plt.rcParams['font.sans-serif'] = ['AR PL UMing CN', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei']
plt.rcParams['axes.unicode_minus'] = False

# ---------------- 图1：自适应/学习型 MPC 架构框图 ----------------
fig, ax = plt.subplots(figsize=(11.5, 6.2))
ax.set_xlim(0, 100); ax.set_ylim(0, 56); ax.axis('off')


def box(x, y, w, h, text, fc='#EAF2FB', ec='#2B5C9E', fs=10.5):
    b = FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.6',
                       facecolor=fc, edgecolor=ec, linewidth=1.6)
    ax.add_patch(b)
    ax.text(x + w / 2, y + h / 2, text, ha='center', va='center', fontsize=fs)


def arrow(x1, y1, x2, y2, text='', fs=9.5, dy=1.6, color='k', style='-|>', curve=0.0):
    a = FancyArrowPatch((x1, y1), (x2, y2), arrowstyle=style,
                        mutation_scale=13, lw=1.4, color=color,
                        connectionstyle=f'arc3,rad={curve}')
    ax.add_patch(a)
    if text:
        ax.text((x1 + x2) / 2, (y1 + y2) / 2 + dy, text, ha='center', fontsize=fs, color=color)


# 主回路（自左向右）
ax.text(4, 40, r'$\psi_d$', fontsize=14, ha='center', va='center')
ax.text(4, 36, '（LOS 导引）', fontsize=9, ha='center', va='center', color='#555555')

box(10, 36, 16, 9, '状态估计与波浪滤波\n（KF / EKF）')
box(33, 36, 17, 9, 'MPC 优化器\n滚动时域 QP / SQP\n约束：舵角、舵速', fc='#FDF3E3', ec='#B0772A')
box(57, 36, 13, 9, '安全监督层\n约束收紧 / tube\n回退 PD 兜底', fc='#FBEAEA', ec='#9E2B2B')
box(77, 36, 9, 9, '舵机\n（限幅）')
box(90, 36, 9, 9, '船舶\n动力学')

# 学习支路（上方）
box(26, 6, 30, 10, '在线辨识 / 模型学习\nRLS（Nomoto K、T）\nGP / SINDy / Koopman 提升模型', fc='#EAF8EC', ec='#2B7A3E')

# 传感器
box(45, 24, 14, 7, '罗经 / GPS / IMU', fc='#F2F2F2', ec='#666666', fs=10)

# 箭头
arrow(6.5, 40.5, 9.5, 40.5)
arrow(26, 40.5, 32.5, 40.5, r'$\hat{\psi},\ \hat{r}$')
arrow(50, 40.5, 56.5, 40.5, r'$\delta_c$ 序列')
arrow(70, 40.5, 76.5, 40.5, r'$\delta_c$')
arrow(86, 40.5, 89.5, 40.5)
arrow(94.5, 36, 94.5, 27.5, style='-')
arrow(94.5, 27.5, 59, 27.5, style='-')
arrow(59, 27.5, 59, 31, style='-|>')
# 传感器 -> 状态估计
arrow(45, 27.5, 22, 27.5, style='-')
arrow(22, 27.5, 22, 35.5, style='-|>', text='量测', dy=0.4)
# 量测 -> 学习
arrow(45, 26, 41, 16.5, curve=-0.2)
# 舵角指令 -> 学习（闭环数据）
arrow(63.5, 36, 56, 16.5, text='闭环数据\n$(\\delta,\\psi,r)$', dy=-4.5, curve=-0.2)
# 学习 -> MPC：更新预测模型
arrow(41, 16.5, 41, 35.5, text='更新预测模型', color='#2B7A3E', dy=0.4)
# 安全监督层与 MPC 的交互（约束收紧回传）
arrow(60, 36, 48, 33.5, text='收紧约束集', dy=-3.2, color='#9E2B2B', curve=-0.15)

ax.text(50, 1.5, '学习支路（绿）：在线更新预测模型；安全支路（红）：约束收紧与故障回退',
        ha='center', fontsize=10, color='#444444')

fig.tight_layout()
fig.savefig('/home/jim/Desktop/auto_rudder/docs/adaptive_autopilot_research/figs/fig_11_adaptive_mpc_structure.png',
            dpi=160, bbox_inches='tight')
plt.close(fig)

# ---------------- 图2：嵌入式计算预算示意 ----------------
items = [
    ('HPIPM 求解 QP\n（nx=3, N=20）', 0.5, '#2B7A3E'),
    ('qpOASES（暖启动）', 1.0, '#2B7A3E'),
    ('acados RTI\n（NMPC 单迭代）', 1.5, '#2B7A3E'),
    ('OSQP（ADMM）', 2.0, '#2B7A3E'),
    ('稀疏 GP 在线推理', 3.0, '#B0772A'),
    ('非稀疏 GP 推理\n（500 样本）', 20.0, '#9E2B2B'),
]
labels = [i[0] for i in items]
vals = [i[1] for i in items]
colors = [i[2] for i in items]

fig, ax = plt.subplots(figsize=(10.5, 5.4))
y = np.arange(len(items))
ax.barh(y, vals, color=colors, alpha=0.85, height=0.6)
for yi, v in zip(y, vals):
    ax.text(v * 1.12, yi, f'{v:g} ms', va='center', fontsize=10)

ax.axvline(50, color='#2B5C9E', ls='--', lw=1.6)
ax.text(50, len(items) - 0.1, ' 20 Hz 控制周期预算 50 ms', fontsize=10, color='#2B5C9E', va='bottom')
ax.axvline(100, color='#2B5C9E', ls=':', lw=1.6)
ax.text(100, -0.62, ' 10 Hz：100 ms', fontsize=10, color='#2B5C9E', va='top')

ax.set_xscale('log')
ax.set_xlim(0.3, 400)
ax.set_yticks(y)
ax.set_yticklabels(labels, fontsize=10)
ax.set_xlabel('单次计算耗时（ms，对数坐标）', fontsize=11)
ax.set_title('25 kn 游艇航向 MPC 嵌入式计算预算量级示意\n（依据 HPIPM/acados 公开基准量级，非实船实测）', fontsize=12)
ax.grid(axis='x', which='both', ls=':', alpha=0.5)

fig.tight_layout()
fig.savefig('/home/jim/Desktop/auto_rudder/docs/adaptive_autopilot_research/figs/fig_11_mpc_runtime_budget.png',
            dpi=160, bbox_inches='tight')
plt.close(fig)
print('done')
