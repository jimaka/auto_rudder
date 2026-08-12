# -*- coding: utf-8 -*-
"""生成 12 章节示意图：验证 V 模型（SIL→HIL→水池→实船）+ HIL 仿真器架构图"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

plt.rcParams['font.sans-serif'] = ['AR PL UMing CN', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei']
plt.rcParams['axes.unicode_minus'] = False

OUT = '/home/jim/Desktop/auto_rudder/docs/adaptive_autopilot_research/figs/'

def box(ax, x, y, w, h, text, fc='#EAF2FB', ec='#2B5C9E', fs=10.5):
    b = FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.5',
                       facecolor=fc, edgecolor=ec, linewidth=1.5)
    ax.add_patch(b)
    if text:
        ax.text(x + w/2, y + h/2, text, ha='center', va='center', fontsize=fs)

def arrow(ax, x1, y1, x2, y2, text='', fs=9, color='k', dy=1.4, ls='-'):
    a = FancyArrowPatch((x1, y1), (x2, y2), arrowstyle='-|>',
                        mutation_scale=13, lw=1.3, color=color, linestyle=ls)
    ax.add_patch(a)
    if text:
        ax.text((x1+x2)/2, (y1+y2)/2 + dy, text, ha='center', fontsize=fs, color=color)

# ---------------- 图1：V 模型验证层级 ----------------
fig, ax = plt.subplots(figsize=(11.5, 7.2))
ax.set_xlim(0, 100); ax.set_ylim(0, 74); ax.axis('off')

# 左臂：设计/开发（自上而下）
box(ax, 3, 48, 26, 8, '需求定义\n性能指标/安全约束/运行域', fs=10)
box(ax, 3, 34, 26, 8, '算法设计\nPD + 增益调度 + ESC\n（极点配置基线）', fs=10)
box(ax, 3, 20, 26, 8, '软件实现\n目标代码 / 参数表', fs=10)

# 右臂：验证（自下而上），对应层级
box(ax, 71, 20, 26, 8, 'SIL 软件在环\n算法逻辑/单元测试\n批量工况扫描', fc='#FDF3E3', ec='#B07B1E', fs=10)
box(ax, 71, 34, 26, 8, 'HIL 硬件在环\n实时性/接口/故障注入\n蒙特卡洛鲁棒性', fc='#FDF3E3', ec='#B07B1E', fs=10)
box(ax, 71, 48, 26, 8, '水池试验\n模型尺度下验证\n控制-水动力耦合', fc='#FDF3E3', ec='#B07B1E', fs=10)
box(ax, 71, 62, 26, 8, '实船试航\nZ 形试验/保向精度\n限海况逐步放开', fc='#FDF3E3', ec='#B07B1E', fs=10)

# 底部横连：实现 → SIL
arrow(ax, 29, 24, 71, 24, '同一代码 / 同一参数集', fs=9.5)

# 左臂纵向
arrow(ax, 16, 48, 16, 42); arrow(ax, 16, 34, 16, 28)
# 右臂纵向（自 SIL 向上）
arrow(ax, 84, 28, 84, 34); arrow(ax, 84, 42, 84, 48); arrow(ax, 84, 56, 84, 62)

# 对应关系（水平虚线：验证结论回溯）
arrow(ax, 71, 38, 29, 38, '接口/实时性缺陷回溯修改设计', fs=8.5, color='#888888', ls='--', dy=1.2)
arrow(ax, 71, 66, 29, 52, '试航数据校核模型与需求（模型更新/包线修订）', fs=8.5, color='#888888', ls='--', dy=1.2)

ax.text(16, 59.5, '开 发 侧', fontsize=12, ha='center', weight='bold', color='#2B5C9E')
ax.text(84, 72.5, '验 证 侧', fontsize=12, ha='center', weight='bold', color='#B07B1E')
ax.text(50, 8, '验证层级逐级逼近真实物理：成本与风险递增、模型保真度要求递增；\n绝大多数缺陷应在 SIL/HIL 阶段暴露，水池与实船只做最终确认',
        fontsize=10.5, ha='center', color='#444444')
ax.set_title('自适应自动舵 V 模型验证层级（SIL → HIL → 水池 → 实船）', fontsize=13)
fig.savefig(OUT + 'fig_12_vmodel.png', dpi=160, bbox_inches='tight')
plt.close(fig)

# ---------------- 图2：HIL 仿真器架构 ----------------
fig, ax = plt.subplots(figsize=(12, 7))
ax.set_xlim(0, 110); ax.set_ylim(0, 64); ax.axis('off')

# 左：被测对象（真实自动舵硬件/目标机）
box(ax, 2, 24, 22, 26, '', fc='#F2F7EC', ec='#4C7A34')
ax.text(13, 46.5, '被测对象（DUT）', ha='center', fontsize=11.5, weight='bold', color='#4C7A34')
box(ax, 4, 36, 18, 6.5, '自动舵控制器\nPD + 增益调度 + ESC\n（目标硬件/实时代码）', fc='#FFFFFF', ec='#4C7A34', fs=9.5)
box(ax, 4, 26.5, 18, 6.5, 'I/O 与通信接口\nNMEA 0183/2000、CAN\n模拟量/开关量', fc='#FFFFFF', ec='#4C7A34', fs=9.5)

# 右：实时仿真器
box(ax, 46, 4, 62, 56, '')
ax.text(77, 56.5, '实时 HIL 仿真器（船舶数字孪生，定步长实时求解）', ha='center', fontsize=11.5, weight='bold', color='#2B5C9E')

box(ax, 49, 41, 56, 11, '环境扰动模型\n波浪谱（ITTC/JONSWAP）：一阶波频力 + 二阶慢漂力/力矩；风、流\n遭遇频率随航速/浪向变换，海况/浪向/风速可场景化设定', fs=9.5)
box(ax, 49, 25, 26, 12, '舵机模型\n伺服动态 + 饱和 ±35°\n舵速限制 / 间隙\n液压故障（卡舵、泵失效）', fs=9.5)
box(ax, 79, 25, 26, 12, '船舶动力学模型\n3/4/6 自由度操纵模型\nNomoto K/T 或水动力导数\n舵效非线性（失速）', fs=9.5)
box(ax, 49, 8, 26, 12, '传感器模型\n罗经/IMU/计程仪\n噪声、延迟、漂移、丢包\n故障注入（冻结、跳变）', fs=9.5)
box(ax, 79, 8, 26, 12, '测试管理与评估\n场景库 / 故障注入脚本\n蒙特卡洛批跑\n指标统计与自动判读', fs=9.5, fc='#FDF3E3', ec='#B07B1E')

# 信号流（顺时针环：DUT → 舵机 → 船舶 → 传感器 → DUT）
arrow(ax, 24, 39, 49, 33, '', fs=9)
ax.text(36, 38.5, '舵角指令 $\\delta_c$', ha='center', fontsize=9.5)
arrow(ax, 75, 31, 79, 31, '', fs=9)
ax.text(77, 38.6, '实际舵角 $\\delta$（饱和/舵速限制后）', ha='center', fontsize=8.5)
arrow(ax, 92, 41, 92, 37.3, '', fs=9)
arrow(ax, 88, 25, 68, 20.3, '', fs=9)
ax.text(81, 21.2, '$\\psi, r, U$', ha='center', fontsize=9)
arrow(ax, 49, 13, 24, 28.5, '', fs=9)
ax.text(35, 16.5, '艏向 ψ、转首角速度 r、航速 U\n（含噪声/延迟/故障注入）', ha='center', fontsize=8.5)

ax.set_title('自动舵 HIL 仿真器构成：被测控制器 + 实时船舶/环境/舵机/传感器模型 + 测试管理', fontsize=13)
fig.savefig(OUT + 'fig_12_hil_architecture.png', dpi=160, bbox_inches='tight')
plt.close(fig)
print('done')
