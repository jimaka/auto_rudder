"""从本目录的实际仿真 CSV 生成中文插图；运行：python3 docs/project_overview/generate_figures.py。"""
from pathlib import Path
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/auto-rudder-overview-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch
import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'figures'
OUT.mkdir(exist_ok=True)
font_path = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
if font_path.exists():
    font_manager.fontManager.addfont(str(font_path))
font_name = font_manager.FontProperties(fname=str(font_path)).get_name() if font_path.exists() else 'Noto Sans CJK SC'
plt.rcParams.update({'font.family': font_name, 'axes.unicode_minus': False,
                     'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False})
BLUE, GREEN, ORANGE = '#2563a6', '#168577', '#d58228'

def save(fig, name):
    fig.savefig(OUT / (name + '.png'), dpi=165, bbox_inches='tight', facecolor='white')
    plt.close(fig)

def canvas(title):
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.set(xlim=(0, 12), ylim=(0, 6))
    ax.axis('off')
    ax.set_title(title, fontsize=19, fontweight='bold', pad=18, color='#18334f')
    return fig, ax

def box(ax, x, y, w, h, label, color=BLUE):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.05,rounding_size=0.12',
                              facecolor=color, edgecolor='none'))
    ax.text(x+w/2, y+h/2, label, ha='center', va='center', color='white', fontsize=12)

def arrow(ax, a, b, label=''):
    ax.annotate('', xy=b, xytext=a, arrowprops={'arrowstyle': '-|>', 'color': '#61758a', 'lw': 1.7})
    if label:
        ax.text((a[0]+b[0])/2, (a[1]+b[1])/2+0.12, label, ha='center', fontsize=10)

fig, ax = canvas('工程架构：航向闭环与自适应增益路径')
box(ax, .2, 3.5, 2.1, 1.2, '目标航向 / 机动\n外部操作入口')
box(ax, 3.1, 3.5, 2.3, 1.2, 'AutopilotCore\n模式与每拍编排')
box(ax, 6.2, 3.5, 2.0, 1.2, 'PdController\n航向误差 → 舵令')
box(ax, 9, 3.5, 2.7, 1.2, '舵机与船舶\n实物或 Nomoto 仿真', GREEN)
for a,b in [((2.3,4.1),(3.1,4.1)),((5.4,4.1),(6.2,4.1)),((8.2,4.1),(9,4.1))]: arrow(ax,a,b)
box(ax, .4, .8, 2.3, 1.2, 'GainSchedule\n船速 × 海况', GREEN)
box(ax, 3.5, .8, 2.7, 1.2, 'AutoTuner\n基线 / ESC / 稳定门', ORANGE)
box(ax, 7.1, .8, 2.7, 1.2, 'NomotoIdentifier\n舵角 + 转向率 → K/T 估计', GREEN)
arrow(ax,(2.7,1.4),(3.5,1.4)); arrow(ax,(7.1,1.4),(6.2,1.4))
arrow(ax,(4.8,2),(4.8,3.5),'增益')
arrow(ax,(10.5,3.5),(10.5,2.5)); arrow(ax,(10.5,2.5),(4.1,2.5),'传感器反馈：航向 / 转向率 / 舵角')
arrow(ax,(4.1,2.5),(4.1,3.5)); arrow(ax,(8.5,2.5),(8.5,2))
ax.text(.4,.1,'算法库只输出目标舵角；硬件驱动、传感器接入与任务调度由集成层负责。', color='#506174')
save(fig,'architecture')

fig, ax = canvas('自动调参：辨识建基线，窗口结算后接受或回退')
labels=['传感器观测\nδ、r、ψ、船速','RLS 辨识\n激励门 + 有效性','极点配置\n稳定门 → 基线']
for i,label in enumerate(labels): box(ax,.5+i*4,4.1,3,1,label,BLUE)
arrow(ax,(3.5,4.6),(4.5,4.6)); arrow(ax,(7.5,4.6),(8.5,4.6))
box(ax,8.5,2.1,3,1,'增益叠加载波\nKp 用 sin，Kd 用 cos',ORANGE)
box(ax,4.5,2.1,3,1,'成本累积与解调\n高通 → 相关累积',ORANGE)
box(ax,.5,2.1,3,1,'窗口结算\n低通 → 梯度下降',ORANGE)
arrow(ax,(10,4.1),(10,3.1)); arrow(ax,(8.5,2.6),(7.5,2.6)); arrow(ax,(4.5,2.6),(3.5,2.6))
box(ax,.5,.1,3,1,'稳定门裁决\n通过：接受候选增益',GREEN)
box(ax,4.5,.1,3,1,'不通过：回退基线\n模型漂移：重基线',GREEN)
arrow(ax,(2,2.1),(2,1.1)); arrow(ax,(3.5,.6),(4.5,.6))
ax.text(8.7,.6,'基线与当前增益分开保存\n重锚时清除旧梯度和窗口',va='center',fontsize=11,color='#506174')
save(fig,'tuning_flow')

s = np.genfromtxt(ROOT/'data/step.csv', delimiter=',', names=True)
fig, axs = plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
fig.suptitle('典型船 30° 阶跃：来自 C++ 仿真的实际记录',fontsize=17)
axs[0].plot(s['t'],s['headingRef'],'--',color=ORANGE,label='目标航向')
axs[0].plot(s['t'],s['heading'],color=BLUE,label='实际航向'); axs[0].set_ylabel('航向 / °')
axs[1].plot(s['t'],s['rudderCmd'],color=BLUE,label='舵角指令')
axs[1].plot(s['t'],s['rudderAct'],'--',color=GREEN,label='实际舵角'); axs[1].set_ylabel('舵角 / °')
axs[2].plot(s['t'],s['error'],color=GREEN,label='航向误差'); axs[2].set_ylabel('误差 / °'); axs[2].set_xlabel('仿真时间 / s')
for ax in axs: ax.grid(alpha=.2); ax.legend(loc='upper right')
save(fig,'step_response')

s = np.genfromtxt(ROOT/'data/tune.csv',delimiter=',',names=True)
regular=s[s['settle']==0]; settled=s[s['settle']==1]
fig, axs=plt.subplots(2,2,figsize=(12,8),layout='constrained')
fig.suptitle('1200 秒自动调参试验：辨识、中心增益与窗口代价',fontsize=17)
for ax,key,truth,label in [(axs[0,0],'Khat',.2,'K 估计 / (1/s)'),(axs[0,1],'That',8,'T 估计 / s')]:
    ax.plot(regular['t'],regular[key],color=BLUE,label='辨识值'); ax.axhline(truth,color=ORANGE,ls='--',label='模型真值'); ax.set_ylabel(label)
ax=axs[1,0]
ax.plot(regular['t'],regular['KpCenter'],label='Kp 中心',color=BLUE)
ax.plot(regular['t'],regular['KdCenter'],label='Kd 中心',color=GREEN); ax.set_ylabel('增益（不含载波）')
axs[1,1].plot(settled['t'],settled['Jwindow'],'o-',color=ORANGE,label='结算前记录的窗口 J'); axs[1,1].set_ylabel('窗口代价 J')
for ax in axs.flat: ax.grid(alpha=.2); ax.legend(); ax.set_xlabel('仿真时间 / s')
save(fig,'tune_result')
print('Generated 4 figures:',OUT)
