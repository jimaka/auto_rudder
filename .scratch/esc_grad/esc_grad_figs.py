# -*- coding: utf-8 -*-
# 《ESC代价函数梯度计算详解》配图（esc_grad_ 前缀，2 张）
# 风格与 .scratch/esc_math_figs.py 一致；参数全部取自
# control/include/auto_tuner.hpp（a=0.05, ω=0.05rad/s, η=0.02,
# τH=5s, τL=10s, Tw=60s, dt=0.02s, ±10% 限幅）
import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager as fm
for _p in ('.scratch/fonts/NotoSansCJKsc-Regular.otf',
           '.scratch/fonts/NotoSansCJKsc-Bold.otf'):
    fm.fontManager.addfont(_p)
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

plt.rcParams.update({
    'font.sans-serif': ['Noto Sans CJK SC'],
    'font.family': 'sans-serif',
    'axes.unicode_minus': False,
    'figure.facecolor': 'white',
    'axes.facecolor': 'white',
    'savefig.facecolor': 'white',
    'axes.grid': True,
    'grid.alpha': 0.3,
    'grid.linestyle': '--',
    'axes.spines.top': False,
    'axes.spines.right': False,
    'font.size': 11,
    'axes.titlesize': 13,
    'axes.labelsize': 11.5,
})
C = dict(blue='#4C72B0', orange='#DD8452', green='#55A868', red='#C44E52',
         purple='#8172B3', gray='#8C8C8C', teal='#64B5CD', yellow='#CCB974')

# ---- 与 auto_tuner.hpp Params 一致 ----
A_REL   = 0.05    # escAmp
OMEGA   = 0.05    # escFreq (rad/s)
K_STEP  = 0.02    # escStep
TAU_H   = 5.0     # escHPFtau
TAU_L   = 10.0    # escLPFtau
WINDOW  = 60.0    # windowSec
DT      = 0.02    # dt
MAXFRAC = 0.10    # maxUpdateFrac

def save(fig, name):
    fig.savefig(f'assets/{name}', dpi=200, bbox_inches='tight',
                facecolor='white')
    plt.close(fig)
    print('saved', name)

# ============================================================
# 图1 esc_grad_pipeline.png —— 梯度计算信号流水线框图（带代码行号）
# ============================================================
def fig1():
    fig, ax = plt.subplots(figsize=(15.0, 9.5))
    ax.set_xlim(-2.75, 15.4); ax.set_ylim(-1.8, 11.35)
    ax.axis('off'); ax.grid(False)

    FC = dict(blue='#eaf1fb', orange='#fdf3e7', green='#eaf6ee',
              purple='#f1edf9', red='#fbeaea', gray='#f2f2f2')

    def box(x, y, w, h, text, color, fs=10):
        p = FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.09',
                           fc=FC[color], ec=C[color], lw=1.8, zorder=3)
        ax.add_patch(p)
        ax.text(x + w / 2, y + h / 2, text, ha='center', va='center',
                fontsize=fs, zorder=4, linespacing=1.55)
        return (x, y, w, h)

    def arrow(p1, p2, color=C['gray'], ls='-', lw=1.8, label=None, lx=0, ly=0,
              lfs=10, lcol=None, rad=0.0):
        a = FancyArrowPatch(p1, p2, arrowstyle='-|>', mutation_scale=16,
                            color=color, lw=lw, linestyle=ls, zorder=2,
                            connectionstyle=f'arc3,rad={rad}')
        ax.add_patch(a)
        if label:
            ax.text((p1[0] + p2[0]) / 2 + lx, (p1[1] + p2[1]) / 2 + ly, label,
                    fontsize=lfs, color=lcol or color, ha='center',
                    va='center', zorder=4,
                    bbox=dict(fc='white', ec='none', alpha=0.85, pad=1.2))

    def right(b):  return (b[0] + b[2], b[1] + b[3] / 2)
    def left(b):   return (b[0], b[1] + b[3] / 2)
    def top(b):    return (b[0] + b[2] / 2, b[1] + b[3])
    def bottom(b): return (b[0] + b[2] / 2, b[1])

    W, H, yt, yb = 3.05, 2.0, 5.9, 2.2
    xs = [0.35, 4.15, 7.95, 11.75]

    # 顶行（每拍，左→右）
    b1 = box(xs[0], yt, W, H,
             '① 成本增量（每拍）\n$c_k$=(t·|e|+wU·|δ|\n+wDu·|dδ|)·dt\n'
             'accumulate() :366', 'green')
    b2 = box(xs[1], yt, W, H,
             '② 一阶 HPF 去直流\ny ← αH·(y+$c_k$−$c_{k-1}$)\n'
             'αH=τH/(τH+dt)，τH=5 s\naccumulate() :369', 'purple')
    b3 = box(xs[2], yt, W, H,
             '③ × 同相载波（两路）\nKp 路：y·sinφ\nKd 路：y·cosφ\n'
             '载波由 escDither() 注入', 'purple')
    b4 = box(xs[3], yt, W, H,
             '④ 逐拍累积 Σ\ndemodKpSum += y·sinφ\ndemodKdSum += y·cosφ\n'
             'accumulate() :373', 'purple')
    # 底行（窗末，右→左）
    b5 = box(xs[3], yb, W, H,
             '⑤ 窗末归一化\n$\\hat{g}$ = 2S/(a·N)\nN = Tw/dt = 3000 拍\n'
             'escStep() :402', 'purple')
    b6 = box(xs[2], yb, W, H,
             '⑥ LPF 平滑梯度\nαL=Tw/(τL+Tw)=60/70\nTw=60 s，τL=10 s\n'
             'escStep() :408', 'purple')
    b7 = box(xs[1], yb, W, H,
             '⑦ 梯度下降 ×η\nK ← K − η·$\\hat{g}$\nη=0.02\n'
             'escStep() :411', 'red')
    b8 = box(xs[0], yb, W, H,
             '⑧ 增益更新\n写回 Kp、Kd\n±10% 限幅\nclampUpdate() :452', 'red')
    # 输入 / 载波 / 输出
    bin_ = box(-2.45, yt + 0.325, 2.1, 1.35,
               '闭环输出（50 Hz）\ne：航向误差\nδ：舵角', 'gray', 10)
    bc = box(7.55, 8.85, 3.85, 1.75,
             '载波生成 escDither() :382-388\nKp×(1+a·sinφ)，Kd×(1+a·cosφ)\n'
             'a=0.05，ω=0.05 rad/s\nφ += ω·dt（每拍推进）', 'orange', 10)
    bout = box(0.35, 0.05, W, 1.35,
               '写回 PD 控制器\n新增益下一窗口生效\n（载波继续注入）', 'gray', 10)

    # 顶行箭头
    arrow(right(bin_), left(b1))
    for ba, bb in ((b1, b2), (b2, b3), (b3, b4)):
        arrow(right(ba), left(bb))
    # 载波两路向下注入 ③
    arrow((8.75, bc[1] - 0.04), (8.75, yt + H + 0.04), color=C['orange'],
          label='sinφ', lx=0.42, ly=0.0, lcol=C['orange'])
    arrow((10.35, bc[1] - 0.04), (10.35, yt + H + 0.04), color=C['orange'],
          label='cosφ', lx=0.48, ly=0.0, lcol=C['orange'])
    # ④ → ⑤ 竖直
    arrow((bottom(b4)[0], bottom(b4)[1] - 0.04),
          (top(b5)[0], top(b5)[1] + 0.04),
          label='窗末（每 60 s）', lx=1.75, lcol=C['purple'])
    # 底行箭头（右→左）
    for ba, bb in ((b5, b6), (b6, b7), (b7, b8)):
        arrow(left(ba), right(bb))
    # ⑧ → 输出
    arrow((bottom(b8)[0], bottom(b8)[1] - 0.04),
          (top(bout)[0], top(bout)[1] + 0.04))

    ax.text(0.35, yt + H + 0.22,
            '每拍链路：dt = 0.02 s（50 Hz），每窗 N = 3000 拍',
            ha='left', fontsize=11, color=C['gray'], style='italic')
    ax.text(8.6, yb - 0.55,
            '窗末结算：每 Tw = 60 s 一次（非重叠窗，结后清零 escStep() :418-425）',
            ha='center', fontsize=11, color=C['gray'], style='italic')
    ax.text(6.3, 11.05,
            'ESC 梯度计算信号流水线（accumulate() 每拍 → escStep() 窗末）',
            ha='center', fontsize=16, fontweight='bold')
    ax.text(6.3, -1.55,
            '行号对应 control/include/auto_tuner.hpp：accumulate() :354-376（每拍）/ escDither() :382-388（载波）'
            '/ escStep() :395-426（窗末）/ clampUpdate() :452（限幅）',
            ha='center', fontsize=9.5, color=C['gray'])
    save(fig, 'esc_grad_pipeline.png')

# ============================================================
# 图2 esc_grad_numeric.png —— 单参数凸代价上的数值演示 + 载波调制波形
# ============================================================
KP_STAR = 0.8
J = lambda kp: 40.0 * (kp - KP_STAR) ** 2     # 合成凸代价（尺度放大仅为可见性）

def esc_1param(Kp0, nwin, seed=5):
    """单参数版估计链路，与 accumulate()/escStep() 逐拍一致（示意用：
    成本增量未乘 dt，常数因子折算进步长，约定同 esc_math_2param.png）。"""
    rng = np.random.default_rng(seed)
    Kp = Kp0
    g_lpf = 0.0
    alphaH = TAU_H / (TAU_H + DT)
    alphaL = WINDOW / (TAU_L + WINDOW)
    win_n = int(WINDOW / DT)
    phase = 0.0
    rec = []
    for _ in range(nwin):
        hpf_y, hpf_c, s_kp = 0.0, None, 0.0
        for _k in range(win_n):
            Kp_t = Kp * (1 + A_REL * np.sin(phase))     # escDither()
            c = J(Kp_t) + 0.02 * rng.standard_normal()  # 合成成本增量
            if hpf_c is None:
                hpf_c = c
            hpf_y = alphaH * (hpf_y + c - hpf_c)        # HPF（accumulate :369）
            hpf_c = c
            s_kp += hpf_y * np.sin(phase)               # 解调累积（:373）
            phase += OMEGA * DT
        g = 2.0 * s_kp / (A_REL * win_n)                # 归一化（:402）
        g_lpf = (1 - alphaL) * g_lpf + alphaL * g       # LPF（:408）
        new = Kp - K_STEP * g_lpf                       # 梯度下降（:411）
        dmax = max(abs(Kp) * MAXFRAC, 1e-3)             # ±10% 限幅
        Kp = max(Kp + np.clip(new - Kp, -dmax, dmax), 0.0)
        rec.append((g, g_lpf, Kp))
    return rec

def fig2():
    Kp0 = 0.5
    rec = esc_1param(Kp0, 2)
    (g1, gl1, Kp1), (g2, gl2, Kp2) = rec
    print(f'  ĝ1={g1:.3f}  LPF后={gl1:.3f}  Kp1={Kp1:.4f}')
    print(f'  ĝ2={g2:.3f}  LPF后={gl2:.3f}  Kp2={Kp2:.4f}')
    x = OMEGA * TAU_H
    reH = x * x / (1 + x * x)   # 同相分量 ReH(jω)，见参考文档 §4.3
    print(f'  ReH(jω)={reH:.4f}，解析 ĝ1≈{reH * Kp0 * 80 * (Kp0 - KP_STAR):.3f}'
          '（整周期值，有限窗有波纹）')

    fig = plt.figure(figsize=(11.5, 9.4))
    gs = GridSpec(2, 2, height_ratios=[1.18, 0.95], hspace=0.34, wspace=0.24,
                  bottom=0.085, top=0.88)
    ax0 = fig.add_subplot(gs[0, :])
    ax1 = fig.add_subplot(gs[1, 0])
    ax2 = fig.add_subplot(gs[1, 1])

    # ---- 上：J(Kp) 抛物线 + 两次迭代 ----
    kp = np.linspace(0.42, 0.95, 400)
    ax0.plot(kp, J(kp), color=C['blue'], lw=2.4)
    ax0.plot([KP_STAR], [0], marker='*', ms=17, color=C['red'], zorder=6)
    ax0.annotate('最优点 Kp*=0.8（J 最小）', (KP_STAR, 0.03), (0.815, 0.85),
                 arrowprops=dict(arrowstyle='->', color=C['red']),
                 color=C['red'], fontsize=10.5, ha='left')
    # 基线点与 ±5% 摄动区间
    ax0.plot([Kp0], [J(Kp0)], 'o', ms=10, color=C['orange'], zorder=6)
    seg_kp = np.linspace(Kp0 * 0.95, Kp0 * 1.05, 60)
    ax0.plot(seg_kp, J(seg_kp), color=C['orange'], lw=5, alpha=0.55,
             solid_capstyle='round', zorder=5)
    ax0.annotate('基线 $Kp_0$=0.500，J=3.60\n摄动区间 ±5%（±a·$Kp_0$=±0.025）',
                 (Kp0 * 0.99, J(Kp0 * 0.99) + 0.06), (0.55, 4.45),
                 color=C['orange'], fontsize=10.5, ha='left',
                 arrowprops=dict(arrowstyle='->', color=C['orange']))
    # 负梯度方向箭头（沿曲线向最优点）
    ax0.annotate('', (Kp0 + 0.011, J(Kp0 + 0.011)), (Kp0, J(Kp0)),
                 arrowprops=dict(arrowstyle='-|>', color=C['red'], lw=2,
                                 linestyle='--'))
    ax0.text(0.535, 3.72, '−$\\hat{g}$ 方向\n（∂J/∂Kp<0，应增大 Kp）',
             color=C['red'], fontsize=10, ha='left')
    # 两次迭代位置
    for kp_i in (Kp1, Kp2):
        ax0.plot([kp_i], [J(kp_i)], 's', ms=9, color=C['green'], zorder=6)
    ax0.annotate('', (Kp1, J(Kp1)), (Kp0, J(Kp0)),
                 arrowprops=dict(arrowstyle='->', color=C['green'], lw=1.6,
                                 connectionstyle='arc3,rad=-0.18'))
    ax0.annotate('', (Kp2, J(Kp2)), (Kp1, J(Kp1)),
                 arrowprops=dict(arrowstyle='->', color=C['green'], lw=1.6,
                                 connectionstyle='arc3,rad=-0.18'))
    ax0.annotate(f'① 第 1 窗后 $Kp_1$={Kp1:.3f}', (Kp1, J(Kp1)),
                 (0.545, 3.12), color=C['green'], fontsize=10.5, ha='left',
                 arrowprops=dict(arrowstyle='->', color=C['green']))
    ax0.annotate(f'② 第 2 窗后 $Kp_2$={Kp2:.3f}', (Kp2, J(Kp2)),
                 (0.555, 2.42), color=C['green'], fontsize=10.5, ha='left',
                 arrowprops=dict(arrowstyle='->', color=C['green']))
    # 数值核对框
    ax0.text(0.985, 0.97,
             '数值核对（链路仿真值）\n'
             '∂J/∂Kp|$Kp_0$ = 80×(0.5−0.8) = −24.0\n'
             f'$\\hat{{g}}_1$ = {g1:.2f} ≈ ReH·$Kp_0$·∂J/∂Kp = −0.71\n'
             '（ReH=(ωτH)²/(1+(ωτH)²)=0.059，余差为有限窗波纹）\n'
             f'LPF 输出 {gl1:.2f}（首窗初值 0，αL=60/70）\n'
             f'$Kp_1$ = 0.500 − 0.02×({gl1:.2f}) = {Kp1:.3f}\n'
             f'$Kp_2$ = {Kp2:.3f}（±10% 限幅未触发）',
             transform=ax0.transAxes, ha='right', va='top', fontsize=10,
             bbox=dict(fc='white', ec=C['gray'], alpha=0.95,
                       boxstyle='round,pad=0.4'))
    ax0.set_xlabel('被调参数 Kp（Kd 固定）')
    ax0.set_ylabel('代价 J(Kp)')
    ax0.set_title('凸代价 J(Kp)=40(Kp−Kp*)² 上的梯度方向与两次迭代', fontweight='bold')
    ax0.set_ylim(-0.4, 6.2)
    ax0.set_xlim(0.42, 0.95)

    # ---- 下：载波调制波形片段（约 1 个载波周期）----
    t = np.linspace(0, 130, 4000)
    s = np.sin(OMEGA * t)
    Kp_t = Kp0 * (1 + A_REL * s)
    J_t = J(Kp_t)
    Tper = 2 * np.pi / OMEGA
    for ax in (ax1, ax2):
        ax.axvspan(0, WINDOW, color=C['yellow'], alpha=0.3)
        ax.axvline(Tper, color=C['gray'], ls=':', lw=1.3)
        ax.set_xlabel('t (s)')
        ax.set_xlim(0, 130)
    ax1.plot(t, Kp_t, color=C['purple'], lw=2)
    ax1.axhline(Kp0, color=C['gray'], ls='--', lw=1.1)
    ax1.axhline(Kp0 * 1.05, color=C['gray'], ls=':', lw=0.9)
    ax1.axhline(Kp0 * 0.95, color=C['gray'], ls=':', lw=0.9)
    ax1.text(WINDOW / 2, Kp0 + 0.0305, '评估窗 Tw=60 s\n（N=3000 拍）',
             ha='center', fontsize=9.5, color='#8a6d00')
    ax1.text(Tper - 1.2, Kp0 - 0.030, '载波周期\n2π/ω≈125.7 s', fontsize=9,
             color=C['gray'], ha='right')
    ax1.set_ylim(0.463, 0.545)
    ax1.set_ylabel('Kp(t)')
    ax1.set_title('① 增益摄动 Kp(t)=$Kp_0$·(1+a·sinωt)，±5%', fontsize=12)
    ax2.plot(t, J_t, color=C['blue'], lw=2)
    ax2.axhline(J(Kp0), color=C['gray'], ls='--', lw=1.1)
    ax2.annotate('', (12, J(Kp0) + 0.6), (12, J(Kp0) - 0.6),
                 arrowprops=dict(arrowstyle='<->', color=C['red'], lw=1.5))
    ax2.text(14.5, 4.36, 'ΔJ(ω) ≈ |∂J/∂Kp|·a·$Kp_0$ ≈ 0.6',
             fontsize=9.5, color=C['red'])
    ax2.text(66, 3.32, 'ω 分量携带梯度信息\n（2ω 分量被同相解调抑制）',
             fontsize=9.5, color=C['gray'])
    ax2.set_ylim(2.7, 4.55)
    ax2.set_ylabel('J(t)')
    ax2.set_title('② 代价被载波调制 J(t)：基线附近随 sinωt 摆动', fontsize=12)

    fig.suptitle('数值演示：单参数凸代价上的 ESC 梯度估计与两次迭代'
                 '（a=0.05，ω=0.05 rad/s，η=0.02，Tw=60 s，τH=5 s，τL=10 s）',
                 fontsize=13.5, fontweight='bold')
    fig.text(0.5, 0.018,
             '注：估计链路与 accumulate()/escStep() 一致（HPF→×载波→Σ→2/(aN)→LPF→×η）；'
             '成本增量未乘 dt（常数因子折算进步长，约定同 esc_math_2param.png），'
             'J 尺度放大 40 倍仅为使迭代位移可见。',
             ha='center', fontsize=9, color=C['gray'])
    save(fig, 'esc_grad_numeric.png')

fig1(); fig2()
print('ALL DONE')
