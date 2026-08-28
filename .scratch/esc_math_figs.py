# -*- coding: utf-8 -*-
# ESC 原理示意图生成脚本（esc_math_ 前缀，5 张）
# 参数全部取自 control/include/auto_tuner.hpp（a=0.05, ω=0.05rad/s, k=0.02,
# τH=5s, τL=10s, window=60s, dt=0.02s, ±10% 限幅, ζ≥0.6, ωn≤0.5, osc≤5°）
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
# 图1 esc_math_gradient.png —— J(θ) 一维二次曲线 + 两侧摄动-响应相关性
# ============================================================
def fig1():
    th_star = 1.0
    J = lambda th: (th - th_star) ** 2
    th_L, th_R, A = 0.45, 1.55, 0.12

    fig = plt.figure(figsize=(11, 8.6))
    gs = GridSpec(2, 2, height_ratios=[1.12, 1.0], hspace=0.32, wspace=0.24)
    ax0 = fig.add_subplot(gs[0, :])
    ax1 = fig.add_subplot(gs[1, 0])
    ax2 = fig.add_subplot(gs[1, 1], sharey=ax1)

    # 上：J(θ) 曲线
    th = np.linspace(0.0, 2.0, 400)
    ax0.plot(th, J(th), color=C['blue'], lw=2.4)
    ax0.axvline(th_star, color=C['gray'], ls=':', lw=1.4)
    ax0.plot([th_star], [0], marker='*', ms=16, color=C['red'], zorder=5)
    ax0.annotate('最优点 θ*（J 最小）', (th_star, 0.02), (th_star + 0.10, 0.24),
                 arrowprops=dict(arrowstyle='->', color=C['red']), color=C['red'])
    for th0, name, dcol, tx in ((th_L, 'θ_L', C['orange'], 0.58),
                                (th_R, 'θ_R', C['green'], 1.42)):
        grad_sign = '∂J/∂θ < 0 → 应增大 θ' if th0 < th_star else '∂J/∂θ > 0 → 应减小 θ'
        ax0.plot([th0], [J(th0)], 'o', ms=10, color=dcol, zorder=5)
        # 摄动区间
        ax0.plot([th0 - A, th0 + A], [J(th0 - A), J(th0 + A)], color=dcol,
                 lw=4, alpha=0.55, solid_capstyle='round')
        ax0.annotate(f'{name}：叠加摄动 a·sinωt\n{grad_sign}', (th0, J(th0)),
                     (tx, 0.66), color=dcol, ha='center', fontsize=10,
                     arrowprops=dict(arrowstyle='->', color=dcol))
        # 负梯度方向箭头（沿曲线切线）
        g = 2 * (th0 - th_star)
        dx = -0.22 * np.sign(g)
        ax0.annotate('', (th0 + dx, J(th0 + dx)), (th0, J(th0)),
                     arrowprops=dict(arrowstyle='-|>', color=dcol, lw=2,
                                     linestyle='--'))
    ax0.set_xlabel('被调参数 θ（如 Kp / Kd）')
    ax0.set_ylabel('代价 J(θ)')
    ax0.set_title('一维代价曲面 J(θ) = (θ − θ*)² 与两点处的正弦摄动', fontweight='bold')

    # 下：两侧摄动-响应
    t = np.linspace(0, 4 * np.pi, 800)
    s = np.sin(t)
    for ax, th0, name, dcol, side in ((ax1, th_L, 'θ_L', C['orange'], '左侧'),
                                      (ax2, th_R, 'θ_R', C['green'], '右侧')):
        dJ = J(th0 + A * s) - J(th0)          # J 响应（去直流后同形）
        prod = s * dJ                          # 解调乘积
        mprod = prod.mean()
        ax.plot(t, s, color=C['gray'], ls='--', lw=1.3, label='载波 sinωt')
        ax.plot(t, dJ, color=dcol, lw=2.0, label='J 摄动响应 y(t)')
        ax.plot(t, prod, color=C['red'], lw=1.0, alpha=0.75,
                label='解调乘积 y·sinωt')
        ax.axhline(mprod, color=C['red'], lw=2.0, ls='-')
        sign = '< 0' if mprod < 0 else '> 0'
        direction = '∂J/∂θ < 0 → 应增大 θ' if mprod < 0 else '∂J/∂θ > 0 → 应减小 θ'
        rel = '反相关（响应与载波反相）' if mprod < 0 else '正相关（响应与载波同相）'
        ax.set_title(f'{name}（最优点{side}）：{rel}', fontsize=12)
        ax.text(0.03, 0.05, '$\\langle y\\cdot\\sin\\omega t\\rangle$'
                f' = (a/2)·∂J/∂θ {sign}\n{direction}',
                transform=ax.transAxes, fontsize=10.5, color=C['red'],
                bbox=dict(fc='white', ec=C['red'], alpha=0.9, boxstyle='round,pad=0.35'))
        ax.set_xlabel('时间（2 个载波周期）')
        ax.set_xticks([0, np.pi, 2*np.pi, 3*np.pi, 4*np.pi])
        ax.set_xticklabels(['0', 'π', '2π', '3π', '4π'])
    ax1.set_ylabel('幅值（示意）')
    ax1.legend(loc='upper right', fontsize=9, ncol=1)
    fig.suptitle('摄动法梯度估计原理：J 响应与载波的相关性给出梯度符号与大小',
                 fontsize=15, fontweight='bold', y=0.985)
    save(fig, 'esc_math_gradient.png')

# ============================================================
# 图2 esc_math_demod.png —— 信号链四阶段时域图，验证 ĝ∝∂J/∂θ
# ============================================================
def fig2():
    rng = np.random.default_rng(7)
    d = 0.8            # θ̄ − θ*（偏离最优）
    T = 400.0
    n = int(T / DT)
    t = np.arange(n) * DT
    s = np.sin(OMEGA * t)
    J = (d + A_REL * s) ** 2 + 0.004 * rng.standard_normal(n)   # 合成代价

    # 一阶 HPF（后向欧拉，同 accumulate()）：y = α(y + c − c_prev)
    alphaH = TAU_H / (TAU_H + DT)
    y = np.zeros(n)
    for k in range(1, n):
        y[k] = alphaH * (y[k - 1] + J[k] - J[k - 1])
    z = y * s                                       # × 同相载波
    run = 2.0 * np.cumsum(z) / (A_REL * np.arange(1, n + 1))   # 滑动平均
    # 窗末（每 60 s）块估计，对应 escStep() 的 ĝ=2Σ/(aN)
    win_n = int(WINDOW / DT)
    wins, gwin = [], []
    for w0 in range(0, n - win_n + 1, win_n):
        blk = z[w0:w0 + win_n]
        wins.append((w0 + win_n) * DT)
        gwin.append(2.0 * blk.sum() / (A_REL * win_n))

    g_true = 2 * d     # ∂J/∂θ = 2(θ̄−θ*)
    # HPF 在载波频率处的稳态幅值/相角：有效增益 |H|·cos∠H = ωτH/(1+(ωτH)²)
    x = OMEGA * TAU_H
    att = x / (1 + x * x)

    fig, axes = plt.subplots(4, 1, figsize=(11, 9.2), sharex=True)
    fig.subplots_adjust(hspace=0.42)
    axes[0].plot(t, J, color=C['blue'], lw=0.8)
    axes[0].set_title('① 原始代价 J(θ(t))：直流 + ω 分量（梯度信息）+ 2ω 分量 + 噪声')
    axes[0].set_ylabel('J(t)')
    axes[1].plot(t, y, color=C['purple'], lw=0.9)
    axes[1].set_title('② HPF（τH=5 s）去直流后 y(t)：保留与载波同频的梯度分量'
                      '（ωτH=0.25，幅值被衰减）')
    axes[1].set_ylabel('y(t)')
    axes[2].plot(t, z, color=C['orange'], lw=0.7)
    axes[2].axhline(z[50:].mean(), color=C['red'], lw=1.6)
    axes[2].set_title('③ ×同相载波 sinωt 后 z(t)：均值恒正（$\\bar{\\theta}$ 在'
                      '最优点右侧，∂J/∂θ>0）')
    axes[2].set_ylabel('z(t)')
    axes[3].plot(t, run, color=C['green'], lw=1.8,
                 label='滑动平均 $\\hat{g}$(t)=2Σz/(aN)')
    axes[3].plot(wins, gwin, 's', color=C['red'], ms=7, zorder=5,
                 label='窗末块估计（每 60 s，对应 escStep()）')
    axes[3].axhline(g_true, color=C['gray'], ls='--', lw=1.6)
    axes[3].text(205, g_true + 0.08,
                 f'真实梯度 ∂J/∂θ = 2($\\bar{{\\theta}}$−θ*) = {g_true:.1f}',
                 color=C['gray'], fontsize=10.5)
    axes[3].axhline(att * g_true, color=C['purple'], ls=':', lw=1.8)
    axes[3].text(205, att * g_true + 0.09,
                 f'HPF 稳态有效增益 ωτH/(1+(ωτH)²)≈{att:.2f}\n'
                 f'→ $\\hat{{g}}$≈{att:.2f}×∂J/∂θ≈{att*g_true:.2f}'
                 '（符号与相对大小不变）',
                 color=C['purple'], fontsize=10, ha='center')
    axes[3].set_ylim(-0.5, 2.0)
    axes[3].set_title('④ 归一化滑动平均收敛到梯度估计值（幅值含 HPF 衰减因子，方向正确）')
    axes[3].set_ylabel('$\\hat{g}$')
    axes[3].set_xlabel('时间 t (s)')
    axes[3].legend(loc='upper left', fontsize=10)
    fig.suptitle('ESC 解调链路验证（合成 J(θ)=(θ−θ*)²，$\\bar{\\theta}$−θ*=0.8，'
                 'a=0.05，ω=0.05 rad/s，dt=0.02 s）', fontsize=14.5, fontweight='bold')
    save(fig, 'esc_math_demod.png')

# ============================================================
# 图3 esc_math_2param.png —— J(Kp,Kd) 等高线 + 双参数 ESC 搜索轨迹
# ============================================================
def J2(Kp, Kd):
    x, y = (Kp - 0.8) / 0.25, (Kd - 0.30) / 0.12
    return x ** 2 + y ** 2 + 0.5 * x * y

def esc_sim(Kp0, Kd0, nwin, seed=3):
    """按 auto_tuner.hpp 的算法逐拍仿真（合成代价 J2，无船舶模型）。
    与源码两点工程性差异（示意用）：① 每窗 HPF 初值取窗首样本，去除启动瞬态；
    ② 成本增量未乘 dt（常数折算进积分步长）。载波相位跨窗连续（同源码）。"""
    rng = np.random.default_rng(seed)
    Kp, Kd = Kp0, Kd0
    gKp_lpf = gKd_lpf = 0.0
    alphaH = TAU_H / (TAU_H + DT)
    alphaL = WINDOW / (TAU_L + WINDOW)
    win_n = int(WINDOW / DT)
    traj = [(Kp, Kd)]
    phase = 0.0
    for _ in range(nwin):
        hpf_y = 0.0
        hpf_c = None
        sKp = sKd = 0.0
        for _k in range(win_n):
            Kp_t = Kp * (1 + A_REL * np.sin(phase))   # escDither()
            Kd_t = Kd * (1 + A_REL * np.cos(phase))
            c = J2(Kp_t, Kd_t) + 0.005 * rng.standard_normal()  # 合成成本增量
            if hpf_c is None:
                hpf_c = c
            hpf_y = alphaH * (hpf_y + c - hpf_c)      # HPF（同 accumulate()）
            hpf_c = c
            sKp += hpf_y * np.sin(phase)              # 相关解调
            sKd += hpf_y * np.cos(phase)
            phase += OMEGA * DT
        gKp = 2.0 * sKp / (A_REL * win_n)             # 窗末归一化 ĝ=2S/(aN)
        gKd = 2.0 * sKd / (A_REL * win_n)
        gKp_lpf = (1 - alphaL) * gKp_lpf + alphaL * gKp   # LPF
        gKd_lpf = (1 - alphaL) * gKd_lpf + alphaL * gKd
        newKp = Kp - K_STEP * gKp_lpf                 # 积分更新
        newKd = Kd - K_STEP * gKd_lpf
        dmax = max(abs(Kp) * MAXFRAC, 1e-3)           # ±10% 限幅（clampUpdate）
        newKp = Kp + np.clip(newKp - Kp, -dmax, dmax)
        dmax = max(abs(Kd) * MAXFRAC, 1e-3)
        newKd = Kd + np.clip(newKd - Kd, -dmax, dmax)
        Kp, Kd = max(newKp, 0.0), max(newKd, 0.0)
        traj.append((Kp, Kd))
    return np.array(traj)

def fig3():
    Kp0, Kd0, NW = 0.42, 0.08, 80
    traj = esc_sim(Kp0, Kd0, NW)
    kp = np.linspace(0.25, 1.15, 220)
    kd = np.linspace(0.0, 0.52, 220)
    KKp, KKd = np.meshgrid(kp, kd)
    JJ = J2(KKp, KKd)

    fig, ax = plt.subplots(figsize=(10.5, 8.2))
    cf = ax.contourf(KKp, KKd, JJ, levels=24, cmap='viridis', alpha=0.35)
    cl = ax.contour(KKp, KKd, JJ, levels=14, cmap='viridis', linewidths=0.8)
    ax.clabel(cl, fmt='%.2f', fontsize=8)
    ax.plot(0.8, 0.30, '*', ms=20, color=C['red'], zorder=6, label='最优增益 (Kp*, Kd*)')
    sc = ax.scatter(traj[:, 0], traj[:, 1], c=np.arange(len(traj)), cmap='plasma',
                    s=42, zorder=5, edgecolor='white', linewidth=0.5)
    ax.plot(traj[:, 0], traj[:, 1], color=C['gray'], lw=1.0, alpha=0.7, zorder=4)
    ax.plot(Kp0, Kd0, 'o', ms=11, color=C['orange'], mec='black', zorder=6,
            label=f'初值 ({Kp0}, {Kd0})')
    ax.annotate('', traj[2], traj[0],
                arrowprops=dict(arrowstyle='->', color=C['orange'], lw=2))
    cb = fig.colorbar(sc, ax=ax, pad=0.02)
    cb.set_label('ESC 窗口序号（每窗 60 s）')
    ax.set_xlabel('Kp（sin 载波解调）')
    ax.set_ylabel('Kd（cos 载波解调）')
    ax.set_title('双参数 ESC 搜索轨迹：正交 sin/cos 摄动，从偏离初值收敛到谷底附近\n'
                 '（合成 J(Kp,Kd) 二次 Bowl，算法与 auto_tuner.hpp 一致：'
                 'a=0.05，ω=0.05 rad/s，k=0.02，±10% 限幅）',
                 fontsize=13, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)
    save(fig, 'esc_math_2param.png')
    print('  轨迹末点:', traj[-1], ' J末值: %.4f' % J2(*traj[-1]))

# ============================================================
# 图4 esc_math_timescale.png —— 时间尺度三分离
# ============================================================
def fig4():
    fig, axes = plt.subplots(3, 1, figsize=(11, 8.8))
    fig.subplots_adjust(hspace=0.52)

    # ① 闭环动态（秒级）：二阶航向响应 ζ=0.85, ωn=0.2
    zeta, wn = 0.85, 0.20
    wd = wn * np.sqrt(1 - zeta ** 2)
    t1 = np.linspace(0, 45, 2000)
    e = np.exp(-zeta * wn * t1) * (np.cos(wd * t1)
        + zeta * wn / wd * np.sin(wd * t1))
    axes[0].plot(t1, e, color=C['blue'], lw=2)
    axes[0].axhline(0, color=C['gray'], lw=0.8)
    ts = 4 / (zeta * wn)
    axes[0].axvline(ts, color=C['red'], ls=':', lw=1.5)
    axes[0].text(ts + 0.6, 0.55, f'调节时间 ≈ 4/(ζωn) ≈ {ts:.0f} s',
                 color=C['red'], fontsize=10.5)
    axes[0].set_title('① 闭环动态（秒级）：航向误差响应，ζ=0.85、ωn=0.2 rad/s', fontsize=12.5)
    axes[0].set_xlabel('t (s)'); axes[0].set_ylabel('e(t)（归一化）')
    axes[0].set_xlim(0, 45)

    # ② 摄动载波（百秒级）
    t2 = np.linspace(0, 300, 6000)
    axes[1].plot(t2, np.sin(OMEGA * t2), color=C['purple'], lw=1.8)
    Tper = 2 * np.pi / OMEGA
    axes[1].axvspan(0, WINDOW, color=C['yellow'], alpha=0.35)
    axes[1].text(WINDOW / 2, -1.45, '评估窗 60 s', ha='center', color='#8a6d00',
                 fontsize=10.5)
    axes[1].annotate('', (Tper, 1.18), (0, 1.18),
                     arrowprops=dict(arrowstyle='<->', color=C['gray']))
    axes[1].text(Tper / 2, 1.28, f'载波周期 2π/ω ≈ {Tper:.0f} s', ha='center',
                 color=C['gray'], fontsize=10.5)
    axes[1].set_title('② 摄动载波（百秒级）：sinωt，ω=0.05 rad/s —— 慢于闭环，'
                      '保证代价对增益的准稳态依赖', fontsize=12.5)
    axes[1].set_xlabel('t (s)'); axes[1].set_ylabel('sinωt')
    axes[1].set_ylim(-1.8, 1.7); axes[1].set_xlim(0, 300)

    # ③ 积分器适应（多窗口，数十分钟）
    traj = esc_sim(0.42, 0.08, 80)
    tw = np.arange(len(traj)) * WINDOW / 60.0
    axes[2].step(tw, traj[:, 0], where='post', color=C['blue'], lw=2, label='Kp')
    axes[2].step(tw, traj[:, 1], where='post', color=C['green'], lw=2, label='Kd')
    axes[2].axhline(0.8, color=C['blue'], ls=':', lw=1.2)
    axes[2].axhline(0.30, color=C['green'], ls=':', lw=1.2)
    axes[2].set_title('③ 积分器适应（多窗口）：每 60 s 窗末更新一次，LPF 平滑 + ±10% 限幅，'
                      '数十分钟收敛', fontsize=12.5)
    axes[2].set_xlabel('时间 (min)'); axes[2].set_ylabel('增益')
    axes[2].legend(loc='center right', fontsize=10)
    fig.suptitle('ESC 时间尺度三分离：闭环动态（秒）≪ 载波周期（百秒）≪ 适应收敛（数十分钟）',
                 fontsize=14.5, fontweight='bold')
    save(fig, 'esc_math_timescale.png')

# ============================================================
# 图5 esc_math_chain.png —— ESC 信号链框图（完整回路）
# ============================================================
def fig5():
    fig, ax = plt.subplots(figsize=(15.5, 9.4))
    ax.set_xlim(-0.45, 15.4); ax.set_ylim(-2.0, 9.3)
    ax.axis('off'); ax.grid(False)

    FC = dict(blue='#eaf1fb', orange='#fdf3e7', green='#eaf6ee',
              purple='#f1edf9', red='#fbeaea', gray='#f2f2f2')

    def box(x, y, w, h, text, color, fs=10.5):
        p = FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.09',
                           fc=FC[color], ec=C[color], lw=1.8, zorder=3)
        ax.add_patch(p)
        ax.text(x + w / 2, y + h / 2, text, ha='center', va='center',
                fontsize=fs, zorder=4, linespacing=1.5)
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

    def seg(points, color, ls='-', lw=1.8):
        xs = [p[0] for p in points]; ys = [p[1] for p in points]
        ax.plot(xs, ys, color=color, ls=ls, lw=lw, zorder=2,
                solid_capstyle='round')

    W, H, yt, yb = 2.35, 1.62, 6.6, 2.4
    xs = [0.35, 3.15, 5.95, 8.75, 11.55]
    # 顶行（左→右）
    b1 = box(xs[0], yt, W, H, '① 扰动注入（每拍）\nKp·(1+a·sinωt)\nKd·(1+a·cosωt)', 'orange')
    b2 = box(xs[1], yt, W, H, 'PD 控制器\n增益 Kp(t), Kd(t)', 'blue')
    b3 = box(xs[2], yt, W, H, '闭环系统\nPD + 船舶\n输出 e, δ', 'blue')
    b4 = box(xs[3], yt, W, H, '代价累积\n$c_k$=(t|e|+wU|δ|\n+wDu|dδ|)·dt', 'green', 10)
    b5 = box(xs[4], yt, W, H, 'HPF 去直流\nτH=5 s\ny=α(y+$c_k$−$c_{k-1}$)', 'purple', 10)
    # 底行（右→左）
    b6 = box(xs[4], yb, W, H, '相关解调（每拍）\n$S_{Kp}$ += y·sinωt\n$S_{Kd}$ += y·cosωt', 'purple', 10)
    b7 = box(xs[3], yb, W, H, '窗末归一化\n$\\hat{g}$ = 2S/(a·N)', 'purple')
    b8 = box(xs[2], yb, W, H, 'LPF 平滑\nα=60/(τL+60)\nτL=10 s', 'purple', 10)
    b9 = box(xs[1], yb, W, H, '积分更新\nK ← K − k·$\\hat{g}$\nk=0.02', 'red')
    b10 = box(xs[0], yb, W, H, '±10% 限幅\n|ΔK|≤max(0.1|K|, $10^{-3}$)', 'red', 10)
    b11 = box(0.35, -0.75, W, 1.3, '稳定门（四条件）\nKp>0；ζ≥0.6\nωn≤0.5；振荡≤5°', 'red', 10)
    b12 = box(3.6, -0.75, W, 1.3, '回退基线\nKp ← Kp*，Kd ← Kd*', 'gray', 10.5)

    def right(b): return (b[0] + b[2], b[1] + b[3] / 2)
    def left(b):  return (b[0], b[1] + b[3] / 2)
    def top(b):   return (b[0] + b[2] / 2, b[1] + b[3])
    def bottom(b):return (b[0] + b[2] / 2, b[1])

    # 顶行箭头
    for ba, bb in ((b1, b2), (b2, b3), (b3, b4), (b4, b5)):
        arrow(right(ba), left(bb))
    ax.text(7.5, yt + H + 0.30, '每控制拍执行：dt = 0.02 s（50 Hz）',
            ha='center', fontsize=11, color=C['gray'], style='italic')
    # b5 → b6 竖直
    arrow(bottom(b5), top(b6), label='每拍累积', lx=1.0, lcol=C['purple'])
    # 底行箭头（右→左）
    for ba, bb in ((b6, b7), (b7, b8), (b8, b9), (b9, b10)):
        arrow(left(ba), right(bb))
    ax.text(7.5, yb - 0.45, '窗末结算：每 60 s 一次（非重叠窗，结后清零）',
            ha='center', fontsize=11, color=C['gray'], style='italic')
    # b10 → b11
    arrow(bottom(b10), top(b11))
    # 通过：持久化 → 左侧竖直通道回到 ①
    seg([left(b11), (-0.15, left(b11)[1]), (-0.15, left(b1)[1])],
        color=C['green'], lw=2.2)
    arrow((-0.15, left(b1)[1]), (left(b1)[0] + 0.03, left(b1)[1]),
          color=C['green'], lw=2.2)
    ax.text(0.07, 3.4, '通过：持久化新增益', rotation=90, color=C['green'],
            fontsize=10, ha='center', va='center')
    # 否则：回退基线 → 底部通道回到 ①
    arrow(right(b11), left(b12), color=C['red'], ls='--', label='否则',
          ly=0.32, lcol=C['red'])
    seg([(bottom(b12)[0], bottom(b12)[1] - 0.05),
         (bottom(b12)[0], -1.55), (-0.32, -1.55), (-0.32, 5.85),
         (bottom(b1)[0], 5.85)], color=C['red'], ls='--', lw=1.8)
    arrow((bottom(b1)[0], 5.85), (bottom(b1)[0], bottom(b1)[1] - 0.03),
          color=C['red'], ls='--', lw=1.8)
    ax.text(3.3, -1.72, '否则：回退基线增益，重新注入', color=C['red'],
            fontsize=10, ha='center')

    ax.text(7.5, 9.02, 'ESC 信号链完整回路（对应 control/include/auto_tuner.hpp）',
            ha='center', fontsize=16, fontweight='bold')
    ax.text(7.5, -1.92,
            '代码位置：escDither() L382（扰动注入）/ accumulate() L354（代价+HPF+解调）/ escStep() L395（归一化+LPF+积分）'
            '/ clampUpdate() L452（限幅）/ stabilityCheck() L241（稳定门）/ revert() L429（回退）',
            ha='center', fontsize=9.5, color=C['gray'])
    save(fig, 'esc_math_chain.png')

fig1(); fig2(); fig3(); fig4(); fig5()
print('ALL DONE')
