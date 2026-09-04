# -*- coding: utf-8 -*-
# ESC 方案"三个滤波器"效果图（esc_filters_effect.png，3 行 × 2 列）
#   ① 一阶高通 HPF（τH=5s，每拍 dt=0.02s）——去直流/慢漂
#   ② 相关解调器（×sin/cos + 窗口平均 Σ，N=3000）——天然带通，梯度提取主滤波器
#   ③ 一阶低通 LPF（τL=10s，时基 Tw=60s，αL=0.857）——平滑窗级梯度估计
# 滤波器划分依据 docs/ESC滤波器设计文档.md §2；递推式与参数全部取自
# control/include/auto_tuner.hpp（accumulate :369-378 / escStep :402-412），
# 风格与 .scratch/esc_math_figs.py、.scratch/esc_grad/esc_grad_figs.py 一致。
import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager as fm
for _p in ('.scratch/fonts/NotoSansCJKsc-Regular.otf',
           '.scratch/fonts/NotoSansCJKsc-Bold.otf'):
    fm.fontManager.addfont(_p)
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

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
    'axes.titlesize': 12.5,
    'axes.labelsize': 11,
})
C = dict(blue='#4C72B0', orange='#DD8452', green='#55A868', red='#C44E52',
         purple='#8172B3', gray='#8C8C8C', teal='#64B5CD', yellow='#CCB974')

# ---- 与 auto_tuner.hpp Params 一致 ----
A_REL   = 0.05    # escAmp（相对摄动幅值）
OMEGA   = 0.05    # escFreq (rad/s)，载波周期 2π/ω≈125.66 s
TAU_H   = 5.0     # escHPFtau
TAU_L   = 10.0    # escLPFtau
WINDOW  = 60.0    # windowSec
DT      = 0.02    # dt
ALPHA_H = TAU_H / (TAU_H + DT)          # 0.99602（accumulate :372）
ALPHA_L = WINDOW / (TAU_L + WINDOW)     # 0.857  （escStep :410）
ALPHA_L_BUG = DT / (TAU_L + DT)         # 0.002  （P11 整改前的错误时基）

KP0     = 0.5
KP_STAR = 0.8
J = lambda kp: 40.0 * (kp - KP_STAR) ** 2   # 合成凸代价（约定同 esc_grad_numeric.png）

def save(fig, name):
    fig.savefig(f'assets/{name}', dpi=200, bbox_inches='tight',
                facecolor='white')
    plt.close(fig)
    print('saved', name)

# ============================================================
# 公共仿真：按 accumulate() 每拍递推生成 c_k / HPF 输出 / 解调乘积
# ============================================================
def simulate_chain(t_end, noise_sigma=0.0, seed=7, drift=True):
    n = int(t_end / DT)
    t = np.arange(n) * DT
    phase = t * OMEGA                       # escPhase_（起点 0）
    kp_t = KP0 * (1.0 + A_REL * np.sin(phase))
    c = J(kp_t)
    if drift:                               # 慢漂：海况/K-T 漂移（远低于载波频率）
        c = c + 0.55 * np.sin(2 * np.pi * t / 500.0) + 0.002 * t
    if noise_sigma > 0:
        rng = np.random.default_rng(seed)
        c = c + noise_sigma * rng.standard_normal(n)
    # 一阶 HPF，后向欧拉（accumulate :372-374）
    y = np.zeros(n)
    y_prev, c_prev = 0.0, c[0]
    for k in range(1, n):
        y[k] = ALPHA_H * (y_prev + c[k] - c_prev)
        y_prev, c_prev = y[k], c[k]
    demod = y * np.sin(phase)               # 因果解调（accumulate :376）
    return t, phase, c, y, demod

# ============================================================
fig = plt.figure(figsize=(13.2, 14.2))
gs = GridSpec(3, 2, hspace=0.42, wspace=0.26,
              left=0.075, right=0.935, top=0.915, bottom=0.078)
ax_hpf_t   = fig.add_subplot(gs[0, 0])
ax_hpf_f   = fig.add_subplot(gs[0, 1])
ax_dem_t   = fig.add_subplot(gs[1, 0])
ax_dem_f   = fig.add_subplot(gs[1, 1])
ax_lpf_t   = fig.add_subplot(gs[2, 0])
ax_lpf_bug = fig.add_subplot(gs[2, 1])

# ------------------------------------------------------------
# ① HPF（a）时域：成本增量 c_k（直流+慢漂+载波纹波）→ 输出 y_k（纯交流）
# ------------------------------------------------------------
T_END = 250.0
t, phase, c, y, demod = simulate_chain(T_END)
ax = ax_hpf_t
ax2 = ax.twinx()
ax2.spines.right.set_visible(True)
l1, = ax.plot(t, c, color=C['blue'], lw=1.0, alpha=0.85, label='输入 $c_k$（左轴）')
ax.axhline(float(np.mean(c)), color=C['blue'], ls=':', lw=1.2)
l2, = ax2.plot(t, y, color=C['orange'], lw=1.4, label='HPF 输出 $y_k$（右轴）')
ax.axvline(WINDOW, color=C['gray'], ls='--', lw=1.1)
ax.text(WINDOW + 2, 3.30, '评估窗 60 s', fontsize=8.5, color=C['gray'],
        bbox=dict(fc='white', ec='none', alpha=0.85, pad=1.0))
ax.set_xlabel('t (s)')
ax.set_ylabel('成本增量 $c_k$', color=C['blue'])
ax2.set_ylabel('HPF 输出 $y_k$', color=C['orange'])
ax.tick_params(axis='y', labelcolor=C['blue'])
ax2.tick_params(axis='y', labelcolor=C['orange'])
ax.set_xlim(0, T_END)
ax.set_title('① HPF 时域效果：直流/慢漂被阻断，只剩载波频率交流分量',
             fontweight='bold', fontsize=11.5)
ax.legend(handles=[l1, l2], loc='upper left', fontsize=9.5)
ax.text(0.985, 0.03,
        'τH=5 s，αH=τH/(τH+dt)=0.996（:372-374）\n'
        '基波幅值衰减 |H|=ωτH/√(1+(ωτH)²)=0.243，相位超前 φH=76°',
        transform=ax.transAxes, ha='right', va='bottom', fontsize=9,
        bbox=dict(fc='white', ec=C['gray'], alpha=0.95, boxstyle='round,pad=0.35'))

# ------------------------------------------------------------
# ① HPF（b）频域：幅频特性 + 载波频率处的同相衰减 ReH
# ------------------------------------------------------------
ax = ax_hpf_f
w = np.logspace(-3, 0.5, 600)               # rad/s
for tau, col, lab, tx, ty in ((TAU_H, C['purple'], 'τH=5 s（现配置）', 0.145, 0.40),
                              (60.0, C['green'], 'τH=60 s（ωτH≳3 准则）', 0.105, 0.76)):
    x = w * tau
    mag = x / np.sqrt(1 + x * x)            # |H(jω)|
    ax.semilogx(w, mag, color=col, lw=2.2, label=lab)
    reh = (OMEGA * tau) ** 2 / (1 + (OMEGA * tau) ** 2)
    ax.plot([OMEGA], [OMEGA * tau / np.sqrt(1 + (OMEGA * tau) ** 2)],
            'o', ms=9, color=col, zorder=5)
    ax.annotate(f'|H|={OMEGA*tau/np.sqrt(1+(OMEGA*tau)**2):.3f}\nReH={reh:.3f}',
                (OMEGA, OMEGA * tau / np.sqrt(1 + (OMEGA * tau) ** 2)),
                (tx, ty),
                fontsize=9.5, color=col, ha='left',
                arrowprops=dict(arrowstyle='->', color=col))
ax.axvline(OMEGA, color=C['red'], ls='--', lw=1.4)
ax.text(OMEGA * 0.85, 1.26, '载波 ω=0.05 rad/s', color=C['red'], fontsize=9.5,
        ha='right', va='top')
ax.set_xlabel('角频率 (rad/s)')
ax.set_ylabel('|H(jω)|（基波幅值留存率）')
ax.set_ylim(0, 1.28)
ax.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
ax.set_title('① HPF 频域效果：载波处留存率决定梯度衰减（ReH=|H|²）',
             fontweight='bold', fontsize=11.5)
ax.legend(loc='lower right', fontsize=9.5)
ax.text(0.03, 0.985,
        'H(s)=τHs/(τHs+1)，φH=atan(1/(ωτH))\n'
        'τH=5 s：φH=76°，ReH=0.059 → 梯度衰减约 17 倍\n'
        '整定准则 ωτH≳3（ReH≳0.9）→ τH≳60 s',
        transform=ax.transAxes, ha='left', va='top', fontsize=9,
        bbox=dict(fc='white', ec=C['gray'], alpha=0.95, boxstyle='round,pad=0.35'))

# ------------------------------------------------------------
# ② 相关解调器（a）时域：y·sinφ 的滑动均值收敛到梯度估计 ĝ
# ------------------------------------------------------------
ax = ax_dem_t
cummean = np.cumsum(demod) / (np.arange(len(demod)) + 1)
ax.plot(t, demod, color=C['teal'], lw=0.5, alpha=0.45, label='$y_k\\cdot\\sin\\varphi$（每拍乘积）')
ax.plot(t, cummean, color=C['red'], lw=2.2, label='累积平均 → 梯度方向')
reh = (OMEGA * TAU_H) ** 2 / (1 + (OMEGA * TAU_H) ** 2)
g_true = reh * KP0 * 80 * (KP0 - KP_STAR) * A_REL / 2.0  # Σ 均值口径（未乘 2/(aN)）
ax.axhline(g_true, color=C['gray'], ls=':', lw=1.3)
for wv in (WINDOW, 2 * WINDOW, 3 * WINDOW, 4 * WINDOW):
    ax.axvline(wv, color=C['gray'], ls='--', lw=1.0)
ax.annotate(f'理论均值 ≈ {g_true:.4f}\n(Kp<Kp*，∂J/∂Kp<0，均值为负 → 应增大 Kp)',
            (T_END * 0.45, g_true), (T_END * 0.42, g_true - 0.0105),
            fontsize=9.5, color=C['gray'],
            arrowprops=dict(arrowstyle='->', color=C['gray']))
ax.text(0.015, 0.96,
        '评估窗 Tw=60 s（竖虚线）\n窗末归一化 $\\hat{g}$=2Σ/(aN)（:402-405）',
        transform=ax.transAxes, ha='left', va='top', fontsize=8.8,
        color=C['gray'],
        bbox=dict(fc='white', ec='none', alpha=0.85, pad=1.2))
ax.set_xlabel('t (s)')
ax.set_ylabel('解调乘积 / 其累积平均')
ax.set_xlim(0, T_END)
ax.set_title('② 相关解调时域效果：乘积均值即梯度（主滤波器，天然带通）',
             fontweight='bold', fontsize=11.5)
ax.legend(loc='lower left', fontsize=9.5)

# ------------------------------------------------------------
# ② 相关解调器（b）频率选择性：各成分对 Σ(y·sinφ) 的贡献（整周期窗）
# ------------------------------------------------------------
ax = ax_dem_f
nper = 4                                    # 整数个载波周期
Tp = 2 * np.pi / OMEGA
n2 = int(nper * Tp / DT)
t2 = np.arange(n2) * DT
rng = np.random.default_rng(3)
comps = {
    '直流残余\n(HPF 泄漏)': 0.05 * np.ones(n2),
    '载波基波 ω\n(梯度信息)': 0.15 * np.sin(OMEGA * t2),
    '二倍频 2ω\n(J″ 谐波)': 0.08 * np.sin(2 * OMEGA * t2 + 0.7),
    '波浪噪声 0.1 Hz\n(遭遇频率)': 0.10 * np.sin(2 * np.pi * 0.1 * t2),
    '宽带白噪声': 0.05 * rng.standard_normal(n2),
}
carrier = np.sin(OMEGA * t2)
contrib = [2.0 * float(np.mean(v * carrier)) / A_REL for v in comps.values()]
cols = [C['gray'], C['red'], C['purple'], C['blue'], C['teal']]
bars = ax.bar(range(len(comps)), contrib, color=cols, alpha=0.85, width=0.62)
for i, v in enumerate(contrib):
    ax.text(i, v + 0.12 if v >= 0 else v - 0.35, f'{v:+.2f}',
            ha='center', fontsize=9.5,
            color=cols[i], fontweight='bold')
ax.set_xticks(range(len(comps)))
ax.set_xticklabels(list(comps.keys()), fontsize=8.8)
ax.axhline(0, color='k', lw=0.8)
ax.set_ylim(-0.8, 3.75)
ax.set_ylabel('对梯度估计的贡献  2·mean(y·sinφ)/a')
ax.set_title('② 相关解调频率选择性：只有"同频同相"分量通过（整周期窗）',
             fontweight='bold', fontsize=11.5)
ax.text(0.985, 0.03,
        '整周期相关：直流/2ω/异频噪声均值严格为 0\n'
        '白噪声被窗口平均压低（σ∝1/√N，N=3000）',
        transform=ax.transAxes, ha='right', va='bottom', fontsize=9,
        bbox=dict(fc='white', ec=C['gray'], alpha=0.95, boxstyle='round,pad=0.35'))

# ------------------------------------------------------------
# ③ LPF（a）窗级：逐窗梯度估计 ĝ_w（涨落）→ LPF 平滑输出
# ------------------------------------------------------------
ax = ax_lpf_t
nwin = 30
rng = np.random.default_rng(11)
alphaL = ALPHA_L
g_lpf = 0.0
kp = KP0
g_raw, g_sm = [], []
for w_i in range(nwin):
    # 解析梯度 + 逐窗涨落（非整数窗残余/海况扰动；幅度夸大以显平滑效果）
    g_exact = reh * kp * 80 * (kp - KP_STAR)
    g = g_exact + 0.16 * rng.standard_normal()
    g_lpf = (1 - alphaL) * g_lpf + alphaL * g      # escStep :411-412
    kp = max(kp - 0.02 * g_lpf, 0.0)               # 梯度下降（:414）
    g_raw.append(g)
    g_sm.append(g_lpf)
widx = np.arange(1, nwin + 1)
ax.plot(widx, g_raw, 'o', ms=5.5, color=C['blue'], alpha=0.65,
        label='窗末归一化 $\\hat{g}_w$（每 60 s 一个）')
ax.plot(widx, g_sm, '-', lw=2.4, color=C['red'],
        label=f'LPF 输出 gradKp（αL={ALPHA_L:.3f}）')
ax.axhline(0, color=C['gray'], ls=':', lw=1.1)
ax.set_xlabel('评估窗序号（1 窗 = 60 s）')
ax.set_ylabel('梯度估计 $\\hat{g}_{Kp}$')
ax.set_title('③ LPF 时域效果：平滑逐窗梯度涨落（弱平滑，防野值冲入积分器）',
             fontweight='bold', fontsize=11.5)
ax.legend(loc='lower right', fontsize=9.5)
ax.text(0.03, 0.97,
        'αL=Tw/(τL+Tw)=60/70=0.857（:410-412）\n'
        '新样本权重 85.7%，等效滞后仅 0.17 窗——定位是"去毛刺"\n'
        '梯度方差主要靠 3000 点窗口平均压低',
        transform=ax.transAxes, ha='left', va='top', fontsize=9,
        bbox=dict(fc='white', ec=C['gray'], alpha=0.95, boxstyle='round,pad=0.35'))

# ------------------------------------------------------------
# ③ LPF（b）阶跃响应：正确时基(Tw) vs P11 错误时基(dt)
# ------------------------------------------------------------
ax = ax_lpf_bug
steps = np.arange(0, 31)
for a_l, col, lab in ((ALPHA_L, C['green'],
                       f'正确：αL=Tw/(τL+Tw)={ALPHA_L:.3f}（时基 60 s）'),
                      (ALPHA_L_BUG, C['red'],
                       f'P11 错误：αL=dt/(τL+dt)={ALPHA_L_BUG:.3f}（时基 0.02 s）')):
    g = 1.0 - (1.0 - a_l) ** steps          # 对单位阶跃的响应
    ax.plot(steps, g, '-o', ms=4, lw=2.2, color=col, label=lab)
ax.axhline(1.0, color=C['gray'], ls=':', lw=1.1)
ax.text(29.8, 1.04, '真实梯度', fontsize=9, color=C['gray'], va='bottom',
        ha='right')
ax.set_xlim(-0.6, 31)
ax.annotate('30 窗（30 min）后仅跟上 5.8%\n→ 新梯度几乎进不了估计器，\n调参结果与初值逐位相同',
            (30, 1.0 - (1.0 - ALPHA_L_BUG) ** 30), (16.5, 0.42),
            fontsize=9.5, color=C['red'],
            arrowprops=dict(arrowstyle='->', color=C['red']))
ax.set_xlabel('评估窗序号')
ax.set_ylabel('LPF 输出 / 输入梯度（单位阶跃响应）')
ax.set_ylim(-0.05, 1.12)
ax.set_title('③ LPF 系数时基对比：P11 整改——430 倍系数差距的后果',
             fontweight='bold', fontsize=11.5)
ax.legend(loc='upper left', fontsize=9.5)

fig.suptitle('ESC 方案三个滤波器的效果图\n'
             '① 一阶高通 HPF（每拍，去直流/慢漂） →  ② 相关解调器（每拍累积，带通提取梯度）'
             ' →  ③ 一阶低通 LPF（窗末，平滑梯度）',
             fontsize=14.5, fontweight='bold')
fig.text(0.5, 0.014,
         '递推式与参数与 control/include/auto_tuner.hpp 一致：HPF $y_k$=αH($y_{k-1}$+$c_k$−$c_{k-1}$)，'
         'αH=τH/(τH+dt)（:372-374）；解调 Σ(y·sinφ)，$\\hat{g}$=2Σ/(aN)（:376-377, :402-405）；'
         'LPF αL=Tw/(τL+Tw)（:410-412）。\n'
         '合成凸代价 $J(K_p)=40(K_p-0.8)^2$，$K_{p0}$=0.5；a=0.05，ω=0.05 rad/s，dt=0.02 s，Tw=60 s；'
         '滤波器划分见 docs/ESC滤波器设计文档.md §2。',
         ha='center', fontsize=8.8, color=C['gray'])
save(fig, 'esc_filters_effect.png')
print('ALL DONE')
