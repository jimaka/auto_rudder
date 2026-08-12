# -*- coding: utf-8 -*-
# 图8-3：ITTC 波谱（PM/JONSWAP）与遭遇频率映射
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['AR PL UMing CN', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei']
plt.rcParams['axes.unicode_minus'] = False

g = 9.81
U = 12.86          # 25 kn = 12.86 m/s
Hs = 2.5           # 海况4上限
T1 = 7.0           # 平均周期 s

w = np.linspace(0.15, 3.0, 2000)

# ITTC/ISSC 双参数谱（PM）
A = 173 * Hs**2 / T1**4
B = 691 / T1**4
S_pm = A / w**5 * np.exp(-B / w**4)

# JONSWAP：同峰频，gamma=3.3，按 m0 归一化
wp = (0.8 * B) ** 0.25
gamma = 3.3
sigma = np.where(w <= wp, 0.07, 0.09)
peak = gamma ** np.exp(-(w - wp)**2 / (2 * sigma**2 * wp**2))
S_j = S_pm * peak
S_j *= np.trapz(S_pm, w) / np.trapz(S_j, w)

fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))

ax = axes[0]
ax.plot(w, S_pm, 'b-', lw=2, label='Pierson–Moskowitz / ITTC')
ax.plot(w, S_j, 'r--', lw=2, label=r'JONSWAP（$\gamma=3.3$）')
ax.axvline(wp, color='gray', ls=':', lw=1)
ax.text(wp + 0.03, ax.get_ylim()[1] * 0.05, r'$\omega_p$', color='gray')
ax.set_xlabel(r'频率 $\omega$ (rad/s)')
ax.set_ylabel(r'谱密度 $S(\omega)$ (m$^2\cdot$s)')
ax.set_title(r'波谱（$H_{1/3}=2.5$ m，$T_1=7$ s，海况 4 上限）')
ax.legend()
ax.grid(alpha=0.3)
ax.set_xlim(0.15, 2.5)

# 遭遇频率映射 omega_e = |omega - omega^2 U cosχ / g|
ax = axes[1]
for chi, lbl, sty in [(180, r'迎浪 $\chi=180°$', 'b-'),
                      (90, r'横浪 $\chi=90°$', 'g-'),
                      (0, r'随浪 $\chi=0°$', 'r-')]:
    we = w - w**2 * U * np.cos(np.radians(chi)) / g
    ax.plot(w, we, sty, lw=2, label=lbl)
ax.axhline(0, color='k', lw=0.8)
# 操纵频段阴影
ax.axhspan(0.0, 0.2, color='orange', alpha=0.15)
ax.text(2.1, 0.08, '操纵频段', color='darkorange', fontsize=10)
# 折叠点
w_fold = g / (2 * U)
ax.annotate('随浪折叠点\n' + r'$\omega=g/2U\approx0.38$', xy=(w_fold, w_fold / 2),
            xytext=(0.9, -0.5), fontsize=9,
            arrowprops=dict(arrowstyle='->', color='r'))
ax.set_xlabel(r'真实波频 $\omega$ (rad/s)')
ax.set_ylabel(r'遭遇频率 $\omega_e$ (rad/s)')
ax.set_title(r'遭遇频率映射（$U=25$ kn，$\omega_e=|\omega-\frac{\omega^2 U}{g}\cos\chi|$）')
ax.legend(loc='lower right')
ax.grid(alpha=0.3)
ax.set_xlim(0.15, 2.5)
ax.set_ylim(-1.0, 3.0)

fig.tight_layout()
out = '/home/jim/Desktop/auto_rudder/docs/adaptive_autopilot_research/figs/fig_08_wave_spectrum.png'
fig.savefig(out, dpi=150)
print('saved', out)
