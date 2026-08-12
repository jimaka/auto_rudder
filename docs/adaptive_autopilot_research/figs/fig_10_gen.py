# -*- coding: utf-8 -*-
"""生成 10 章节示意图：RRS 频带分离幅频特性图 + 有无 RRS 横摇角示意仿真对比"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['AR PL UMing CN', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei']
plt.rcParams['axes.unicode_minus'] = False

OUT = '/home/jim/Desktop/auto_rudder/docs/adaptive_autopilot_research/figs/'

# ---------------- 图 10-1：舵→横摇 / 舵→艏摇幅频特性与频带分离 ----------------
w = np.logspace(np.log10(0.03), np.log10(20), 2000)

wn_roll = 0.6      # 横摇固有频率 rad/s（小型高速船典型量级）
z_roll = 0.08      # 横摇阻尼比
# 舵→横摇：带通特性，在横摇固有频率处共振峰
G_roll = (w**2) / np.sqrt((wn_roll**2 - w**2)**2 + (2*z_roll*wn_roll*w)**2)
G_roll /= G_roll.max()

# 舵→艏向：Nomoto 一阶+积分，低频特性
T_nom = 3.0
G_yaw = 1.0 / (w * np.sqrt(1 + (w*T_nom)**2))
G_yaw /= G_yaw[np.argmin(np.abs(w - 0.05))]

fig, ax = plt.subplots(figsize=(10.5, 5.8))
ax.axvspan(0.03, 0.15, color='#2B6CB0', alpha=0.10)
ax.axvspan(0.3, 1.5, color='#C05621', alpha=0.12)
ax.axvspan(3.8, 20, color='#2F855A', alpha=0.10)

ax.semilogx(w, 20*np.log10(G_yaw + 1e-12), color='#2B6CB0', lw=2.2, label=r'舵→艏向通道 $|\psi/\delta|$（低频）')
ax.semilogx(w, 20*np.log10(G_roll + 1e-12), color='#C05621', lw=2.2, label=r'舵→横摇通道 $|\varphi/\delta|$（带通，共振于 $\omega_\varphi$）')

ax.axvline(wn_roll, color='#C05621', ls='--', lw=1.2)
ax.text(wn_roll*1.06, -38, r'横摇固有频率 $\omega_\varphi$', color='#C05621', fontsize=11)
ax.axvline(6.4*wn_roll, color='#2F855A', ls='--', lw=1.2)
ax.text(6.4*wn_roll*1.06, -30, r'舵机带宽下限 $\approx 6.4\,\omega_\varphi$', color='#2F855A', fontsize=11)

ax.text(0.058, 26, '航向控制通道\n(低频，带宽 $\\ll \\omega_\\varphi$)', fontsize=10.5, color='#2B6CB0', ha='center')
ax.text(0.68, 24, '波频遭遇频率区间 + 减摇通道\n(横摇固有频率附近)', fontsize=10.5, color='#9C4221', ha='center')
ax.text(7.6, 18, '舵机伺服频带\n(必须覆盖)', fontsize=10.5, color='#276749', ha='center')

ax.annotate('频带分离条件：\n$\\omega_{yaw} \\ll \\omega_\\varphi < \\omega_e <$ 舵机带宽',
            xy=(0.16, -8), xytext=(0.16, -34), fontsize=10.5,
            bbox=dict(boxstyle='round,pad=0.4', fc='#FFFBEA', ec='#B7791F'))

ax.set_xlabel(r'频率 $\omega$ (rad/s)')
ax.set_ylabel('幅值 (dB，归一化)')
ax.set_xlim(0.03, 20); ax.set_ylim(-55, 35)
ax.set_xticks([0.05, 0.1, 0.3, 0.6, 1, 2, 4, 10, 20])
ax.set_xticklabels(['0.05', '0.1', '0.3', '0.6', '1', '2', '4', '10', '20'])
ax.minorticks_off()
ax.grid(True, which='both', ls=':', alpha=0.5)
ax.legend(loc='lower left', fontsize=10.5)
ax.set_title('图 10-1  RRS 频带分离原理：舵→艏向 / 舵→横摇幅频特性示意（高速游艇量级参数）')
fig.tight_layout()
fig.savefig(OUT + 'fig_10_frequency_separation.png', dpi=150)
plt.close(fig)

# ---------------- 图 10-2：有无 RRS 的横摇角示意仿真 ----------------
# 简化二阶横摇模型：φ'' + 2ζωφ φ' + ωφ² φ = Kw·w(t) + Kδ·δ
# RRS 通道：δ = -(k1·φ + k2·φ̇)，含舵角限幅 ±12°、舵速限制 10°/s
dt = 0.01
t = np.arange(0, 120, dt)
zeta, wphi, Kdelta = 0.07, 0.6, 0.15
Kw = 0.03

rng = np.random.default_rng(7)
# 波浪激励：遭遇频率附近的窄带谱（若干正弦叠加 + 慢变幅值）
freqs = np.linspace(0.42, 0.85, 12)
amps = np.sqrt(2.0 / 12.0) * np.ones(12)
phs = rng.uniform(0, 2*np.pi, 12)
env = 0.7 + 0.5*np.sin(2*np.pi*t/47.0) * np.sin(2*np.pi*t/23.0 + 1.0)
w_exc = np.zeros_like(t)
for f, a, p in zip(freqs, amps, phs):
    w_exc += a * np.sin(2*np.pi*(f/ (2*np.pi))*t + p)  # freqs 已是 rad/s
w_exc = Kw * env * w_exc

def simulate(rrs):
    phi = np.zeros_like(t); phid = np.zeros_like(t)
    delta = np.zeros_like(t); d_prev = 0.0
    for i in range(1, len(t)):
        acc = w_exc[i] + Kdelta*d_prev - 2*zeta*wphi*phid[i-1] - wphi**2*phi[i-1]
        phid[i] = phid[i-1] + acc*dt
        phi[i] = phi[i-1] + phid[i]*dt
        if rrs:
            d_cmd = -(1.0*phi[i] + 1.6*phid[i])
        else:
            d_cmd = 0.0
        d_cmd = np.clip(d_cmd, np.deg2rad(-12), np.deg2rad(12))
        max_step = np.deg2rad(10)*dt   # 舵速限制 10°/s
        d_prev += np.clip(d_cmd - d_prev, -max_step, max_step)
        delta[i] = d_prev
    return np.rad2deg(phi), np.rad2deg(delta)

phi_off, d_off = simulate(False)
phi_on, d_on = simulate(True)
seg = t >= 20
red = 100*(1 - np.sqrt(np.mean(phi_on[seg]**2))/np.sqrt(np.mean(phi_off[seg]**2)))

fig, axes = plt.subplots(2, 1, figsize=(10.5, 7.2), sharex=True,
                         gridspec_kw={'height_ratios': [1.5, 1]})
ax = axes[0]
ax.plot(t, phi_off, color='#718096', lw=0.9, label='无 RRS（仅航向保持）')
ax.plot(t, phi_on, color='#C05621', lw=1.1, label='有 RRS（横摇反馈通道投入）')
ax.axhline(0, color='k', lw=0.6)
ax.set_ylabel('横摇角 $\\varphi$ (°)')
ax.legend(loc='upper right', fontsize=10.5)
ax.grid(True, ls=':', alpha=0.5)
ax.set_title(f'图 10-2  有无 RRS 的横摇角时域对比（示意仿真：二阶横摇模型，'
             f'$\\omega_\\varphi$={wphi} rad/s，舵速限 10°/s，横摇 RMS 削减约 {red:.0f}%）')

ax = axes[1]
ax.plot(t, d_on, color='#2B6CB0', lw=1.0, label='RRS 通道舵角（减摇分量）')
ax.set_xlabel('时间 (s)')
ax.set_ylabel('舵角 $\\delta$ (°)')
ax.legend(loc='upper right', fontsize=10.5)
ax.grid(True, ls=':', alpha=0.5)
ax.set_xlim(0, 120)
fig.tight_layout()
fig.savefig(OUT + 'fig_10_rrs_simulation.png', dpi=150)
plt.close(fig)

print('RMS roll reduction: %.1f%%' % red)
print('done')
