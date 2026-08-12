# -*- coding: utf-8 -*-
"""生成 05 章节示意图：NN 自适应航向控制结构框图 + PD 与 NN 自适应对比曲线"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

plt.rcParams['font.sans-serif'] = ['AR PL UMing CN', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei']
plt.rcParams['axes.unicode_minus'] = False

# ---------------- 图1：结构框图 ----------------
fig, ax = plt.subplots(figsize=(11.5, 5.6))
ax.set_xlim(0, 100); ax.set_ylim(0, 50); ax.axis('off')

def box(x, y, w, h, text, fc='#EAF2FB', ec='#2B5C9E'):
    b = FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.6',
                       facecolor=fc, edgecolor=ec, linewidth=1.6)
    ax.add_patch(b)
    ax.text(x + w/2, y + h/2, text, ha='center', va='center', fontsize=11)

def arrow(x1, y1, x2, y2, text='', fs=10, style='-|>', dy=1.8, color='k'):
    a = FancyArrowPatch((x1, y1), (x2, y2), arrowstyle=style,
                        mutation_scale=14, lw=1.4, color=color)
    ax.add_patch(a)
    if text:
        ax.text((x1+x2)/2, (y1+y2)/2 + dy, text, ha='center', fontsize=fs)

# 主回路
ax.text(6, 30, r'$\psi_d$', fontsize=14, ha='center')
ax.add_patch(plt.Circle((12, 30), 1.6, fill=False, lw=1.4))
ax.text(12, 30, r'$-$', fontsize=14, ha='center', va='center')
arrow(2.5, 30, 10.4, 30)
box(15, 26.5, 15, 7, '反步法虚拟控制律\n' + r'$\alpha_1=-c_1 z_1+\dot{\psi}_d$')
box(34, 26.5, 15, 7, '实际控制律（舵角指令）\n' + r'$\delta=\hat{f}^{-1}$ 逼近项+反馈项')
box(53, 26.5, 10, 7, '舵机\n饱和/速率限制')
box(67, 26.5, 16, 7, '船舶操纵模型\nNomoto/非线性\nK、T 时变+风浪干扰')
arrow(13.6, 30, 15, 30)
ax.text(14.6, 32.6, r'$z_1=\psi-\psi_d$', ha='center', fontsize=9)
arrow(30, 30, 34, 30)
arrow(49, 30, 53, 30, r'$\delta_c$', fs=9)
arrow(63, 30, 67, 30, r'$\delta$', fs=9)
arrow(83, 30, 95, 30, r'$\psi,r$', fs=9)
arrow(94, 30, 94, 12, '', color='k')
arrow(94, 12, 12, 12, '', color='k')
arrow(12, 12, 12, 28.4, '', color='k')

# NN 逼近器与自适应律（下方支路）
box(30, 4, 22, 8, 'RBF/MLP 神经网络逼近器\n' + r'$\hat{f}(Z)=\hat{W}^{T}h(Z)$' + '\n逼近模型不确定项 Δf', fc='#FDF3E3', ec='#B07B1E')
box(57, 4, 22, 8, '权值自适应律（Lyapunov 设计）\n' + r'$\dot{\hat{W}}=\Gamma(h(Z)z_2-\sigma\hat{W})$', fc='#FDF3E3', ec='#B07B1E')
arrow(75, 26.5, 68, 12.5, r'$z_2,r,\psi$', fs=9, color='#B07B1E')
arrow(52, 8, 57, 8, '', color='#B07B1E')
arrow(68, 12.5, 45, 26.5, r'$\hat{f}(Z)$ 前馈补偿', fs=9, color='#B07B1E')
ax.text(50, 47, '神经网络自适应（反步法）船舶航向控制结构框图', fontsize=13, ha='center')
fig.savefig('/home/jim/Desktop/auto_rudder/docs/adaptive_autopilot_research/figs/fig_05_nn_adaptive_structure.png',
            dpi=160, bbox_inches='tight')
plt.close(fig)

# ---------------- 图2：PD 固定增益 vs NN 自适应 原理对比 ----------------
t = np.linspace(0, 300, 3000)
# 海况在 t=120 时从 2 级突增至 5 级：等效干扰与 K/T 漂移
dt = t[1] - t[0]

def simulate(kind):
    """二阶 Nomoto 型简化仿真。kind: 'pd' 固定增益 / 'nn' 自适应前馈（在线学习慢变干扰）"""
    K, T = 0.12, 9.0
    psi, r = 0.0, 0.0
    out, err, dnorm = [], [], []
    d_hat = 0.0  # 慢变干扰（风流压差等效）估计
    gamma, sigma = 0.02, 0.002
    for ti in t:
        psi_d = 30.0 if ti >= 20 else 0.0
        # 海况突变：K/T 漂移 + 慢变干扰（风流）与波浪高频分量同时增强
        if ti >= 120:
            Ke, Te = K*1.8, T*1.5
            d_wave = 1.8 + 0.8*np.sin(0.05*ti) + 0.8*np.sin(0.9*ti)
        else:
            Ke, Te = K, T
            d_wave = 0.3 + 0.15*np.sin(0.9*ti)
        e = psi - psi_d
        delta = -(1.6*e + 12.0*r)
        if kind == 'nn':
            # 自适应律：仅在航向保持段（|e| 小）学习，含 sigma 泄漏保证有界
            if abs(e) < 8.0:
                d_hat += dt * (gamma*e - sigma*d_hat)
            d_hat = np.clip(d_hat, -6, 6)
            delta -= d_hat / K  # 前馈补偿，按标称增益折算到舵角
        u = np.clip(delta, -35, 35)
        r_dot = (-r + Ke*u)/Te + d_wave/Te
        r += r_dot*dt
        psi += r*dt
        out.append(psi); err.append(e); dnorm.append(abs(d_hat))
    return np.array(out), np.array(err), np.array(dnorm)

psi_pd, e_pd, _ = simulate('pd')
psi_nn, e_nn, W_nn = simulate('nn')

fig, axes = plt.subplots(2, 1, figsize=(9.5, 6.8), sharex=True,
                         gridspec_kw={'height_ratios': [2, 1]})
axes[0].axhline(30, color='gray', ls='--', lw=1, label='期望航向 30°')
axes[0].plot(t, psi_pd, color='#C0392B', lw=1.4, label='PD 固定增益（无自适应）')
axes[0].plot(t, psi_nn, color='#1F6FB2', lw=1.4, label='NN 自适应（前馈在线学习）')
axes[0].axvline(120, color='k', ls=':', lw=1)
axes[0].text(124, 24.5, '海况突变：K/T 漂移 + 风流/波浪干扰增强', fontsize=10)
axes[0].set_ylim(-1, 38)
axes[0].set_ylabel('航向角 ψ (°)')
axes[0].set_title('海况突变下航向保持性能对比（原理示意仿真）')
axes[0].legend(loc='lower right', fontsize=10)
axes[0].grid(alpha=0.3)

axes[1].plot(t, np.abs(e_pd), color='#C0392B', lw=1.2, label='|e|：PD')
axes[1].plot(t, np.abs(e_nn), color='#1F6FB2', lw=1.2, label='|e|：NN 自适应')
ax2 = axes[1].twinx()
ax2.plot(t, W_nn, color='#B07B1E', lw=1.2, ls='-.', label='自适应权值 |d̂|（有界收敛）')
ax2.set_ylabel('|d̂|', color='#B07B1E')
ax2.tick_params(axis='y', labelcolor='#B07B1E')
axes[1].set_xlabel('时间 t (s)')
axes[1].set_ylabel('航向误差 |e| (°)')
lines1, lab1 = axes[1].get_legend_handles_labels()
lines2, lab2 = ax2.get_legend_handles_labels()
axes[1].legend(lines1+lines2, lab1+lab2, loc='upper right', fontsize=9)
axes[1].grid(alpha=0.3)
fig.savefig('/home/jim/Desktop/auto_rudder/docs/adaptive_autopilot_research/figs/fig_05_nn_vs_pd_response.png',
            dpi=160, bbox_inches='tight')
plt.close(fig)
print('done')
