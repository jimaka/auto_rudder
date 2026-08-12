# 生成 fig_06_sta_discretization.png：超螺旋算法显式/隐式离散化对比
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['AR PL UMing CN', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei']
plt.rcParams['axes.unicode_minus'] = False

# 被控滑模变量动态: ds/dt = u + d(t), STA: u = -k1*|s|^0.5*sign(s) + z, dz/dt = -k2*sign(s)
k1, k2 = 1.8, 1.5
T_end = 30.0
s0, z0 = 3.0, 0.0

def d(t):
    return 0.6 * np.sin(0.5 * t) + 0.2 * np.sin(2.3 * t)

def sign(x):
    return 1.0 if x > 0 else (-1.0 if x < 0 else 0.0)

def sim_explicit(h):
    n = int(T_end / h)
    t = np.arange(n) * h
    s = np.zeros(n); z = np.zeros(n); u = np.zeros(n)
    s[0], z[0] = s0, z0
    for k in range(n - 1):
        u[k] = -k1 * np.sqrt(abs(s[k])) * sign(s[k]) + z[k]
        s[k + 1] = s[k] + h * (u[k] + d(t[k]))
        z[k + 1] = z[k] - h * k2 * sign(s[k])
    u[-1] = -k1 * np.sqrt(abs(s[-1])) * sign(s[-1]) + z[-1]
    return t, s, u

def sim_implicit(h):
    # Brogliato-Polyakov-Efimov 隐式离散化（标量闭式解）
    n = int(T_end / h)
    t = np.arange(n) * h
    s = np.zeros(n); z = np.zeros(n); u = np.zeros(n)
    s[0], z[0] = s0, z0
    for k in range(n - 1):
        A = s[k] + h * (d(t[k]) + z[k])
        if abs(A) <= h * h * k2:
            s_new = 0.0
        else:
            sg = 1.0 if A > 0 else -1.0
            rhs = abs(A) - h * h * k2          # y + h*k1*sqrt(y) = rhs
            sq = (-h * k1 + np.sqrt(h * h * k1 * k1 + 4.0 * rhs)) / 2.0
            s_new = sg * sq * sq
        z_new = z[k] - h * k2 * sign(s_new)
        u[k] = -k1 * np.sqrt(abs(s_new)) * sign(s_new) + z_new
        s[k + 1], z[k + 1] = s_new, z_new
    u[-1] = -k1 * np.sqrt(abs(s[-1])) * sign(s[-1]) + z[-1]
    return t, s, u

t_f, s_f, u_f = sim_explicit(0.002)    # 细采样参考
t_c, s_c, u_c = sim_explicit(0.08)     # 粗采样显式 Euler
t_i, s_i, u_i = sim_implicit(0.08)     # 粗采样隐式离散化

fig, axes = plt.subplots(2, 1, figsize=(8.6, 6.2), sharex=True)

ax = axes[0]
ax.plot(t_f, s_f, color='0.35', lw=1.2, label='显式 Euler，$T_s$=2 ms（近似连续参考）')
ax.plot(t_c, s_c, color='#d62728', lw=1.2, label='显式 Euler，$T_s$=80 ms')
ax.plot(t_i, s_i, color='#1f77b4', lw=1.4, label='隐式离散化，$T_s$=80 ms')
ax.set_ylabel('滑模面 $s$')
ax.set_title('超螺旋算法（STA）离散化方式对比（$k_1$=1.8, $k_2$=1.5，扰动 $|\\dot d|\\leq$ 0.76）')
ax.legend(loc='upper right', fontsize=9)
ax.grid(alpha=0.3)
ax.set_ylim(-0.6, 3.3)
axin = ax.inset_axes([0.06, 0.10, 0.52, 0.55])
for t_, s_, c_, lw_ in [(t_f, s_f, '0.35', 1.0), (t_c, s_c, '#d62728', 1.0), (t_i, s_i, '#1f77b4', 1.2)]:
    m = t_ > 12
    axin.plot(t_[m], s_[m], color=c_, lw=lw_)
axin.set_ylim(-0.045, 0.045)
axin.set_title('收敛后局部放大', fontsize=8)
axin.grid(alpha=0.3)

ax = axes[1]
m_f = t_f > 12; m_c = t_c > 12; m_i = t_i > 12
ax.plot(t_f[m_f], u_f[m_f], color='0.35', lw=1.0, label='显式 Euler，$T_s$=2 ms')
ax.plot(t_c[m_c], u_c[m_c], color='#d62728', lw=1.0, label='显式 Euler，$T_s$=80 ms（数值抖振）')
ax.plot(t_i[m_i], u_i[m_i], color='#1f77b4', lw=1.4, label='隐式离散化，$T_s$=80 ms（平滑）')
ax.set_ylabel('控制量 $u$')
ax.set_xlabel('时间 $t$ / s')
ax.legend(loc='upper right', fontsize=9)
ax.grid(alpha=0.3)

fig.tight_layout()
out = '/home/jim/Desktop/auto_rudder/docs/adaptive_autopilot_research/figs/fig_06_sta_discretization.png'
fig.savefig(out, dpi=150)
print('saved', out)

# 打印收敛后抖振幅值供正文引用
print('explicit 80ms |s| max after 12s:', np.abs(s_c[t_c > 12]).max())
print('implicit 80ms |s| max after 12s:', np.abs(s_i[t_i > 12]).max())
print('explicit 80ms u ptp after 12s:', np.ptp(u_c[t_c > 12]))
print('implicit 80ms u ptp after 12s:', np.ptp(u_i[t_i > 12]))
print('d derivative bound:', 0.6 * 0.5 + 0.2 * 2.3)
