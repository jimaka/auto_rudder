# 生成 fig_01_sigma_modification.png：
# 舵机饱和 + 波浪扰动下，理想 Lyapunov MRAC 与 σ-modification / e-modification 的参数漂移对比
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['AR PL UMing CN', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei']
plt.rcParams['axes.unicode_minus'] = False

# 对象：一阶 Nomoto 型  T*dy/dt + y = K*sat(u) + d
a, b = 0.05, 0.08          # 对象参数（控制器未知真值）
am, bm = 0.10, 0.10        # 参考模型  y_m' = -am y_m + bm r
th1_star, th2_star = bm / b, (am - a) / b
u_max = 0.6                # 舵角饱和限幅（归一化）
gamma = 0.5                # 自适应增益
sigma = 0.05               # 修正系数
dt, T_sim = 0.02, 2000.0
t = np.arange(0, T_sim, dt)

def ref_input(tt):
    # 周期性改向指令（度，归一化），每 400 s 换向一次
    return 1.0 if (tt // 400) % 2 == 0 else -1.0

def run(mode):
    y = ym = 0.0
    th1, th2 = 1.0, 0.4    # 初始猜测偏离真值
    out = []
    for tt in t:
        r = ref_input(tt)
        d = 0.35 * np.sin(2 * np.pi * 0.08 * tt) + 0.15 * np.sin(2 * np.pi * 0.17 * tt)  # 波浪扰动
        u = th1 * r - th2 * y
        u_sat = np.clip(u, -u_max, u_max)
        e = y - ym
        # 自适应律
        leak1 = leak2 = 0.0
        if mode == 'sigma':        # σ-modification
            leak1, leak2 = sigma * th1, sigma * th2
        elif mode == 'e':          # e-modification
            leak1, leak2 = sigma * abs(e) * th1, sigma * abs(e) * th2
        dth1 = -gamma * e * r - gamma * leak1
        dth2 = gamma * e * y - gamma * leak2
        th1 += dth1 * dt
        th2 += dth2 * dt
        ym += (-am * ym + bm * r) * dt
        y += (-a * y + b * u_sat + d) * dt
        out.append((y, ym, e, th1, th2))
    return np.array(out)

res = {m: run(m) for m in ['ideal', 'sigma', 'e']}

fig, axes = plt.subplots(1, 2, figsize=(12.4, 4.4))
styles = {'ideal': ('理想 Lyapunov 律（无修正）', '#c0392b', '-'),
          'sigma': ('σ-modification（σ=0.05）', '#1f77b4', '-'),
          'e': ('e-modification（σ=0.05）', '#2ca02c', '--')}

ax = axes[0]
for m, (label, c, ls) in styles.items():
    ax.plot(t, res[m][:, 4], ls, color=c, lw=1.4, label=label)
ax.axhline(th2_star, color='k', ls=':', lw=1.2, label=r'理想匹配值 $\theta_2^*$')
ax.set_xlabel('时间（s）')
ax.set_ylabel(r'反馈增益自适应参数 $\theta_2$')
ax.set_title('(a)  饱和 + 波浪扰动下的参数漂移')
ax.grid(alpha=0.3)
ax.legend(fontsize=9, loc='upper left')

ax = axes[1]
for m, (label, c, ls) in styles.items():
    e = np.abs(res[m][:, 2])
    # 滑动包络（100 s 窗口最大值）便于观察趋势
    w = int(100 / dt)
    env = np.array([e[max(0, i - w):i + 1].max() for i in range(0, len(e), w // 2)])
    te = t[::w // 2][:len(env)]
    ax.plot(te, env, ls, color=c, lw=1.4, label=label)
ax.set_xlabel('时间（s）')
ax.set_ylabel(r'跟踪误差包络 $|e|$（100 s 窗口最大）')
ax.set_title('(b)  跟踪误差包络对比')
ax.grid(alpha=0.3)
ax.legend(fontsize=9)

fig.tight_layout()
fig.savefig('/home/jim/Desktop/auto_rudder/docs/adaptive_autopilot_research/figs/fig_01_sigma_modification.png', dpi=150)
print('saved')
