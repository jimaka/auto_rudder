# 生成 09_track_keeping.md 所用插图：
#   fig_09_los_geometry.png  LOS 制导几何示意
#   fig_09_los_variants.png  PLOS / ILOS / AILOS 定常流下横偏收敛对比（运动学仿真）
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Arc, Circle

plt.rcParams['font.sans-serif'] = ['AR PL UMing CN', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei']
plt.rcParams['axes.unicode_minus'] = False

FIGDIR = '/home/jim/Desktop/auto_rudder/docs/adaptive_autopilot_research/figs'

# ---------------------------------------------------------------- fig 1: LOS 几何
fig, ax = plt.subplots(figsize=(9.5, 5.6))

# 路径：WP1 -> WP2 沿 x 轴
ax.plot([-15, 215], [0, 0], color='tab:blue', lw=2.2)
ax.plot(0, 0, 'ks', ms=9)
ax.plot(200, 0, 'ks', ms=9)
ax.annotate(r'航路点 $WP_k$', (0, 0), xytext=(-6, 12), fontsize=11)
ax.annotate(r'航路点 $WP_{k+1}$', (200, 0), xytext=(168, 12), fontsize=11)
ax.annotate('期望路径（航迹线）', (150, 0), xytext=(26, 7), fontsize=11, color='tab:blue')

# 路径切向角 gamma_p
arc = Arc((20, 0), 26, 26, theta1=0, theta2=0, color='tab:blue')
ax.add_patch(Arc((20, 0), 26, 26, theta1=-40, theta2=0, color='tab:blue', lw=1.2))
ax.annotate(r'路径切向角 $\gamma_p$', (20, 0), xytext=(8, -22), fontsize=11, color='tab:blue',
            arrowprops=dict(arrowstyle='->', color='tab:blue', lw=1))

# 船位与投影点
ship = np.array([80.0, -30.0])
proj = np.array([80.0, 0.0])
ax.plot(*ship, 'o', ms=10, color='tab:red')
ax.annotate('船位 $(x,y)$', ship, xytext=(56, -46), fontsize=11, color='tab:red')
ax.plot(*proj, 'k+', ms=10, mew=2)
# 横向偏差 y_e
ax.annotate('', xy=proj, xytext=ship,
            arrowprops=dict(arrowstyle='<->', color='k', lw=1.4, linestyle='--'))
ax.annotate(r'横向偏差 $y_e$', (80, -15), xytext=(84, -18), fontsize=11)

# 前视距离 Δ 与 LOS 点
Delta = 48.0
los = np.array([80.0 + Delta, 0.0])
ax.annotate('', xy=los, xytext=proj,
            arrowprops=dict(arrowstyle='<->', color='tab:green', lw=1.6))
ax.annotate(r'前视距离 $\Delta$', (104, 0), xytext=(100, 8), fontsize=11, color='tab:green')
ax.plot(*los, marker='*', ms=16, color='tab:green')
ax.annotate('LOS 点', los, xytext=(134, -11), fontsize=11, color='tab:green')
ax.add_patch(Circle(proj, Delta, fill=False, ls=':', lw=1, color='tab:green', alpha=0.6))

# 期望艏向 psi_d（指向 LOS 点）
v_psi = los - ship
v_psi = v_psi / np.linalg.norm(v_psi)
ax.add_patch(FancyArrowPatch(ship, ship + 42 * v_psi, arrowstyle='-|>', mutation_scale=18,
                             color='tab:red', lw=2))
ang_psi = np.degrees(np.arctan2(v_psi[1], v_psi[0]))
ax.add_patch(Arc(ship, 34, 34, theta1=0, theta2=ang_psi, color='tab:red', lw=1.2))
ax.annotate(r'期望艏向 $\psi_d$', ship + 26 * v_psi, xytext=(102, -52), fontsize=11,
            color='tab:red', arrowprops=dict(arrowstyle='->', color='tab:red', lw=1))

# 实际航速方向：航迹向 chi = psi + beta（侧滑角 beta 由风流压差引起）
beta_deg = 9.0
ang_chi = np.radians(ang_psi - beta_deg)   # 流把船往下压，航迹向低于艏向
v_chi = np.array([np.cos(ang_chi), np.sin(ang_chi)])
ax.add_patch(FancyArrowPatch(ship, ship + 40 * v_chi, arrowstyle='-|>', mutation_scale=18,
                             color='tab:orange', lw=2))
ax.add_patch(Arc(ship, 22, 22, theta1=np.degrees(ang_chi), theta2=ang_psi,
                 color='tab:orange', lw=1.4))
ax.annotate(r'航迹向 $\chi=\psi+\beta$（对水速度矢量）', ship + 40 * v_chi,
            xytext=(40, -62), fontsize=11, color='tab:orange',
            arrowprops=dict(arrowstyle='->', color='tab:orange', lw=1))
ax.annotate(r'侧滑角 $\beta$', ship + 14 * np.array([np.cos(np.radians(ang_psi - beta_deg / 2)),
                                                     np.sin(np.radians(ang_psi - beta_deg / 2))]),
            xytext=(66, -58), fontsize=11, color='tab:orange')

# 海流矢量
cur0 = np.array([10.0, -58.0])
ax.add_patch(FancyArrowPatch(cur0, cur0 + np.array([26, 12]), arrowstyle='-|>',
                             mutation_scale=15, color='tab:cyan', lw=2))
ax.annotate(r'海流 $U_c$（低速漂流的来源之一）', cur0, xytext=(-12, -66), fontsize=11,
            color='tab:cyan')

ax.text(0.5, -0.13,
        'LOS 制导律：$\\psi_d=\\gamma_p+\\arctan(-y_e/\\Delta)$，'
        '使船艏始终指向路径上前视距离处的 LOS 点',
        transform=ax.transAxes, ha='center', fontsize=11.5)
ax.set_xlim(-20, 220)
ax.set_ylim(-72, 30)
ax.set_aspect('equal')
ax.axis('off')
fig.tight_layout()
fig.savefig(f'{FIGDIR}/fig_09_los_geometry.png', dpi=200, bbox_inches='tight')
plt.close(fig)

# ------------------------------------------------- fig 2: PLOS / ILOS / AILOS 对比仿真
U = 12.86          # 25 kn ≈ 12.86 m/s
beta_true = np.radians(5.0)   # 定常侧滑角（等效定常流/风压差）
Delta = 60.0       # 前视距离 m
tau = 2.0          # 航向自动驾驶仪一阶响应时间常数 s
dt = 0.05
T = 90.0
N = int(T / dt)
t = np.arange(N) * dt


def simulate(mode):
    """运动学仿真：ẋ=Ucos(ψ+β), ẏ=Usin(ψ+β), 航向回路一阶惯性。"""
    x, y, psi = 0.0, 60.0, 0.0
    y_int, beta_hat = 0.0, 0.0
    gam = 1e-4     # AILOS 自适应增益（Fossen 2015 律中 γ 需取小值，量级 ~1/(U·Δ)）
    k_i = 1.0      # ILOS 积分增益系数
    ys, psi_hist, bhat_hist = [], [], []
    for _ in range(N):
        y_e = y
        if mode == 'PLOS':
            psi_d = np.arctan(-y_e / Delta)
        elif mode == 'ILOS':
            den = Delta**2 + (y_e + y_int)**2
            psi_d = np.arctan(-(y_e + y_int) / Delta)
            y_int += dt * k_i * Delta * U * y_e / den
        else:  # AILOS (Fossen et al. 2015)
            den = np.sqrt(Delta**2 + (y_e + Delta * beta_hat)**2)
            psi_d = np.arctan(-y_e / Delta - beta_hat)
            beta_hat += dt * gam * U * Delta * y_e / den
        psi += dt * (psi_d - psi) / tau
        x += dt * U * np.cos(psi + beta_true)
        y += dt * U * np.sin(psi + beta_true)
        ys.append(y_e)
        psi_hist.append(psi)
        bhat_hist.append(np.degrees(beta_hat))
    return np.array(ys), np.array(bhat_hist)


y_plos, _ = simulate('PLOS')
y_ilos, _ = simulate('ILOS')
y_ailos, bhat = simulate('AILOS')

fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.2))
ax = axes[0]
ax.axhline(Delta * np.tan(beta_true), color='gray', ls=':', lw=1)
ax.annotate(r'PLOS 稳态偏差 $\approx\Delta\tan\beta$', (T * 0.55, Delta * np.tan(beta_true) + 0.4),
            fontsize=10, color='gray')
ax.plot(t, y_plos, label='PLOS（比例 LOS）', color='tab:blue', lw=1.8)
ax.plot(t, y_ilos, label='ILOS（积分 LOS）', color='tab:orange', lw=1.8)
ax.plot(t, y_ailos, label='AILOS（自适应侧滑补偿）', color='tab:green', lw=1.8)
ax.set_xlabel('时间 (s)')
ax.set_ylabel('横向偏差 $y_e$ (m)')
ax.set_title('定常侧滑（$\\beta=5°$）下三种制导律的横偏收敛')
ax.legend(fontsize=10)
ax.grid(alpha=0.3)

ax = axes[1]
ax.axhline(5.0, color='gray', ls=':', lw=1)
ax.annotate(r'真值 $\beta=5°$', (T * 0.6, 5.15), fontsize=10, color='gray')
ax.plot(t, bhat, color='tab:green', lw=1.8)
ax.set_xlabel('时间 (s)')
ax.set_ylabel(r'侧滑角估计 $\hat{\beta}$ (°)')
ax.set_title('AILOS 自适应律对侧滑角的在线估计')
ax.grid(alpha=0.3)
ax.set_ylim(0, 8)

fig.suptitle('LOS 制导变体对比（运动学仿真：$U=25$ kn，$\\Delta=60$ m，航向回路一阶滞后 $\\tau=2$ s）',
             fontsize=12)
fig.tight_layout(rect=[0, 0, 1, 0.93])
fig.savefig(f'{FIGDIR}/fig_09_los_variants.png', dpi=200, bbox_inches='tight')
plt.close(fig)
print('done')
