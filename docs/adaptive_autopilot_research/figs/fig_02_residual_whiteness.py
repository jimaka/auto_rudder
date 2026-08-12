# 生成 fig_02_residual_whiteness.png：辨识残差白性检验示意
# 左：模型-噪声结构匹配时，RLS 预报误差近似白噪声，ACF 落在 Bartlett 95% 置信带内
# 右：噪声模型失配（未建波浪有色噪声）时，残差 ACF 显著出带 -> 参数估计有偏风险
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['AR PL UMing CN', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei']
plt.rcParams['axes.unicode_minus'] = False

rng = np.random.default_rng(42)
N = 2000          # 滑窗样本数
m = 40            # 最大滞后数

# 白残差：模型结构正确时的理想创新序列
e_white = rng.standard_normal(N)

# 有色残差：白噪声经二阶波浪成型滤波器（谐振峰约 0.1 Hz，采样 1 Hz）
# H(z) = 1 / (1 - 2*r*cos(w0) z^-1 + r^2 z^-2)
r, w0 = 0.97, 2 * np.pi * 0.1
a1, a2 = -2 * r * np.cos(w0), r**2
e_col = np.zeros(N)
w = rng.standard_normal(N)
for t in range(2, N):
    e_col[t] = -a1 * e_col[t - 1] - a2 * e_col[t - 2] + w[t]
e_col /= np.std(e_col)


def acf(x, m):
    x = x - x.mean()
    r0 = np.dot(x, x) / len(x)
    return np.array([np.dot(x[:-k], x[k:]) / (len(x) - k) / r0 for k in range(1, m + 1)])


lag = np.arange(1, m + 1)
band = 1.96 / np.sqrt(N)   # Bartlett 渐近 95% 置信带

fig, axes = plt.subplots(1, 2, figsize=(10, 3.8), sharey=True)
for ax, e, title, c in [
    (axes[0], e_white, "噪声模型匹配：残差近似白噪声", "#1f77b4"),
    (axes[1], e_col, "噪声模型失配：波浪频段残差有色", "#d62728"),
]:
    rho = acf(e, m)
    ax.bar(lag, rho, width=0.6, color=c, alpha=0.8)
    ax.axhline(band, ls="--", color="k", lw=1)
    ax.axhline(-band, ls="--", color="k", lw=1)
    ax.axhline(0, color="k", lw=0.5)
    ax.set_xlabel("滞后数 $\\tau$（采样周期）")
    ax.set_title(title, fontsize=11)
    out = np.mean(np.abs(rho) > band) * 100
    ax.text(0.98, 0.95, f"出带比例 {out:.0f}%", transform=ax.transAxes,
            ha="right", va="top", fontsize=10,
            bbox=dict(boxstyle="round", fc="white", ec="gray", alpha=0.9))
axes[0].set_ylabel("归一化自相关 $\\hat{\\rho}(\\tau)$")
axes[0].text(2, band * 2.2, "Bartlett 95% 置信带 $\\pm1.96/\\sqrt{N}$", fontsize=9)
fig.suptitle("RLS 辨识残差的白性检验（$N=2000$ 滑窗）", fontsize=12)
fig.tight_layout(rect=[0, 0, 1, 0.93])
out = "/home/jim/Desktop/auto_rudder/docs/adaptive_autopilot_research/figs/fig_02_residual_whiteness.png"
fig.savefig(out, dpi=150)
print("saved:", out)
