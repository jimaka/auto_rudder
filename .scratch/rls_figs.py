#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RLS 在线辨识原理示意图（5 张），输出到 assets/rls_math_*.png
事实来源：control/include/nomoto_identifier.hpp / auto_tuner.hpp
公式一律走 mathtext（Noto Sans CJK SC 缺下标/组合符字形）
合成示例船：K=0.08，T=5 s（响应较快，Z 形 ±15° 下激励门控的 dr 通道可触发）
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

for f in [".scratch/fonts/NotoSansCJKsc-Regular.otf", ".scratch/fonts/NotoSansCJKsc-Bold.otf"]:
    font_manager.fontManager.addfont(f)
plt.rcParams.update({
    "font.family": "Noto Sans CJK SC",
    "axes.unicode_minus": False,
    "figure.facecolor": "white",
    "savefig.facecolor": "white",
    "axes.edgecolor": "#666666",
    "axes.linewidth": 0.9,
    "font.size": 10.5,
    "mathtext.fontset": "dejavusans",
})
DPI = 180
OUT = "assets"

C = dict(
    orange=("#fdf0e2", "#e8873a"),
    blue=("#e8f1f9", "#4f86b8"),
    green=("#e9f5ea", "#57a05a"),
    purple=("#f1eaf7", "#8064a8"),
    red=("#fbeaea", "#c25b5b"),
    gray=("#f2f2f2", "#888888"),
)

def box(ax, x, y, w, h, title, body, color, title_fs=11, body_fs=9.5, ec_lw=1.6):
    fc, ec = color
    b = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.006,rounding_size=0.012",
                       fc=fc, ec=ec, lw=ec_lw, mutation_aspect=1.0,
                       transform=ax.transAxes, zorder=2)
    ax.add_patch(b)
    if title:
        ax.text(x + w/2, y + h - 0.032, title, ha="center", va="top",
                fontsize=title_fs, fontweight="bold", transform=ax.transAxes, zorder=3)
    if body:
        ax.text(x + w/2, y + h - (0.082 if title else 0.028), body, ha="center", va="top",
                fontsize=body_fs, transform=ax.transAxes, zorder=3, linespacing=1.55)

def arrow(ax, p1, p2, color="#555555", lw=1.6, style="-|>", ls="-", conn="arc3,rad=0.0",
          label=None, fs=9, loffset=(0, 0.02)):
    a = FancyArrowPatch(p1, p2, arrowstyle=style, mutation_scale=14, lw=lw, color=color,
                        linestyle=ls, connectionstyle=conn, transform=ax.transAxes, zorder=4)
    ax.add_patch(a)
    if label:
        mx, my = (p1[0]+p2[0])/2 + loffset[0], (p1[1]+p2[1])/2 + loffset[1]
        ax.text(mx, my, label, fontsize=fs, color=color, ha="center", transform=ax.transAxes, zorder=5)

def diagram_ax(figsize, title):
    fig = plt.figure(figsize=figsize)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
    fig.suptitle(title, fontsize=15, fontweight="bold", y=0.985)
    return fig, ax

# ============================================================
# 合成船舶数据：一阶 Nomoto  T·ṙ + r = K·δ
# ============================================================
def sim_ship(delta_fn, T, K, dt, n, noise=0.0, seed=1):
    rng = np.random.default_rng(seed)
    r = np.zeros(n); d = np.zeros(n)
    for k in range(1, n):
        d[k] = delta_fn(k * dt)
        r[k] = r[k-1] + dt * (K * d[k] - r[k-1]) / T
    if noise > 0:
        r = r + rng.normal(0, noise, n)
        d = d + rng.normal(0, noise * 2, n)
    return d, r

def zigzag(amp=15.0, period=40.0):
    """每 period/4 换向一次（period=40 → 每 10 s 换向）"""
    def f(t):
        ph = (t % period) / period
        return amp if (ph < 0.25 or 0.5 <= ph < 0.75) else -amp
    return f

K_TRUE, T_TRUE = 0.08, 5.0   # 合成示例船参数

# ============================================================
# 图 1 rls_math_regression.png —— 回归化：辨识 = 噪声数据上拟合平面
# ============================================================
def fig1():
    dt = 0.02
    rng = np.random.default_rng(3)
    d, r = sim_ship(zigzag(15.0, 40.0), T_TRUE, K_TRUE, dt, n=16000, noise=0.0)
    dr = (K_TRUE * d - r) / T_TRUE               # 模型一致的真实 ṙ
    dm = d + rng.normal(0, 0.15, len(d))          # 量测噪声
    rm = r + rng.normal(0, 0.08, len(r))
    drm = dr + rng.normal(0, 0.012, len(dr))
    m = (np.abs(dm) > 3.0) & ((np.abs(rm) > 0.5) | (np.abs(drm) > 0.2))   # 激励门控
    phi1, phi2, y = dm[m], -drm[m], rm[m]
    A = np.column_stack([phi1, phi2])
    theta, *_ = np.linalg.lstsq(A, y, rcond=None)   # θ = [K, T]

    # 偏回归（added-variable）分量：e_y vs e_regressor，斜率精确等于对应系数
    def partial(x, z, yv):
        ez = z - x * (x @ z / (x @ x))
        ey = yv - x * (x @ yv / (x @ x))
        return ez, ey
    e2, ey1 = partial(phi2, phi1, y)    # 控制 φ2 后：斜率 = K̂
    e1, ey2 = partial(phi1, phi2, y)    # 控制 φ1 后：斜率 = T̂

    fig = plt.figure(figsize=(14.5, 5.6))
    fig.suptitle("回归化：把 Nomoto 方程写成 $y=\\varphi^T\\theta$，辨识 = 在噪声数据上拟合平面",
                 fontsize=14, fontweight="bold", y=0.985)
    fig.text(0.5, 0.94,
             "$T\\dot{r}+r=K\\delta\\;\\Rightarrow\\;r=K\\delta-T\\dot{r}$"
             "：取 $y=r$，$\\varphi=[\\delta,\\,-\\dot{r}]^T$，$\\theta=[K,\\,T]^T$"
             "（代码：nomoto_identifier.hpp:3-4）",
             ha="center", va="top", fontsize=10.5, color="#444444")

    idx = np.random.default_rng(0).choice(len(y), 900, replace=False)

    ax3 = fig.add_subplot(131, projection="3d")
    ax3.scatter(phi1[idx], phi2[idx], y[idx], s=4, c="#4f86b8", alpha=0.45)
    g1 = np.linspace(phi1.min(), phi1.max(), 12)
    g2 = np.linspace(phi2.min(), phi2.max(), 12)
    G1, G2 = np.meshgrid(g1, g2)
    ax3.plot_surface(G1, G2, theta[0]*G1 + theta[1]*G2, color="#e8873a", alpha=0.45)
    ax3.set_xlabel("$\\varphi_1=\\delta$ (deg)", fontsize=9, labelpad=1)
    ax3.set_ylabel("$\\varphi_2=-\\dot{r}$ (deg/s$^2$)", fontsize=9, labelpad=1)
    ax3.set_zlabel("$y=r$ (deg/s)", fontsize=9, labelpad=1)
    ax3.set_title(f"拟合平面 $\\hat{{y}}=\\hat{{K}}\\delta-\\hat{{T}}\\dot{{r}}$\n"
                  f"$\\hat{{K}}$={theta[0]:.3f}（真值 {K_TRUE}），$\\hat{{T}}$={theta[1]:.1f} s（真值 {T_TRUE} s）",
                  fontsize=9.5)
    ax3.view_init(elev=18, azim=-128)
    ax3.tick_params(labelsize=7.5)

    ax = fig.add_subplot(132)
    ax.scatter(e2[idx], ey1[idx], s=5, c="#4f86b8", alpha=0.4)
    xs = np.array([e2.min(), e2.max()])
    ax.plot(xs, theta[0]*xs, color="#c25b5b", lw=2)
    ax.set_xlabel("$\\delta$ 去掉 $-\\dot{r}$ 分量的残差 (deg)")
    ax.set_ylabel("$r$ 去掉 $-\\dot{r}$ 分量的残差 (deg/s)")
    ax.set_title(f"偏回归看 $K$：控制另一回归量后，\n残差对残差的斜率 = $\\hat{{K}}$ = {theta[0]:.3f}",
                 fontsize=9.5)
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(133)
    ax.scatter(e1[idx], ey2[idx], s=5, c="#57a05a", alpha=0.4)
    xs = np.array([e1.min(), e1.max()])
    ax.plot(xs, theta[1]*xs, color="#c25b5b", lw=2)
    ax.set_xlabel("$-\\dot{r}$ 去掉 $\\delta$ 分量的残差 (deg/s$^2$)")
    ax.set_ylabel("$r$ 去掉 $\\delta$ 分量的残差 (deg/s)")
    ax.set_title(f"偏回归看 $T$：控制另一回归量后，\n残差对残差的斜率 = $\\hat{{T}}$ = {theta[1]:.1f} s",
                 fontsize=9.5)
    ax.grid(alpha=0.3)

    fig.text(0.5, 0.012,
             "合成示例船：一阶 Nomoto（$K$=0.08，$T$=5 s）+ Z 形舵 ±15° + 量测噪声；仅画满足激励门控的样本。"
             "离批最小二乘给出平面，RLS 做的是同一平面的逐拍递推。",
             ha="center", fontsize=9, color="#666666")
    fig.subplots_adjust(left=0.06, right=0.98, top=0.82, bottom=0.14, wspace=0.3)
    fig.savefig(f"{OUT}/rls_math_regression.png", dpi=DPI)
    plt.close(fig)
    print("fig1 K,T =", theta)

# ============================================================
# 图 2 rls_math_update.png —— RLS 递推一拍的信息流
# ============================================================
def fig2():
    fig, ax = diagram_ax((13.5, 7.2), "带遗忘因子的 RLS：一拍递推的信息流（nomoto_identifier.hpp:75-106）")

    box(ax, 0.02, 0.62, 0.150, 0.24, "新样本拍",
        "$y_k=r_k$\n$\\varphi_k=[\\delta/50,\\,-dr]^T$\n$dr=(r_k-r_{k-1})/\\Delta t$", C["orange"])
    box(ax, 0.215, 0.62, 0.150, 0.24, "① 预测",
        "$\\hat{y}_k=\\varphi_k^T\\theta_{k-1}$", C["blue"])
    box(ax, 0.41, 0.62, 0.150, 0.24, "② 残差（新息）",
        "$e_k=y_k-\\hat{y}_k$", C["blue"])
    box(ax, 0.605, 0.62, 0.185, 0.24, "③ 增益",
        "$L_k=P_{k-1}\\varphi_k\\,/\\,(\\lambda+\\varphi_k^T P_{k-1}\\varphi_k)$\n$\\lambda=0.98$", C["green"])
    box(ax, 0.835, 0.62, 0.150, 0.24, "输出估计",
        "$\\hat{K}=\\theta_0/50$\n$\\hat{T}=\\theta_1/1$\n（反归一化）", C["purple"])
    arrow(ax, (0.170, 0.74), (0.215, 0.74))
    arrow(ax, (0.365, 0.74), (0.410, 0.74))
    arrow(ax, (0.560, 0.74), (0.605, 0.74), label="$e_k$", loffset=(0, 0.025))
    arrow(ax, (0.790, 0.74), (0.835, 0.74))

    box(ax, 0.605, 0.20, 0.185, 0.24, "④ 参数更新",
        "$\\theta_k=\\theta_{k-1}+L_k e_k$\n$\\theta=[K,\\,T]^T$（归一化域）", C["green"])
    box(ax, 0.215, 0.20, 0.19, 0.24, "⑤ 协方差更新",
        "$P_k=(P_{k-1}-L_k\\varphi_k^T P_{k-1})/\\lambda$\n对角元保底 $P\\geq 10^{-3}$（防塌陷）", C["red"])
    arrow(ax, (0.6975, 0.62), (0.6975, 0.44), label="$L_k,\\ e_k$", loffset=(0.048, 0.0))
    arrow(ax, (0.605, 0.32), (0.405, 0.32))
    arrow(ax, (0.405, 0.32), (0.10, 0.32), color="#888888")
    arrow(ax, (0.10, 0.32), (0.10, 0.62), color="#888888")
    ax.text(0.2525, 0.285, "$P_k$、$\\theta_k$ 留待下一拍（递归状态，无需存历史数据）",
            fontsize=9, color="#888888", ha="center", transform=ax.transAxes)
    arrow(ax, (0.79, 0.32), (0.91, 0.32), color="#888888")
    arrow(ax, (0.91, 0.32), (0.91, 0.62), color="#888888", label="$\\theta_k$", loffset=(0.032, 0.0))

    ax.text(0.5, 0.085,
            "关键性质：每拍 O($n^2$)（$n$=2，仅 2×2 矩阵）；$\\lambda$<1 让旧样本权重指数衰减 → 可跟踪 $K$/$T$ 慢时变；\n"
            "$\\lambda$ 出现在增益分母 → 新样本相对旧信息权重被放大；$P$ 保底防止长期运行后增益熄零，保持自适应能力。",
            ha="center", fontsize=10, color="#333333")
    ax.text(0.5, 0.015,
            "代码位置：增益 nomoto_identifier.hpp:79-85 / 残差与 θ 更新 :88-94 / P 更新与保底 :96-106 / 反归一化 :114-115",
            ha="center", fontsize=8.5, color="#777777")
    fig.savefig(f"{OUT}/rls_math_update.png", dpi=DPI)
    plt.close(fig)

# ============================================================
# 图 3 rls_math_forgetting.png —— 遗忘因子：权重衰减 + 偏差-方差权衡
# ============================================================
def rls_run(d, r, dr, beat_idx, lam):
    """按 nomoto_identifier.hpp 语义做 1 Hz RLS（归一化 φ、门控、P 保底），返回每拍 K̂"""
    th = np.zeros(2); P = np.eye(2)   # p0 = 1.0
    out = []
    for k in beat_idx:
        phi = np.array([d[k]/50.0, -dr[k]/1.0])
        if abs(d[k]) <= 3.0 or not (abs(r[k]) > 0.5 or abs(dr[k]) > 0.2):
            out.append(th[0]/50.0); continue
        Pphi = P @ phi
        den = lam + phi @ Pphi
        L = Pphi / den
        e = r[k] - phi @ th
        th = th + L * e
        P = (P - np.outer(L, Pphi)) / lam
        P[0, 0] = max(P[0, 0], 1e-3); P[1, 1] = max(P[1, 1], 1e-3)
        out.append(th[0]/50.0)
    return np.array(out)

def fig3():
    fig = plt.figure(figsize=(13.5, 5.4))
    fig.suptitle("遗忘因子 $\\lambda$：旧样本指数失权 → 用方差换时变跟踪能力", fontsize=14, fontweight="bold")

    # --- 左：权重衰减 ---
    ax = fig.add_subplot(121)
    age = np.arange(0, 151)
    for lam, c, ls in [(0.98, "#4f86b8", "-"), (0.95, "#e8873a", "-"), (1.0, "#888888", "--")]:
        ax.plot(age, lam**age, color=c, ls=ls, lw=2, label=f"$\\lambda$ = {lam}")
    ax.axhline(1/np.e, color="#c25b5b", lw=1, ls=":")
    ax.axvline(50, color="#4f86b8", lw=1, ls=":")
    ax.annotate("有效记忆窗 $1/(1-\\lambda)$ = 50 拍\n（权重降至 1/e ≈ 37% 附近）",
                xy=(50, 1/np.e), xytext=(60, 0.55),
                arrowprops=dict(arrowstyle="->", color="#333333"), fontsize=9.5)
    ax.text(125, 0.93, "$\\lambda$=1：永不遗忘\n（退化为普通最小二乘）", fontsize=8.5, color="#888888", ha="center")
    ax.set_xlabel("样本年龄 $j$（拍，1 拍 = 1 s）")
    ax.set_ylabel("权重 $w=\\lambda^j$")
    ax.set_title("代价函数 $J=\\sum_k\\lambda^{N-k}e_k^2$ 中的样本权重", fontsize=10.5)
    ax.grid(alpha=0.3); ax.legend(fontsize=9.5, loc="lower left")
    ax.set_ylim(0, 1.02)

    # --- 右：K 阶跃跟踪 Monte-Carlo ---
    dt = 0.02; n = 20000
    t = np.arange(n) * dt
    beat = np.arange(50, n, 50)          # 1 Hz 拍
    tb = beat * dt
    K_of_t = np.where(t < 200, 0.08, 0.11)   # t=200 s 处 K 阶跃（载况/航速变化）
    runs = {0.98: [], 1.0: []}
    for seed in range(12):
        rng = np.random.default_rng(seed)
        dc = np.array([zigzag()(tt) for tt in t])
        rc = np.zeros(n)
        for k in range(1, n):
            rc[k] = rc[k-1] + dt * (K_of_t[k]*dc[k] - rc[k-1]) / T_TRUE
        d = dc + rng.normal(0, 0.15, n)
        r = rc + rng.normal(0, 0.08, n)
        dr = (K_of_t*dc - rc)/T_TRUE + rng.normal(0, 0.012, n)
        for lam in runs:
            runs[lam].append(rls_run(d, r, dr, beat, lam))
    m98 = np.mean(runs[0.98], axis=0); s98 = np.std(runs[0.98], axis=0)
    m10 = np.mean(runs[1.0], axis=0);  s10 = np.std(runs[1.0], axis=0)
    # 实测跟踪时间：阶跃后 K̂ 均值进入新真值 −10% 带内的耗时
    band = 0.11 - 0.003
    post = tb > 200
    i_tr = np.argmax(m98[post] > band)
    t_track = tb[post][i_tr] - 200 if i_tr > 0 else float("nan")

    ax2 = fig.add_subplot(122)
    ax2.axvline(200, color="#333333", lw=1, ls="--")
    ax2.text(200, 0.116, "$t$=200 s：$K$ 真值 0.08 → 0.11（载况/航速变化）",
             fontsize=8.5, ha="center")
    ax2.plot(tb, np.where(tb < 200, 0.08, 0.11), color="#333333", lw=1.5, ls="--", label="$K$ 真值")
    ax2.plot(tb, m98, color="#4f86b8", lw=1.8, label="$\\hat{K}$ 均值（$\\lambda$=0.98）")
    ax2.fill_between(tb, m98-s98, m98+s98, color="#4f86b8", alpha=0.18)
    ax2.plot(tb, m10, color="#888888", lw=1.8, label="$\\hat{K}$ 均值（$\\lambda$=1）")
    ax2.fill_between(tb, m10-s10, m10+s10, color="#888888", alpha=0.18)
    ax2.annotate(f"$\\lambda$=0.98：约 {t_track:.0f} s 收敛到新真值\n（门控限流，慢于理论值 50 s）",
                 xy=(392, 0.1075), xytext=(272, 0.086),
                 arrowprops=dict(arrowstyle="->", color="#4f86b8"), fontsize=9.5, color="#0f4c81")
    ax2.annotate("$\\lambda$=1：平滑（方差小）\n但 $P$ 已收敛，400 s 仍停在 ≈0.074（偏差大）",
                 xy=(393, 0.0735), xytext=(215, 0.015),
                 arrowprops=dict(arrowstyle="->", color="#888888"), fontsize=9.5, color="#555555")
    ax2.set_xlabel("时间 $t$ (s)"); ax2.set_ylabel("$\\hat{K}$  ((deg/s)/deg)")
    ax2.set_title("阶跃跟踪 Monte-Carlo（12 次，均值 ±1σ）：$\\lambda$ 的偏差-方差权衡", fontsize=10.5)
    ax2.grid(alpha=0.3); ax2.legend(fontsize=8.5, loc="upper left")
    ax2.set_xlim(0, 400); ax2.set_ylim(0, 0.12)

    fig.text(0.5, 0.012,
             "工程取值 $\\lambda$=0.98（nomoto_identifier.hpp:18）：记忆窗 ≈50 拍 ≈ 50 s，兼顾 $K$/$T$ 慢时变跟踪与噪声抑制。"
             "仿真：合成示例船（$T$=5 s）+ Z 形舵，RLS 语义与代码一致（含门控与 P 保底）。",
             ha="center", fontsize=9, color="#666666")
    fig.subplots_adjust(left=0.07, right=0.97, top=0.86, bottom=0.13, wspace=0.25)
    fig.savefig(f"{OUT}/rls_math_forgetting.png", dpi=DPI)
    plt.close(fig)
    print(f"fig3 t_track={t_track:.0f}s, K̂98 final={m98[-1]:.4f}, K̂10 final={m10[-1]:.4f}")

# ============================================================
# 图 4 rls_math_pe.png —— 持续激励与门控
# ============================================================
def fig4():
    dt = 0.02
    fig = plt.figure(figsize=(14.5, 5.6))
    fig.suptitle("持续激励（PE）与激励门控：什么样的数据才能辨识出 $K$ 和 $T$",
                 fontsize=14, fontweight="bold")

    # (a) 稳态直航段
    n1 = 3000
    rng = np.random.default_rng(5)
    d1 = rng.normal(0, 0.4, n1)                       # 直航：舵角微动
    r1 = np.zeros(n1)
    for k in range(1, n1):
        r1[k] = r1[k-1] + dt*(K_TRUE*d1[k] - r1[k-1])/T_TRUE
    dr1 = (K_TRUE*d1 - r1)/T_TRUE + rng.normal(0, 0.02, n1)
    r1 += rng.normal(0, 0.1, n1)
    t1 = np.arange(n1)*dt

    ax = fig.add_subplot(131)
    ax.plot(t1, d1, color="#4f86b8", lw=0.9, label="$\\delta$ (deg)")
    ax.plot(t1, r1, color="#e8873a", lw=0.9, label="$r$ (deg/s)")
    ax.plot(t1, dr1, color="#57a05a", lw=0.9, label="$\\dot{r}$ (deg/s$^2$)")
    for v, c in [(3.0, "#4f86b8"), (-3.0, "#4f86b8"), (0.5, "#e8873a"), (-0.5, "#e8873a"),
                 (0.2, "#57a05a"), (-0.2, "#57a05a")]:
        ax.axhline(v, color=c, ls="--", lw=1)
    ax.text(30, 4.2, "门控阈值（虚线）：$|\\delta|$>3°，$|r|$>0.5 或 $|\\dot{r}|$>0.2", fontsize=8.5, ha="center")
    ax.text(30, -6.6, "拒绝：本段不更新 $\\theta$", fontsize=13, fontweight="bold", color="#c25b5b",
            ha="center", bbox=dict(fc="white", ec="#c25b5b", boxstyle="round,pad=0.3"))
    ax.set_ylim(-8, 8)
    ax.set_xlabel("$t$ (s)"); ax.set_title("(a) 稳态直航段：$\\delta$≈0、$r$≈0、$\\dot{r}$≈0", fontsize=10.5)
    ax.legend(fontsize=8, loc="upper right"); ax.grid(alpha=0.3)

    # (b) Z 形转向段：δ 用左轴，r 与 ṙ 用右轴（ṙ×5 便于观察）
    n2 = 6000
    rng2 = np.random.default_rng(7)
    d2c, r2c = sim_ship(zigzag(), T_TRUE, K_TRUE, dt, n=n2, noise=0.0)
    dr2 = (K_TRUE*d2c - r2c)/T_TRUE + rng2.normal(0, 0.02, n2)
    d2 = d2c + rng2.normal(0, 0.15, n2)
    r2 = r2c + rng2.normal(0, 0.08, n2)
    t2 = np.arange(n2)*dt

    ax = fig.add_subplot(132)
    axr = ax.twinx()
    ax.plot(t2, d2, color="#4f86b8", lw=0.9, label="$\\delta$ (deg，左轴)")
    axr.plot(t2, r2, color="#e8873a", lw=1.0, label="$r$ (deg/s，右轴)")
    axr.plot(t2, dr2*5, color="#57a05a", lw=1.0, label="$\\dot{r}\\times 5$ (deg/s$^2$，右轴)")
    ax.axhline(3.0, color="#4f86b8", ls="--", lw=1); ax.axhline(-3.0, color="#4f86b8", ls="--", lw=1)
    axr.axhline(0.5, color="#e8873a", ls="--", lw=1); axr.axhline(-0.5, color="#e8873a", ls="--", lw=1)
    axr.axhline(0.2*5, color="#57a05a", ls="--", lw=1); axr.axhline(-0.2*5, color="#57a05a", ls="--", lw=1)
    for tc in np.arange(10, 120, 10):   # δ 每 10 s 换向 → 随后 r 过零、ṙ 峰值
        ax.axvspan(tc, tc+4, color="#57a05a", alpha=0.10)
    axr.text(60, -1.52, "绿带：换向后 $r$ 过零、$\\dot{r}$ 达峰——正是 $T$ 的信息所在", fontsize=8.5, ha="center")
    ax.text(8, -18.6, "接受：更新 $\\theta$ 与 $P$", fontsize=13, fontweight="bold", color="#57a05a",
            ha="left", bbox=dict(fc="white", ec="#57a05a", boxstyle="round,pad=0.3"))
    ax.set_ylim(-22, 22); axr.set_ylim(-1.6, 1.6)
    ax.set_xlabel("$t$ (s)"); ax.set_ylabel("$\\delta$ (deg)", color="#4f86b8")
    axr.set_ylabel("$r$ (deg/s)  /  $\\dot{r}\\times 5$ (deg/s$^2$)", color="#e8873a")
    ax.set_title("(b) Z 形转向段（±15°）：$\\delta$、$r$、$\\dot{r}$ 均充分激励", fontsize=10.5)
    h1, l1 = ax.get_legend_handles_labels(); h2, l2 = axr.get_legend_handles_labels()
    ax.legend(h1+h2, l1+l2, fontsize=7, loc="upper right")
    ax.grid(alpha=0.3)

    # (c) 回归向量空间
    ax = fig.add_subplot(133)
    p1 = np.column_stack([d1/50.0, -dr1])
    m2 = (np.abs(d2) > 3) & ((np.abs(r2) > 0.5) | (np.abs(dr2) > 0.2))
    p2 = np.column_stack([d2[m2]/50.0, -dr2[m2]])
    i1 = np.random.default_rng(0).choice(len(p1), 400, replace=False)
    i2 = np.random.default_rng(0).choice(len(p2), 400, replace=False)
    ax.scatter(p1[i1, 0], p1[i1, 1], s=6, c="#c25b5b", alpha=0.5,
               label="直航段：$\\varphi$ 缩在原点附近（无激励）→ $K$、$T$ 欠定")
    ax.scatter(p2[i2, 0], p2[i2, 1], s=6, c="#4f86b8", alpha=0.4,
               label="Z 形段：$\\varphi$ 在两个方向张开 → $K$、$T$ 均可辨识")
    ax.axhline(0, color="#999999", lw=0.8); ax.axvline(0, color="#999999", lw=0.8)
    ax.set_xlabel("$\\varphi_1=\\delta/50$（归一化）"); ax.set_ylabel("$\\varphi_2=-\\dot{r}$")
    ax.set_title("(c) 回归向量空间：激励 = $\\varphi$ 的方向覆盖", fontsize=10.5)
    ax.legend(fontsize=8.5, loc="lower right"); ax.grid(alpha=0.3)

    fig.text(0.5, 0.012,
             "门控条件（nomoto_identifier.hpp:68-69）：$|\\delta|$>3° 且（$|r|$>0.5°/s 或 $|\\dot{r}|$>0.2°/s²）；"
             "保留 $r$ 过零附近的 $\\dot{r}$ 瞬态，否则 $\\varphi_2$≈0，$T$ 永远辨识不出来。",
             ha="center", fontsize=9, color="#666666")
    fig.subplots_adjust(left=0.06, right=0.94, top=0.87, bottom=0.12, wspace=0.3)
    fig.savefig(f"{OUT}/rls_math_pe.png", dpi=DPI)
    plt.close(fig)

# ============================================================
# 图 5 rls_math_chain.png —— 辨识链路总框图
# ============================================================
def fig5():
    fig, ax = diagram_ax((14.5, 8.2), "RLS 在线辨识链路总框图（数据从传感器到下游调参消费）")

    y1, h1 = 0.70, 0.20
    box(ax, 0.015, y1, 0.115, h1, "传感器",
        "MEMS 陀螺 → $r$ (deg/s)\n舵角反馈 → $\\delta$ (deg)", C["gray"])
    box(ax, 0.145, y1, 0.115, h1, "入口校验",
        "$|\\delta|$≤35°，$|r|$≤30°/s\n有限性检查\n超限丢弃本拍", C["red"])
    box(ax, 0.275, y1, 0.115, h1, "自降频",
        "RLS 拍 1 Hz\nbeatSec = 1.0 s\n$\\Delta t$ 模块自测", C["orange"])
    box(ax, 0.405, y1, 0.115, h1, "差分",
        "$dr=(r_k-r_{k-1})/\\Delta t$\n首拍 $dr$=0", C["orange"])
    box(ax, 0.535, y1, 0.125, h1, "激励门控",
        "$|\\delta|$>3° 且\n($|r|$>0.5 或 $|dr|$>0.2)\n否则丢弃本拍", C["red"])
    box(ax, 0.675, y1, 0.115, h1, "归一化回归向量",
        "$\\varphi=[\\delta/50,\\,-dr]^T$\n$y=r$", C["blue"])
    box(ax, 0.805, y1, 0.180, h1, "RLS 三式（$\\lambda$=0.98）",
        "$L=P\\varphi/(\\lambda+\\varphi^T P\\varphi)$\n$\\theta\\leftarrow\\theta+Le$，$e=y-\\varphi^T\\theta$\n"
        "$P\\leftarrow(P-L\\varphi^T P)/\\lambda$", C["green"])
    for x0, x1 in [(0.130, 0.145), (0.260, 0.275), (0.390, 0.405), (0.520, 0.535), (0.660, 0.675), (0.790, 0.805)]:
        arrow(ax, (x0, y1+h1/2), (x1, y1+h1/2))

    y2, h2 = 0.40, 0.20
    box(ax, 0.805, y2, 0.180, h2, "P 保底",
        "对角元 $P\\geq 10^{-3}$\n防协方差塌陷\n保持长期自适应", C["green"])
    box(ax, 0.535, y2, 0.155, h2, "valid 物理门限",
        "$K\\in(0,\\,2]$\n$T\\in[1,\\,60]$ s\n不合理则快照无效", C["red"])
    box(ax, 0.275, y2, 0.155, h2, "辨识快照（只读）",
        "IdSnapshot\n{excited, valid,\n$\\hat{K}$=$\\theta_0$/50, $\\hat{T}$=$\\theta_1$}", C["purple"])
    arrow(ax, (0.895, y1), (0.895, y2+h2), label="每拍", loffset=(0.032, 0.0))
    arrow(ax, (0.805, y2+h2/2), (0.690, y2+h2/2))
    arrow(ax, (0.535, y2+h2/2), (0.430, y2+h2/2))

    y3, h3 = 0.09, 0.22
    box(ax, 0.02, y3, 0.29, h3, "极点配置建基线（Z 形试验收官）",
        "establishBaseline() 一次性评估\n$K_p=\\omega_n^2 T/K$，$K_d=(2\\zeta\\omega_n T-1)/K$\n过稳定门后建立/重锚基线", C["blue"])
    box(ax, 0.355, y3, 0.29, h3, "稳定门校验",
        "stabilityCheck($\\hat{K}$, $\\hat{T}$, $K_p$, $K_d$, osc)\n$\\zeta\\geq 0.6$，$\\omega_n\\leq 0.5$，振荡 ≤ 5°\n不过门 → 不动基线", C["blue"])
    box(ax, 0.69, y3, 0.29, h3, "漂移门 → 重基线判断（tick 每拍）",
        "$|\\hat{K}-K^*|/K^*$ > 25% 才触发重基线\n容差内交给 ESC 连续调参\n（kReBaselineTol = 0.25）", C["blue"])
    arrow(ax, (0.3525, y2), (0.165, y3+h3), conn="arc3,rad=0.15", label="有效快照", loffset=(-0.045, 0.03))
    arrow(ax, (0.3525, y2), (0.50, y3+h3))
    arrow(ax, (0.3525, y2), (0.835, y3+h3), conn="arc3,rad=-0.15")

    ax.text(0.5, 0.022,
            "代码位置：入口校验 :53-55 / 自降频 :58-61 / 差分 :65-67 / 门控 :68-73 / 归一化 :76 / RLS 三式 :79-103 / P 保底 :105-106 / "
            "valid :119-124（nomoto_identifier.hpp）；IdSnapshot :31 / polePlacement :228 / stabilityCheck :241 / kReBaselineTol :499（auto_tuner.hpp）",
            ha="center", fontsize=8, color="#777777")
    fig.savefig(f"{OUT}/rls_math_chain.png", dpi=DPI)
    plt.close(fig)

if __name__ == "__main__":
    fig1(); fig2(); fig3(); fig4(); fig5()
    print("done")
