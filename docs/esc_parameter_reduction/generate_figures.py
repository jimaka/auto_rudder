#!/usr/bin/env python3
"""Generate analytical illustrations, not simulation results, for the ESC proposal.

Run from the repository root:
    MPLBACKEND=Agg python3 docs/esc_parameter_reduction/generate_figures.py
Requires numpy, matplotlib and a Chinese font (Noto Sans CJK or equivalent).
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
import numpy as np


OUT = Path(__file__).resolve().parent / "figures"
COLORS = {
    "ink": "#18324B", "blue": "#2167B2", "teal": "#008577",
    "amber": "#C97D18", "red": "#BF4654", "muted": "#64788B",
    "pale": "#EFF5FA", "mint": "#E8F5EF", "sand": "#FFF4DE",
}


def configure():
    candidates = ("Noto Sans CJK SC", "Noto Sans CJK JP", "Source Han Sans SC",
                  "Microsoft YaHei", "WenQuanYi Zen Hei")
    available = {font.name for font in font_manager.fontManager.ttflist}
    chinese_font = next((name for name in candidates if name in available), None)
    if chinese_font is None:
        raise RuntimeError("A Chinese font is required to render these diagrams.")
    plt.rcParams.update({
        "font.family": [chinese_font, "DejaVu Sans"],
        "font.size": 11, "axes.titlesize": 13, "axes.labelsize": 11,
        "text.color": COLORS["ink"], "axes.labelcolor": COLORS["ink"],
        "xtick.color": COLORS["muted"], "ytick.color": COLORS["muted"],
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.unicode_minus": False, "figure.facecolor": "white",
        "savefig.facecolor": "white", "svg.fonttype": "path",
    })
    OUT.mkdir(parents=True, exist_ok=True)


def save(fig, name):
    for suffix in ("png", "svg"):
        fig.savefig(OUT / f"{name}.{suffix}", dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"Generated {name}.png and {name}.svg")


def box(ax, x, y, width, height, title, body, fill="pale", edge="blue"):
    ax.add_patch(FancyBboxPatch(
        (x, y), width, height, boxstyle="round,pad=0.025,rounding_size=0.12",
        linewidth=1.5, edgecolor=COLORS[edge], facecolor=COLORS[fill], zorder=2,
    ))
    ax.text(x + width / 2, y + height * .73, title, ha="center", va="center",
            fontsize=13, weight="bold", zorder=3)
    ax.text(x + width / 2, y + height * .32, body, ha="center", va="center",
            fontsize=10.5, linespacing=1.6, zorder=3)


def arrow(ax, start, end, color="muted"):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=15,
                               linewidth=1.6, color=COLORS[color], zorder=1))


def architecture():
    fig, ax = plt.subplots(figsize=(15, 8))
    ax.set(xlim=(0, 16), ylim=(0, 9))
    ax.axis("off")
    ax.text(.2, 8.65, "ESC 减参：把操作意图转换为成套算法参数", fontsize=22, weight="bold")
    ax.text(.2, 8.17, "设计架构 · 3 项现场设置 + 船型配置 + 独立安全约束；尚未实现", color=COLORS["muted"])
    box(ax, .2, 6.1, 4.55, 1.6, "现场设置：3 项", "学习速度 · 带宽探索幅度\n舵机活动权重", "mint", "teal")
    box(ax, 5.7, 6.1, 4.55, 1.6, "已验证船型配置", "阻尼目标 · 时间比例 · 归一化尺度\n噪声门限 · 独立安全边界")
    box(ax, 11.2, 6.1, 4.55, 1.6, "模型与实际时钟", "辨识基线 K̂ / T̂ · 有效性\n实际 Δt · 工况与传感器状态")
    box(ax, 4.35, 3.95, 7.25, 1.35, "参数推导与准入检查", "成套生成 P / W / τH / τL / η / ρ；窗口内冻结配置")
    for x in (2.475, 7.975, 13.475):
        arrow(ax, (x, 6.05), (7.975, 5.33))
    nodes = [
        (.2, "单变量 ESC", "仅搜索 q = ln(ωn / ωn,b)", "pale", "blue"),
        (4.2, "极点配置", "固定 ζ*，计算 Kp / Kd", "pale", "blue"),
        (8.2, "最终增益安全门", "检查中心与完整扰动包络", "sand", "amber"),
        (12.2, "PD → 舵机 → 船舶", "执行控制，采集航向与舵角", "mint", "teal"),
    ]
    for x, title, body, fill, edge in nodes:
        box(ax, x, 1.55, 3.55, 1.45, title, body, fill, edge)
    arrow(ax, (5.5, 3.92), (1.975, 3.04))
    arrow(ax, (10.0, 3.92), (9.975, 3.04), "amber")
    for x in (3.77, 7.77, 11.77):
        arrow(ax, (x, 2.27), (x + .40, 2.27))
    ax.plot([13.97, 13.97, 1.975, 1.975], [1.5, .69, .69, 1.18],
            color=COLORS["teal"], linewidth=1.6)
    arrow(ax, (1.975, 1.18), (1.975, 1.53), "teal")
    ax.text(8.0, .76, "反馈：无量纲代价 → 高通 → 按观测时刻解调 → 梯度平滑",
            ha="center", va="bottom", color=COLORS["teal"],
            bbox={"facecolor": "white", "edgecolor": "none", "pad": 4})
    ax.text(.2, .05, "准入失败：停止探索并按故障策略保持、回退或退出自动模式；安全约束不作为现场性能旋钮。",
            fontsize=10, color=COLORS["muted"])
    save(fig, "01_architecture")


def timing():
    fig = plt.figure(figsize=(14, 7.5))
    grid = fig.add_gridspec(2, 2, width_ratios=[1.2, 1], hspace=.55, wspace=.32)
    axes = [fig.add_subplot(grid[0, 0]), fig.add_subplot(grid[1, 0])]
    t = np.linspace(0, 145, 1200)
    examples = [(.05, 60., COLORS["red"], "当前默认：60 s 窗口只覆盖 0.477 个周期"),
                (2 * np.pi / 120, 120., COLORS["teal"], "成套示例：120 s 窗口覆盖 1 个周期")]
    for ax, (omega, window, color, title) in zip(axes, examples):
        ax.axvspan(0, window, color=color, alpha=.09, label="评估窗口")
        ax.plot(t, np.sin(omega * t), color=color, linewidth=2, label="正弦载波")
        ax.axvline(window, color=color, linestyle="--", linewidth=1.3)
        ax.axhline(0, color=COLORS["muted"], linewidth=.6)
        ax.set(xlim=(0, 145), ylim=(-1.35, 1.35), xlabel="时间 t / s", ylabel="载波（单位幅值）",
               title=title)
        ax.grid(alpha=.15)
    ax = fig.add_subplot(grid[:, 1])
    x = np.geomspace(.04, 30, 500)
    ax.semilogx(x, x*x/(1+x*x), color=COLORS["blue"], linewidth=2.5)
    for xx, label, color, position in [
        (.25, "默认：ΩτH = 0.25\n同相系数 ≈ 0.059", "red", (.11, .29)),
        (np.pi, "示例：ΩτH = π\n同相系数 ≈ 0.908", "teal", (1.2, .63)),
    ]:
        yy = xx*xx/(1+xx*xx)
        ax.scatter([xx], [yy], color=COLORS[color], s=65, zorder=5)
        ax.annotate(label, xy=(xx, yy), xytext=position, fontsize=11,
                    color=COLORS[color], arrowprops={"arrowstyle": "->", "color": COLORS[color]})
    ax.set(xlabel=r"无量纲频率 $\Omega\tau_H$", ylabel=r"$\operatorname{Re}\{H(j\Omega)\}$",
           title="高通对同相解调的影响（连续时间近似）", ylim=(0, 1.08))
    ax.grid(alpha=.2, which="both")
    fig.suptitle("时间参数应成套设计，不能分别凭经验调整", fontsize=21, weight="bold", y=1.01)
    fig.text(.08, -.02, "解析示意，不是闭环仿真。右图仅含一阶高通，不含船舶相位、噪声与舵机动态；0.908 不代表闭环效果提高 15 倍。",
             color=COLORS["muted"], fontsize=10)
    save(fig, "02_timing_and_hpf")


def search_path():
    model_k, model_t, zeta = .2, 8., .85
    wn = np.linspace(.10, .40, 301)
    kp = wn**2 * model_t / model_k
    kd = (2*zeta*wn*model_t - 1) / model_k
    assert np.isclose(.2**2 * model_t / model_k, 1.6)
    assert np.isclose((2*zeta*.2*model_t - 1)/model_k, 8.6)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={"wspace": .32})
    axes[0].plot(kp, kd, color=COLORS["blue"], linewidth=3)
    for w, offset in [(.16, (12, -23)), (.20, (12, -1)), (.25, (12, -3))]:
        p, d = w*w*model_t/model_k, (2*zeta*w*model_t-1)/model_k
        axes[0].scatter([p], [d], s=65, color=COLORS["teal"], zorder=5)
        axes[0].annotate(f"ωn={w:.2f}\nKp={p:g}, Kd={d:g}", (p, d), xytext=offset,
                         textcoords="offset points", fontsize=10)
    axes[0].set(xlabel="Kp", ylabel="Kd / s", title="二维增益平面 → 一条固定阻尼曲线")
    axes[1].plot(wn, kp/1.6, color=COLORS["blue"], linewidth=2.4, label="Kp / 基线 Kp")
    axes[1].plot(wn, kd/8.6, color=COLORS["teal"], linewidth=2.4, label="Kd / 基线 Kd")
    axes[1].axvline(.2, color=COLORS["muted"], linewidth=1, linestyle="--")
    axes[1].axhline(1, color=COLORS["muted"], linewidth=1, linestyle="--")
    axes[1].set(xlabel="自然频率 ωn / (rad/s)", ylabel="增益相对基线的倍数",
                title="一个搜索变量，同时决定两个 PD 增益")
    axes[1].legend(frameon=False)
    for ax in axes:
        ax.grid(alpha=.2)
    fig.suptitle("降维：ESC 搜索响应快慢，极点配置维持名义阻尼", fontsize=21, weight="bold", y=1.03)
    fig.text(.07, -.03, "解析示例：K̂=0.2 s⁻¹，T̂=8 s，ζ*=0.85。曲线不是已验证安全域，也不表示全局最优；真实响应仍受模型误差与非线性影响。",
             fontsize=10, color=COLORS["muted"])
    save(fig, "03_single_variable_path")


def envelope():
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.7), sharey=True,
                             gridspec_kw={"wspace": .18})
    phase = np.linspace(0, 2*np.pi, 401)
    low, high, amplitude = -.30, .30, .08
    for ax, center, title, color in zip(
        axes, [.27, .18], ["只检查中心：扰动峰值会越界", "先收缩中心区间：为整条载波留余量"],
        ["red", "teal"],
    ):
        ax.axhspan(low+amplitude, high-amplitude, color=COLORS["teal"], alpha=.07,
                   label="可用中心区间")
        for bound in (low, high):
            ax.axhline(bound, color=COLORS["red"], linestyle="--", linewidth=1.2)
        for bound in (low+amplitude, high-amplitude):
            ax.axhline(bound, color=COLORS["teal"], linestyle=":", linewidth=1)
        ax.axhline(center, color=COLORS["muted"], linewidth=1, label="增益中心对应的 q")
        ax.plot(phase/(2*np.pi), center+amplitude*np.sin(phase), color=COLORS[color],
                linewidth=2.5, label="实际施加的 q（含扰动）")
        ax.set(xlabel="载波周期进度 t / P", xlim=(0, 1), ylim=(-.36, .41), title=title)
        ax.grid(alpha=.12)
    axes[0].set_ylabel("无量纲搜索坐标 q")
    axes[0].annotate("峰值 0.35 > 上界 0.30", xy=(.25, .35), xytext=(.36, .38),
                     fontsize=10, color=COLORS["red"],
                     arrowprops={"arrowstyle": "->", "color": COLORS["red"]})
    axes[1].legend(loc="center", bbox_to_anchor=(.55, .34), frameon=True,
                   facecolor="white", edgecolor="white", fontsize=10)
    fig.suptitle("安全边界必须覆盖最终施加增益，而不只是更新中心", fontsize=21, weight="bold", y=1.04)
    fig.text(.07, -.04, "边界算术示意：q ∈ [−0.30, 0.30]、a=0.08 → 中心只能在 [−0.22, 0.22]；实际项目还需独立模型与执行器安全检查。",
             fontsize=10, color=COLORS["muted"])
    save(fig, "04_dither_envelope")


def main():
    configure()
    architecture()
    timing()
    search_path()
    envelope()


if __name__ == "__main__":
    main()
