#!/usr/bin/env python3
"""animate_sim.py — 自动舵闭环仿真动画生成工具

读取 ar_sim 导出的 CSV，生成时间演化的动画视频（MP4/GIF）。
左半部分为多子图（航向/参考、舵角、转首率、误差、增益），
右半部分为机动极坐标轨迹（可选）。

用法：
    python3 animate_sim.py out/step.csv -o out/step.mp4
    python3 animate_sim.py out/will.csv -o out/will.mp4 --polar
    python3 animate_sim.py out/step.csv -o out/step.gif          # 无 ffmpeg 时
    python3 animate_sim.py out/step.csv -o out/step.mp4 --fps 25 --speed 2.0

依赖：matplotlib、numpy；视频编码需 ffmpeg（apt install ffmpeg）。
"""
import argparse, csv, os, sys
from pathlib import Path
import numpy as np
import matplotlib
if os.environ.get("ANIM_SIM_AGG"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter
from matplotlib.gridspec import GridSpec

COLUMNS = ["index", "t", "heading", "headingRef", "rudderCmd", "rudderAct",
           "rate", "error", "Kp", "Kd", "mode"]

# 复用 plot_sim.py 的 CJK 字体配置
for _cjk in ["Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Zen Hei",
             "WenQuanYi Micro Hei", "AR PL UMing CN", "AR PL UKai CN"]:
    try:
        matplotlib.font_manager.findfont(_cjk, fallback_to_default=False)
        matplotlib.rcParams["font.sans-serif"] = [_cjk] + matplotlib.rcParams["font.sans-serif"]
        break
    except Exception:
        continue
matplotlib.rcParams["axes.unicode_minus"] = False


def load_csv(path):
    data = {c: [] for c in COLUMNS}
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise ValueError(f"{path}: 空 CSV")
        header = [h.strip() for h in header]
        idx_map = {h: i for i, h in enumerate(header)}
        for row in reader:
            if not row or len(row) < len(COLUMNS):
                continue
            for c in COLUMNS:
                if c not in idx_map:
                    continue
                v = row[idx_map[c]]
                try:
                    data[c].append(int(float(v)) if c == "mode" else float(v))
                except ValueError:
                    data[c].append(0.0)
    return {c: np.array(data[c]) for c in COLUMNS}


def pick_writer(out_path):
    if out_path.lower().endswith(".gif"):
        return PillowWriter(fps=20)
    return FFMpegWriter(fps=30)


def animate(data, out_path, fps=30, speed=1.0, polar=False, title=""):
    t = data["t"]
    n = len(t)
    # 下采样：按目标 fps 控制总帧数（避免视频过长）
    total_dur = t[-1] - t[0] if n > 1 else 0.0
    target_frames = max(30, min(int(fps * total_dur / max(speed, 0.01)), 1200))
    stride = max(1, n // target_frames)
    idx = np.arange(0, n, stride)
    t = t[idx]; n = len(idx)

    if polar:
        fig = plt.figure(figsize=(14, 7))
        gs = GridSpec(5, 2, figure=fig, hspace=0.4, wspace=0.25,
                      left=0.06, right=0.97, top=0.93, bottom=0.07,
                      width_ratios=[1.4, 1.0])
        ax_polar = fig.add_subplot(gs[:, 1], projection="polar")
    else:
        fig = plt.figure(figsize=(10, 10))
        gs = GridSpec(5, 1, figure=fig, hspace=0.35, left=0.08, right=0.97,
                      top=0.93, bottom=0.07)

    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    ax3 = fig.add_subplot(gs[2], sharex=ax1)
    ax4 = fig.add_subplot(gs[3], sharex=ax1)
    ax5 = fig.add_subplot(gs[4], sharex=ax1)
    ax5b = ax5.twinx()

    # 预设坐标范围
    psi_max = max(np.max(np.abs(data["heading"])), np.max(np.abs(data["headingRef"])), 1.0) * 1.1
    ax1.set_ylim(-psi_max, psi_max); ax1.set_ylabel("航向 (°)")
    ax1.set_title(title or "自动舵闭环仿真动画", fontsize=11)
    ax1.grid(alpha=0.3)
    line_psi, = ax1.plot([], [], "C0", lw=1.5, label="航向 ψ")
    line_ref, = ax1.plot([], [], "k--", lw=1.0, label="参考 ψ_ref")
    head_dot, = ax1.plot([], [], "o", color="C0", ms=8)
    ax1.legend(loc="upper right", fontsize=8)

    r_max = max(np.max(np.abs(data["rudderCmd"])), np.max(np.abs(data["rudderAct"])), 35.0) * 1.1
    ax2.set_ylim(-r_max, r_max); ax2.set_ylabel("舵角 (°)")
    ax2.axhline(35, color="r", ls=":", lw=0.8); ax2.axhline(-35, color="r", ls=":", lw=0.8)
    ax2.grid(alpha=0.3)
    line_cmd, = ax2.plot([], [], "C1", lw=1.5, label="δ_cmd")
    line_act, = ax2.plot([], [], "C7", lw=1.0, alpha=0.7, label="δ_act")
    ax2.legend(loc="upper right", fontsize=8)

    rate_max = max(np.max(np.abs(data["rate"])), 0.5) * 1.1
    ax3.set_ylim(-rate_max, rate_max); ax3.set_ylabel("转首率 (°/s)")
    ax3.grid(alpha=0.3)
    line_rate, = ax3.plot([], [], "C2", lw=1.2)

    err_max = max(np.max(np.abs(data["error"])), 1.0) * 1.1
    ax4.set_ylim(-err_max, err_max); ax4.set_ylabel("误差 e (°)")
    ax4.axhline(0, color="k", lw=0.5); ax4.grid(alpha=0.3)
    line_err, = ax4.plot([], [], "C3", lw=1.2)

    kp_max = max(np.max(data["Kp"]), 1.0) * 1.1
    kd_max = max(np.max(data["Kd"]), 1.0) * 1.1
    ax5.set_ylim(0, kp_max); ax5b.set_ylim(0, kd_max)
    ax5.set_ylabel("Kp", color="C4"); ax5b.set_ylabel("Kd", color="C5")
    ax5.tick_params(axis="y", labelcolor="C4"); ax5b.tick_params(axis="y", labelcolor="C5")
    ax5.set_xlabel("时间 (s)"); ax5.grid(alpha=0.3)
    line_kp, = ax5.plot([], [], "C4", lw=1.2)
    line_kd, = ax5b.plot([], [], "C5", lw=1.2, ls=":")

    ax1.set_xlim(t[0], t[-1])

    # 极坐标
    if polar:
        ax_polar.set_theta_zero_location("N"); ax_polar.set_theta_direction(-1)
        ax_polar.set_title("机动轨迹（θ=航向, r=累计转首角）", pad=15)
        polar_line = ax_polar.plot([], [], "C0", lw=1.5)[0]
        polar_dot = ax_polar.plot([], [], "o", color="r", ms=8)[0]
        # 预设半径范围
        dpsi_all = np.abs(np.diff(data["heading"], prepend=data["heading"][0]))
        r_all = np.cumsum(dpsi_all)[idx]
        ax_polar.set_ylim(0, max(r_all[-1], 1.0) * 1.05)

    time_text = ax1.text(0.02, 0.92, "", transform=ax1.transAxes, fontsize=10,
                        bbox=dict(facecolor="white", alpha=0.7, edgecolor="none"))

    def init():
        for ln in (line_psi, line_ref, line_cmd, line_act, line_rate, line_err, line_kp, line_kd):
            ln.set_data([], [])
        head_dot.set_data([], [])
        if polar:
            polar_line.set_data([], [])
            polar_dot.set_data([], [])
        time_text.set_text("")
        return []

    def update(frame):
        i = frame + 1
        if i > n:
            i = n
        xs = t[:i]
        line_psi.set_data(xs, data["heading"][idx[:i]])
        line_ref.set_data(xs, data["headingRef"][idx[:i]])
        head_dot.set_data([t[i-1]], [data["heading"][idx[i-1]]])
        line_cmd.set_data(xs, data["rudderCmd"][idx[:i]])
        line_act.set_data(xs, data["rudderAct"][idx[:i]])
        line_rate.set_data(xs, data["rate"][idx[:i]])
        line_err.set_data(xs, data["error"][idx[:i]])
        line_kp.set_data(xs, data["Kp"][idx[:i]])
        line_kd.set_data(xs, data["Kd"][idx[:i]])
        if polar:
            psi = np.deg2rad(data["heading"][idx[:i]])
            dpsi = np.abs(np.diff(data["heading"][idx[:i]], prepend=data["heading"][idx[0]]))
            r = np.cumsum(dpsi)
            polar_line.set_data(psi, r)
            polar_dot.set_data([psi[-1]], [r[-1]])
        time_text.set_text(f"t = {t[i-1]:.2f} s")
        return []

    anim = FuncAnimation(fig, update, frames=n, init_func=init,
                         interval=1000.0 / fps, blit=False, repeat=False)
    writer = pick_writer(out_path)
    anim.save(out_path, writer=writer, dpi=110)
    plt.close(fig)
    return out_path


def main():
    ap = argparse.ArgumentParser(description="自动舵闭环仿真动画生成")
    ap.add_argument("csv", help="ar_sim 导出的 CSV 路径")
    ap.add_argument("-o", "--out", required=True, help="输出视频路径（.mp4 或 .gif）")
    ap.add_argument("--fps", type=int, default=30, help="帧率（默认 30）")
    ap.add_argument("--speed", type=float, default=1.0, help="回放速度倍数（>1 加速）")
    ap.add_argument("--polar", action="store_true", help="右侧追加机动极坐标轨迹")
    ap.add_argument("--title", default="", help="视频标题")
    args = ap.parse_args()

    if not Path(args.csv).exists():
        sys.exit(f"CSV 不存在: {args.csv}")
    data = load_csv(args.csv)
    if len(data["t"]) < 2:
        sys.exit("CSV 采样点不足")

    print(f"生成动画: {args.csv} → {args.out} (fps={args.fps}, speed={args.speed}x, polar={args.polar})")
    animate(data, args.out, fps=args.fps, speed=args.speed,
            polar=args.polar, title=args.title)
    print(f"视频已保存: {args.out}")


if __name__ == "__main__":
    main()
