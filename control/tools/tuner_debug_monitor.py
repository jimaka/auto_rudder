#!/usr/bin/env python3
# tuner_debug_monitor.py
# 实船调试参数实时查看器：驱动 ar_tuner_debug 进程，按固定周期下发 status 命令，
# 解析 key=value 输出，用 matplotlib 实时绘制调参器内部状态曲线。
#
# 功能：
#   1. 实时曲线：增益（当前/基线）、ESC 梯度、代价 J、振荡幅值与载波相位；
#   2. 命令转发：监控期间在终端输入的任何命令原样转发给 ar_tuner_debug（回显打印），
#      即"一面看曲线、一面做单环节调试"；
#   3. --demo：内置激励序列（建基线 → 喂正弦误差 → 周期 settle），无实船/仿真
#      也能立刻看到各参数动起来，用于验证监控链路本身；
#   4. --log：把每次 status 快照追加写入 CSV，事后可用 plot_sim.py 风格工具复查；
#   5. --duration + --out：无 GUI 环境跑固定时长后保存图片退出（冒烟/归档用）。
#
# 用法：
#   python3 tools/tuner_debug_monitor.py                    # 交互监控（默认找 ../build/tools/ar_tuner_debug）
#   python3 tools/tuner_debug_monitor.py --demo             # 内置激励演示
#   python3 tools/tuner_debug_monitor.py --interval 0.2 --window 120 --log out/dbg.csv
#   PLOT_SIM_AGG=1 python3 tools/tuner_debug_monitor.py --demo --duration 10 --out out/monitor.png
import argparse
import csv
import os
import subprocess
import sys
import threading
import time
from collections import deque

import matplotlib

if os.environ.get("PLOT_SIM_AGG") == "1":
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

# 与 .scratch/esc_math_figs.py 同一套中文字体策略：优先注册仓库内置字体文件
_here = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(_here, "..", "..", ".scratch", "fonts", "NotoSansCJKsc-Regular.otf"),
           os.path.join(_here, "..", "..", ".scratch", "fonts", "NotoSansCJKsc-Bold.otf")):
    if os.path.isfile(_p):
        font_manager.fontManager.addfont(_p)
for cand in ["Noto Sans CJK SC", "WenQuanYi Zen Hei", "SimHei", "Microsoft YaHei"]:
    if any(f.name == cand for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.sans-serif"] = [cand, "DejaVu Sans"]
        break
plt.rcParams["axes.unicode_minus"] = False

# status 输出中被监控的字段（分组到 4 个子图）
FIELDS = ["curKp", "curKd", "baseKp", "baseKd", "gradKp", "gradKd",
          "lastJ", "osc", "escPhase", "winCounter", "demodCount", "refAgeSec",
          "established", "enabled"]


class DebugLink:
    """ar_tuner_debug 进程的请求-响应通道（每条命令恰好一行响应，线程安全）。"""

    def __init__(self, exe):
        if not os.path.isfile(exe):
            sys.exit(f"找不到 ar_tuner_debug：{exe}\n先执行：cd control/build && cmake .. && make ar_tuner_debug")
        self.proc = subprocess.Popen([exe], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, text=True, bufsize=1)
        self.lock = threading.Lock()

    def cmd(self, line):
        with self.lock:
            try:
                self.proc.stdin.write(line + "\n")
                self.proc.stdin.flush()
                return self.proc.stdout.readline().strip()
            except (BrokenPipeError, ValueError):
                return "ERR 通道已断开"

    def status(self):
        resp = self.cmd("status")
        if not resp.startswith("OK"):
            return None
        out = {}
        for kv in resp.split()[1:]:
            if "=" in kv:
                k, v = kv.split("=", 1)
                try:
                    out[k] = float(v)
                except ValueError:
                    pass
        return out

    def close(self):
        try:
            self.cmd("quit")
            self.proc.wait(timeout=2)
        except Exception:
            self.proc.kill()


class Monitor:
    def __init__(self, args):
        self.args = args
        self.link = DebugLink(args.exe)
        self.t0 = time.time()
        self.hist = {f: deque() for f in FIELDS}
        self.hist["t"] = deque()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.log_fp = open(args.log, "a", newline="") if args.log else None
        if self.log_fp and os.path.getsize(args.log) == 0:
            csv.writer(self.log_fp).writerow(["t"] + FIELDS)

    # ---------- 后台线程 ----------
    def poll_loop(self):
        while not self.stop.is_set():
            s = self.link.status()
            if s:
                t = time.time() - self.t0
                with self.lock:
                    self.hist["t"].append(t)
                    for f in FIELDS:
                        self.hist[f].append(s.get(f, 0.0))
                if self.log_fp:
                    csv.writer(self.log_fp).writerow(
                        [f"{t:.3f}"] + [s.get(f, 0.0) for f in FIELDS])
                    self.log_fp.flush()
            self.stop.wait(self.args.interval)

    def console_loop(self):
        """终端输入原样转发（status 之外的响应打印回显）。"""
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            if line in ("quit", "exit"):
                self.stop.set()
                break
            resp = self.link.cmd(line)
            print(f"  <- {resp}")

    def demo_loop(self):
        """内置激励：建基线 → 喂 ±2° 正弦误差 → 每 6 s 一次 settle。"""
        time.sleep(0.3)
        print("[demo] baseline 0.2 8 ->", self.link.cmd("baseline 0.2 8"))
        print("[demo] forceenabled 1 ->", self.link.cmd("forceenabled 1"))
        i = 0
        while not self.stop.is_set():
            t = time.time() - self.t0
            e = 2.0 * (1.0 if (i // 25) % 2 == 0 else -1.0)  # 方波误差，周期 5 s
            self.link.cmd(f"accum {e:.3f} {0.5 * e:.3f} 1")
            self.link.cmd("dither 1.6 8.6")
            if i > 0 and i % 60 == 0:
                print("[demo] settle ->", self.link.cmd("settle 1 0.2 8"))
            i += 1
            self.stop.wait(0.1)

    # ---------- 绘图 ----------
    def snapshot(self):
        with self.lock:
            return {k: list(v) for k, v in self.hist.items()}

    def build_fig(self):
        fig, axes = plt.subplots(2, 2, figsize=(12, 7))
        fig.suptitle("ESC 调参器调试参数实时监控（ar_tuner_debug status）")
        return fig, axes

    def draw(self, axes):
        h = self.snapshot()
        t = h["t"]
        if not t:
            return
        w = self.args.window
        m = [i for i, x in enumerate(t) if x >= t[-1] - w]
        tt = [t[i] for i in m]

        def g(f):
            return [h[f][i] for i in m]

        ax = axes[0][0]
        ax.clear()
        ax.plot(tt, g("curKp"), "b-", label="curKp")
        ax.plot(tt, g("curKd"), "r-", label="curKd")
        ax.plot(tt, g("baseKp"), "b--", alpha=0.5, label="baseKp")
        ax.plot(tt, g("baseKd"), "r--", alpha=0.5, label="baseKd")
        ax.set_title("增益（实线=当前，虚线=基线）")
        ax.legend(fontsize=8, loc="upper right")
        ax.grid(alpha=0.3)

        ax = axes[0][1]
        ax.clear()
        ax.plot(tt, g("gradKp"), "b-", label="gradKp")
        ax.plot(tt, g("gradKd"), "r-", label="gradKd")
        ax.axhline(0, color="k", lw=0.5)
        ax.set_title("ESC 梯度估计（LPF 后）")
        ax.legend(fontsize=8, loc="upper right")
        ax.grid(alpha=0.3)

        ax = axes[1][0]
        ax.clear()
        ax.plot(tt, g("lastJ"), "g-", label="lastJ（窗末代价）")
        ax.set_title("窗口代价 J")
        ax.legend(fontsize=8, loc="upper right")
        ax.grid(alpha=0.3)

        ax = axes[1][1]
        ax.clear()
        ax.plot(tt, g("osc"), "m-", label="振荡峰-峰 (°)")
        ax.plot(tt, g("escPhase"), "c-", alpha=0.6, label="载波相位 (rad)")
        ax.set_title("实测振荡 / 载波相位")
        ax.legend(fontsize=8, loc="upper right")
        ax.grid(alpha=0.3)
        for a in axes.flat:
            a.set_xlabel("t (s)")

    def run(self):
        threads = [threading.Thread(target=self.poll_loop, daemon=True),
                   threading.Thread(target=self.console_loop, daemon=True)]
        if self.args.demo:
            threads.append(threading.Thread(target=self.demo_loop, daemon=True))
        for th in threads:
            th.start()

        fig, axes = self.build_fig()
        headless = self.args.duration is not None or matplotlib.get_backend() == "Agg"
        if headless:  # 固定时长：结束后一次性绘制保存
            time.sleep(self.args.duration or 10)
            self.stop.set()
            time.sleep(0.2)
            self.draw(axes)
            fig.tight_layout()
            out = self.args.out or "monitor.png"
            fig.savefig(out, dpi=150)
            print(f"已保存 {out}")
        else:         # 交互：FuncAnimation 实时刷新
            from matplotlib.animation import FuncAnimation

            def update(_):
                self.draw(axes)

            # 必须持有引用——否则动画对象被 GC，窗口空白（matplotlib 官方陷阱）
            self._anim = FuncAnimation(fig, update, interval=int(self.args.interval * 1000),
                                       cache_frame_data=False)
            plt.show()
            self.stop.set()
        self.link.close()
        if self.log_fp:
            self.log_fp.close()


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="ar_tuner_debug 调试参数实时查看器")
    ap.add_argument("--exe", default=os.path.join(here, "..", "build", "tools", "ar_tuner_debug"),
                    help="ar_tuner_debug 可执行文件路径")
    ap.add_argument("--interval", type=float, default=0.1, help="status 轮询周期 (s)，默认 0.1")
    ap.add_argument("--window", type=float, default=60.0, help="曲线滚动时间窗 (s)，默认 60")
    ap.add_argument("--demo", action="store_true", help="内置激励序列（无需实船即可看到参数变化）")
    ap.add_argument("--log", help="把 status 快照追加写入 CSV")
    ap.add_argument("--duration", type=float, help="跑固定时长 (s) 后保存退出（无 GUI 冒烟用）")
    ap.add_argument("--out", help="--duration 模式的输出图片路径")
    args = ap.parse_args()
    Monitor(args).run()


if __name__ == "__main__":
    main()
