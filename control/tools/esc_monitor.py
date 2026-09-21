#!/usr/bin/env python3
"""Read-only ESC cost/gradient dashboard for simulation and telemetry CSV.

Offline: python3 control/tools/esc_monitor.py --csv control/out/tune.csv --out control/out/esc.png
Live:    python3 control/tools/esc_monitor.py --csv control/out/debug.csv --follow --window 120
See README.md for telemetry semantics and acquisition commands.
"""

import argparse
import csv
import math
import os
import sys
import time
from collections import deque
from pathlib import Path


# Never reconstruct an ESC gradient from gain differences or substitute zero for
# a missing measurement. In this project gradients are the tuner's LPF outputs.
ALIASES = {
    "t": ("t", "time", "timestamp"),
    "currentJ": ("currentJ", "J"),
    "lastJ": ("lastJ",),
    "Jwindow": ("Jwindow",),
    "gradKp": ("gradKp",),
    "gradKd": ("gradKd",),
    "KpCenter": ("KpCenter", "curKp"),
    "KdCenter": ("KdCenter", "curKd"),
    "Kp": ("Kp",),
    "Kd": ("Kd",),
    "hpfY": ("hpfY",),
    "enabled": ("enabled",),
    "baseline": ("baseline", "established"),
    "settle": ("settle",),
}


class CsvSource:
    """Incremental, bounded CSV reader; follow mode consumes complete lines only.

    The producer must write a single-line header and numeric rows, with elapsed
    seconds in t/time/timestamp. Partial writes are retried on the next poll.
    Rotation (inode change) and observed truncation start a fresh session.
    """

    def __init__(self, path, follow=False, max_points=50000):
        self.path = Path(path)
        self.follow = follow
        self.rows = deque(maxlen=max_points)
        self.fp = None
        self.identity = None
        self.header = None
        self.columns = {}
        self.total = self.skipped = self.invalid = self.restarts = self.rewinds = 0
        self.last_sample_t = None
        self.updated_at = None
        self.waiting = False

    def close(self):
        if self.fp is not None:
            self.fp.close()
            self.fp = None

    def _open(self, identity):
        restarting = self.identity is not None
        self.close()
        self.fp = self.path.open("rb")
        self.identity = identity
        self.header = None
        self.columns = {}
        self.rows.clear()
        self.total = self.skipped = self.invalid = self.rewinds = 0
        self.last_sample_t = self.updated_at = None
        if restarting:
            self.restarts += 1

    def _set_header(self, header):
        header = [name.strip() for name in header]
        if not header or len(set(header)) != len(header) or "" in header:
            raise ValueError("CSV 表头为空或有重复列名")
        self.columns = {key: next((name for name in names if name in header), None)
                        for key, names in ALIASES.items()}
        if not self.columns["t"]:
            raise ValueError("CSV 缺少时间列 t / time / timestamp（单位必须为秒）")
        if not any(self.columns[key] for key in ("currentJ", "lastJ", "Jwindow", "gradKp", "gradKd")):
            raise ValueError("CSV 没有 ESC 代价或梯度列；普通 --esc CSV 不含这些数据，请重新运行 --tune")
        self.header = header

    def _add(self, cells):
        if len(cells) != len(self.header):
            self.skipped += 1
            return
        raw = dict(zip(self.header, cells))
        row = {}
        for key, column in self.columns.items():
            value = raw.get(column, "").strip()
            try:
                number = float(value) if value else math.nan
            except ValueError:
                number = math.nan
            if value and not math.isfinite(number):
                self.invalid += 1
            row[key] = number if math.isfinite(number) else math.nan
        if not math.isfinite(row["t"]):
            self.skipped += 1
            return
        # Simulation appends inferred window-reset events after all sample rows.
        # Their timestamps may go backwards; those are not new sessions.
        if row["settle"] != 1.0:
            if self.last_sample_t is not None and row["t"] < self.last_sample_t:
                self.rewinds += 1
            self.last_sample_t = row["t"]
        self.rows.append(row)
        self.total += 1
        self.updated_at = time.monotonic()

    def poll(self):
        try:
            stat = self.path.stat()
        except FileNotFoundError:
            if not self.follow:
                raise
            self.waiting = True
            return
        self.waiting = False
        identity = (stat.st_dev, stat.st_ino)
        if self.fp is None or identity != self.identity or stat.st_size < self.fp.tell():
            self._open(identity)
        # A budget prevents an actively growing file from starving GUI refreshes.
        budget = 10000 if self.follow else math.inf
        consumed = 0
        while consumed < budget:
            offset = self.fp.tell()
            line = self.fp.readline()
            if not line:
                break
            if self.follow and not line.endswith(b"\n"):
                self.fp.seek(offset)
                break
            consumed += 1
            if not line.strip():
                continue
            try:
                cells = next(csv.reader([line.decode("utf-8-sig")], strict=True))
            except (UnicodeDecodeError, csv.Error) as exc:
                if self.header is None:
                    raise ValueError(f"CSV 表头无法解析：{exc}") from exc
                self.skipped += 1
                continue
            if self.header is None:
                self._set_header(cells)
            else:
                self._add(cells)

    def visible_rows(self, window=0.0):
        rows = sorted(self.rows, key=lambda row: row["t"])
        if rows and window > 0.0:
            start = rows[-1]["t"] - window
            rows = [row for row in rows if row["t"] >= start]
        return rows

    def notices(self):
        notes = []
        if self.header is not None:
            for key in ("currentJ", "gradKp", "gradKd"):
                if not self.columns[key]:
                    notes.append(f"missing {key}")
            if not self.columns["lastJ"] and self.columns["Jwindow"]:
                notes.append("legacy Jwindow: pre-clear approximation, not exact lastJ")
        if self.skipped or self.invalid:
            notes.append(f"skipped rows={self.skipped}, invalid values={self.invalid}")
        if self.rewinds:
            notes.append("sample time rewinds: check for mixed sessions")
        if self.total > len(self.rows):
            notes.append(f"retained {len(self.rows)}/{self.total} rows (--max-points)")
        if self.restarts:
            notes.append(f"file restarts={self.restarts}")
        return notes


class Dashboard:
    def __init__(self, plt, source, window=0.0):
        self.plt, self.source, self.window = plt, source, window
        self.fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
        self.axes = axes.ravel()
        self.fig.subplots_adjust(top=0.87, bottom=0.13, hspace=0.40, wspace=0.20)
        self.title = self.fig.suptitle("ESC cost / gradient monitor", fontsize=16)
        self.status = self.fig.text(0.5, 0.915, "", ha="center", fontsize=10)
        self.notes = self.fig.text(0.5, 0.02, "", ha="center", fontsize=9, color="#7c3c13")

    @staticmethod
    def _line(ax, rows, key, label, color, step=False, **kwargs):
        if not any(math.isfinite(row[key]) for row in rows):
            return
        # Keep NaN entries, so gaps are not silently bridged with invented data.
        ax.plot([row["t"] for row in rows], [row[key] for row in rows],
                label=label, color=color, linewidth=1.4,
                drawstyle="steps-post" if step else "default", **kwargs)

    def draw(self):
        rows = self.source.visible_rows(self.window)
        # Event rows in historical tune.csv contain PRE-update centers and
        # POST-update centers in Kp/Kd, unlike normal rows. Never join them into
        # actual-gain curves. New ESC telemetry in event rows is post-tick.
        samples = [row for row in rows if row["settle"] != 1.0]
        for ax in self.axes:
            ax.clear()
        current, completed, gkp, gkd, kp, kd = self.axes
        self._line(current, rows, "currentJ", "currentJ (unfinished window)", "#167c80")
        self._line(completed, rows, "lastJ", "lastJ (held register)", "#dd7a19", step=True)
        if not self.source.columns.get("lastJ"):
            events = [row for row in rows if row["settle"] == 1.0]
            self._line(completed, events, "Jwindow", "Jwindow (pre-clear estimate)",
                       "#dd7a19", marker="o", linestyle="none", markersize=3)
        self._line(gkp, rows, "gradKp", "gradKp (demod + LPF)", "#3169b3", step=True, marker=".")
        self._line(gkd, rows, "gradKd", "gradKd (demod + LPF)", "#af4385", step=True, marker=".")
        for ax, key, color in ((kp, "Kp", "#3169b3"), (kd, "Kd", "#af4385")):
            self._line(ax, samples, key + "Center", key + " center", color, step=True)
            self._line(ax, samples, key, key + " applied (with dither)", "#78878b", alpha=0.7)
        titles = ("Current window cost", "Last settled cost / legacy estimate",
                  "Kp gradient estimate", "Kd gradient estimate", "Kp gain", "Kd gain")
        for ax, title in zip(self.axes, titles):
            ax.set_title(title, fontsize=11, loc="left")
            ax.grid(True, alpha=0.25)
            ax.ticklabel_format(axis="y", style="sci", scilimits=(-3, 4), useOffset=False)
            if ax.lines:
                ax.legend(loc="best", fontsize=8)
            else:
                ax.text(0.5, 0.5, "No valid telemetry for this panel", transform=ax.transAxes,
                        ha="center", va="center", color="#777777")
        for ax in (gkp, gkd):
            ax.axhline(0, color="#444444", linewidth=0.7, linestyle="--")
        for ax in (current, completed):
            ax.set_ylabel("J (raw project cost)")
        for ax in (gkp, gkd):
            ax.set_ylabel("Estimated gradient")
        for ax in (kp, kd):
            ax.set_xlabel("Time (s)")
        if rows:
            lo, hi = rows[0]["t"], rows[-1]["t"]
            current.set_xlim(lo if hi > lo else lo - 0.5, hi if hi > lo else hi + 0.5)
            latest = samples[-1] if samples else rows[-1]
            values = "  |  ".join(f"{key}={latest[key]:.4g}" if math.isfinite(latest[key])
                                   else f"{key}=N/A" for key in ("enabled", "baseline", "hpfY"))
            status = f"t={hi:.2f}s  |  {len(rows)} visible rows  |  {values}"
        else:
            status = "Waiting for complete telemetry rows..."
        if self.source.follow:
            age = (time.monotonic() - self.source.updated_at
                   if self.source.updated_at is not None else None)
            status += f"\nNo new valid rows for {age:.1f}s" if age is not None else "\nWaiting for input"
            if self.source.waiting:
                status += " (file not present)"
        self.status.set_text(status)
        notes = self.source.notices()
        notes.append("Read-only. Zero cost/gradient can mean initialization or gating, NOT convergence.")
        self.notes.set_text("\n".join(notes))

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.fig.savefig(path, dpi=150, bbox_inches="tight")


def positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("必须是大于零的有限数")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description="ESC 代价/梯度只读监测：CSV 回放、实时跟随和图片导出")
    parser.add_argument("--csv", required=True, type=Path, help="仿真 --tune 或调试遥测 CSV")
    parser.add_argument("--follow", action="store_true", help="持续跟随追加行（等候文件创建，Ctrl+C 退出）")
    parser.add_argument("--window", type=positive_float, default=None, help="只显示最近 N 秒；默认显示全部保留数据")
    parser.add_argument("--interval", type=positive_float, default=0.5, help="实时刷新周期（秒，默认 0.5）")
    parser.add_argument("--max-points", type=int, default=50000, help="最多保留的行数，默认 50000")
    parser.add_argument("--out", type=Path, help="保存图片；指定后使用无 GUI 模式")
    parser.add_argument("--duration", type=positive_float, help="实时跟随运行 N 秒后退出")
    args = parser.parse_args(argv)
    if args.max_points < 2:
        parser.error("--max-points 至少为 2")
    if args.duration and not args.follow:
        parser.error("--duration 仅用于 --follow")
    if args.follow and args.out and not args.duration:
        parser.error("无 GUI 跟随需要同时指定 --duration，避免无限等候")
    same_file = args.out and (args.out.resolve() == args.csv.resolve()
                             or (args.out.exists() and args.csv.exists()
                                 and os.path.samefile(args.out, args.csv)))
    if same_file:
        parser.error("--out 不得覆盖输入 CSV")
    try:
        import matplotlib
        if args.out or os.environ.get("PLOT_SIM_AGG") == "1":
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        parser.error("缺少 matplotlib，请安装 control/tools/requirements.txt 中的依赖")
    if not args.out and str(matplotlib.get_backend()).lower() == "agg":
        parser.error("当前无 GUI，请指定 --out；实时跟随还需 --duration")
    source = CsvSource(args.csv, args.follow, args.max_points)
    dashboard = None
    try:
        source.poll()
        if not args.follow and not source.rows:
            raise ValueError("CSV 中没有有效的数据行")
        dashboard = Dashboard(plt, source, args.window or 0.0)
        dashboard.draw()
        if args.follow:
            deadline = time.monotonic() + args.duration if args.duration else math.inf
            if args.out:
                try:
                    while time.monotonic() < deadline:
                        time.sleep(min(args.interval, max(0.0, deadline - time.monotonic())))
                        source.poll()
                except KeyboardInterrupt:
                    pass
                source.poll()
                dashboard.draw()
            else:
                from matplotlib.animation import FuncAnimation
                refresh_errors = []

                def refresh(_frame):
                    try:
                        source.poll()
                        dashboard.draw()
                    except (OSError, ValueError, csv.Error) as exc:
                        refresh_errors.append(exc)
                        plt.close(dashboard.fig)
                        return
                    if time.monotonic() >= deadline:
                        plt.close(dashboard.fig)

                # Retain the animation; otherwise matplotlib may garbage-collect it.
                dashboard.animation = FuncAnimation(dashboard.fig, refresh,
                                                     interval=args.interval * 1000,
                                                     cache_frame_data=False)
                plt.show()
                if refresh_errors:
                    raise refresh_errors[0]
        elif not args.out:
            plt.show()
        if args.out:
            dashboard.save(args.out)
            print(f"已保存：{args.out}（保留 {len(source.rows)} 行，读取 {source.total} 行）")
        for note in source.notices():
            print(f"提示：{note}", file=sys.stderr)
        if not source.rows:
            print("未收到有效遥测数据。", file=sys.stderr)
            return 1
        return 0
    except (OSError, ValueError, csv.Error) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 0
    finally:
        source.close()
        if dashboard is not None:
            plt.close(dashboard.fig)


if __name__ == "__main__":
    sys.exit(main())
