"""Run: MPLBACKEND=Agg python3 -m unittest discover -s control/tools/tests -v"""

import csv
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

os.environ.setdefault("MPLBACKEND", "Agg")
TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
from esc_monitor import CsvSource, Dashboard  # noqa: E402


class MonitorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "telemetry.csv"

    def source(self, text, **kwargs):
        self.path.write_text(text, encoding="utf-8")
        source = CsvSource(self.path, **kwargs)
        self.addCleanup(source.close)
        source.poll()
        return source

    def test_aliases_and_legitimate_zero(self):
        source = self.source("timestamp,J,lastJ,gradKp,gradKd,curKp,curKd,established\n"
                             "0,0,0,0,-0.00000001,1,2,1\n")
        row = source.rows[0]
        self.assertEqual(row["currentJ"], 0)
        self.assertEqual(row["gradKp"], 0)
        self.assertEqual(row["gradKd"], -1e-8)
        self.assertEqual(row["KpCenter"], 1)
        self.assertEqual(row["baseline"], 1)
        self.assertTrue(math.isnan(row["Kp"]))

    def test_missing_fields_are_nan_not_zero(self):
        source = self.source("t,lastJ\n0,42\n")
        self.assertTrue(math.isnan(source.rows[0]["gradKp"]))
        self.assertIn("missing gradKp", source.notices())

    def test_invalid_numbers_and_malformed_rows(self):
        source = self.source("t,currentJ,gradKp,gradKd\n"
                             "0,1,nan,inf\n1,2,oops,3\nbad,1,2,3\n2,3\n")
        self.assertEqual(len(source.rows), 2)
        self.assertEqual(source.skipped, 2)
        self.assertEqual(source.invalid, 4)
        self.assertTrue(math.isnan(source.rows[0]["gradKp"]))
        self.assertTrue(math.isnan(source.rows[0]["gradKd"]))

    def test_header_validation(self):
        for header in ("currentJ,gradKp\n", "t,t,currentJ\n", "t,error,Kp,Kd\n"):
            with self.subTest(header=header), self.assertRaises(ValueError):
                self.source(header)

    def test_bom_and_last_line_without_newline_offline(self):
        source = self.source("\ufefft,lastJ\n0,1")
        self.assertEqual(source.rows[0]["lastJ"], 1)

    def test_out_of_order_legacy_events_and_zero_cost(self):
        source = self.source("t,Kp,Kd,KpCenter,KdCenter,Jwindow,settle\n"
                             "0,1,2,1,2,0,0\n10,3,4,3,4,0,0\n"
                             "5,99,99,98,98,0,1\n")
        self.assertEqual([row["t"] for row in source.visible_rows()], [0, 5, 10])
        self.assertEqual(source.rewinds, 0)
        import matplotlib.pyplot as plt
        dashboard = Dashboard(plt, source)
        self.addCleanup(plt.close, dashboard.fig)
        dashboard.draw()
        # A zero-J event is valid; sample-row placeholders are not events.
        self.assertEqual(list(dashboard.axes[1].lines[0].get_ydata()), [0])
        # Historical event gains have a different meaning: exclude them.
        for line in dashboard.axes[4].lines:
            self.assertEqual(list(line.get_ydata()), [1, 3])

    def test_window_bounded_memory_and_rewind_warning(self):
        source = self.source("t,lastJ\n0,1\n2,2\n1,3\n3,4\n", max_points=3)
        self.assertEqual(source.total, 4)
        self.assertEqual(len(source.rows), 3)
        self.assertEqual(source.rewinds, 1)
        self.assertEqual([row["t"] for row in source.visible_rows(1)], [2, 3])

    def test_follow_partial_header_and_partial_row(self):
        source = self.source("t,last", follow=True)
        self.assertIsNone(source.header)
        with self.path.open("a") as fp:
            fp.write("J,gradKp\n0,5,")
        source.poll()
        self.assertEqual(len(source.rows), 0)
        with self.path.open("a") as fp:
            fp.write("-0.1\n")
        source.poll()
        source.poll()  # No duplicated rows on an idle poll.
        self.assertEqual(len(source.rows), 1)
        self.assertEqual(source.rows[0]["gradKp"], -0.1)

    def test_follow_waits_for_file_creation(self):
        source = CsvSource(self.path, follow=True)
        self.addCleanup(source.close)
        source.poll()
        self.assertTrue(source.waiting)
        self.path.write_text("t,lastJ\n0,1\n")
        source.poll()
        self.assertFalse(source.waiting)
        self.assertEqual(len(source.rows), 1)

    def test_follow_truncation_and_rotation(self):
        source = self.source("t,lastJ,gradKp\n0,123456,0.456789\n", follow=True)
        self.path.write_text("t,lastJ\n1,2\n")
        source.poll()
        self.assertEqual(source.restarts, 1)
        self.assertEqual([row["t"] for row in source.rows], [1])
        self.path.rename(self.path.with_suffix(".old"))
        self.path.write_text("t,lastJ\n2,3\n")
        source.poll()
        self.assertEqual(source.restarts, 2)
        self.assertEqual([row["t"] for row in source.rows], [2])

    def test_plot_preserves_gaps_and_separates_costs(self):
        source = self.source("t,currentJ,lastJ,gradKp,gradKd\n"
                             "0,1,10,1,2\n1,2,10,,3\n2,0,12,-1,-2\n")
        import matplotlib.pyplot as plt
        dashboard = Dashboard(plt, source)
        self.addCleanup(plt.close, dashboard.fig)
        dashboard.draw()
        self.assertEqual(list(dashboard.axes[0].lines[0].get_ydata()), [1, 2, 0])
        self.assertEqual(list(dashboard.axes[1].lines[0].get_ydata()), [10, 10, 12])
        self.assertTrue(math.isnan(dashboard.axes[2].lines[0].get_ydata()[1]))
        out = Path(self.temp.name) / "plot.png"
        dashboard.save(out)
        self.assertGreater(out.stat().st_size, 10000)

    def run_cli(self, *args):
        return subprocess.run([sys.executable, str(TOOLS / "esc_monitor.py"), *map(str, args)],
                              capture_output=True, text=True, timeout=30)

    def test_headless_cli_single_row(self):
        self.path.write_text("t,currentJ,lastJ,gradKp,gradKd\n0,0,0,0,0\n")
        out = Path(self.temp.name) / "nested" / "esc.png"
        result = self.run_cli("--csv", self.path, "--out", out)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(out.is_file())

    def test_cli_rejects_invalid_arguments_without_modifying_input(self):
        original = "t,lastJ\n0,1\n"
        self.path.write_text(original)
        for args in (("--out", self.path), ("--max-points", "1"), ("--interval", "nan"),
                     ("--duration", "1"), ("--follow", "--out", "x.png")):
            with self.subTest(args=args):
                self.assertEqual(self.run_cli("--csv", self.path, *args).returncode, 2)
        self.assertEqual(self.path.read_text(), original)

    def test_empty_csv_fails_cleanly(self):
        self.path.write_text("t,lastJ\n")
        result = self.run_cli("--csv", self.path, "--out", Path(self.temp.name) / "empty.png")
        self.assertEqual(result.returncode, 1)
        self.assertIn("没有有效", result.stderr)

    def test_output_hardlink_cannot_overwrite_input(self):
        original = "t,lastJ\n0,1\n"
        self.path.write_text(original)
        out = self.path.with_suffix(".png")
        os.link(self.path, out)
        result = self.run_cli("--csv", self.path, "--out", out)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(self.path.read_text(), original)

    def test_follow_without_data_returns_nonzero_and_waiting_image(self):
        out = Path(self.temp.name) / "waiting.png"
        result = self.run_cli("--csv", self.path, "--follow", "--duration", "0.05", "--out", out)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn("未收到", result.stderr)
        self.assertTrue(out.is_file())


class DebugLoggerTests(unittest.TestCase):
    def test_status_aliases(self):
        from tuner_debug_monitor import DebugLink
        link = object.__new__(DebugLink)
        link.cmd = lambda _: "OK phase=1.2 win=34 demodN=56 refAge=7.8 currentJ=9 gradKp=-1e-8"
        status = link.status()
        self.assertEqual(status["escPhase"], 1.2)
        self.assertEqual(status["winCounter"], 34)
        self.assertEqual(status["demodCount"], 56)
        self.assertEqual(status["refAgeSec"], 7.8)
        self.assertEqual(status["currentJ"], 9)
        self.assertEqual(status["gradKp"], -1e-8)

    def test_old_log_header_rejected_before_process_start(self):
        from argparse import Namespace
        from tuner_debug_monitor import Monitor
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "old.csv"
            with path.open("w", newline="") as fp:
                csv.writer(fp).writerow(["t", "lastJ"])
            with self.assertRaisesRegex(SystemExit, "表头不兼容"):
                Monitor(Namespace(log=str(path)))


class TelemetryIntegrationTests(unittest.TestCase):
    """Optional when C++ binaries are absent; run cmake --build control/build first."""

    def executable(self, relative):
        path = TOOLS.parent / "build" / relative
        if not path.is_file():
            self.skipTest(f"先构建 C++ 工具：{path}")
        return str(path)

    def test_debug_cli_exports_actual_cost_and_small_gradient(self):
        exe = self.executable("tools/ar_tuner_debug")
        result = subprocess.run([exe], input="baseline 0.2 8\nforceenabled 1\n"
                                "forcegrad 1e-9 -2e-9\naccum 1 2 1\naccum 1 2 1\nstatus\nj\nquit\n",
                                text=True, capture_output=True, timeout=10, check=True)
        lines = result.stdout.splitlines()
        status = next(line for line in lines if "currentJ=" in line)
        fields = dict(token.split("=", 1) for token in status.split()[1:])
        reported_j = float(next(line.split("=", 1)[1] for line in lines if line.startswith("OK J=")))
        self.assertAlmostEqual(float(fields["currentJ"]), reported_j, places=6)
        self.assertGreater(reported_j, 0)
        self.assertEqual(float(fields["gradKp"]), 1e-9)
        self.assertEqual(float(fields["gradKd"]), -2e-9)

    def test_simulation_exports_real_gradients_without_changing_legacy_columns(self):
        exe = self.executable("sim/ar_sim")
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "tune.csv"
            result = subprocess.run([exe, "--tune", "--seconds", "1200", "--out", str(path)],
                                    capture_output=True, text=True, timeout=90)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            with path.open(newline="") as fp:
                reader = csv.DictReader(fp)
                self.assertEqual(reader.fieldnames[:19],
                                 "index,t,psi,psiRef,rudderCmd,rudderAct,rate,error,Kp,Kd,"
                                 "KpCenter,KdCenter,Khat,That,identValid,baseline,Jwindow,winIndex,settle".split(","))
                rows = list(reader)
            self.assertTrue(any(float(row["gradKp"]) != 0 for row in rows))
            self.assertTrue(any(float(row["gradKd"]) != 0 for row in rows))
            for row in rows:
                for key in ("currentJ", "lastJ", "gradKp", "gradKd", "hpfY"):
                    self.assertTrue(math.isfinite(float(row[key])), (key, row))
            # Clear-event rows contain a post-tick snapshot, not fake zero telemetry.
            events = [row for row in rows if row["settle"] == "1"]
            self.assertGreater(len(events), 4)
            self.assertTrue(any(float(row["lastJ"]) > 0 for row in events))
            source = CsvSource(path)
            try:
                source.poll()
                self.assertEqual(source.total, len(rows))
                self.assertEqual(source.invalid, 0)
                self.assertEqual(source.skipped, 0)
            finally:
                source.close()


if __name__ == "__main__":
    unittest.main()
