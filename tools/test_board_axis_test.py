"""Offline grading/protocol tests; no serial port or bench needed."""
import io
import contextlib
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch
from board_axis_test import (BenchFault, Console, fields, parser, point_values, run,
                            summarise_run, summarise_step)


def row(phase, attempt, error=0, state="1", status="ok"):
    return (f"LOOP-TEST-ROW phase={phase} attempt={attempt} LOOP it={attempt} st={status} "
            f"doff_mc={error} tone_ppm={error} skew_cps={error} "
            f"corr_a_ppm={state} corr_b_ppm={state} offa_mc=2 offb_mc=2")


def run_lines():
    return ([row("pre", i) for i in range(4)] + [row("train", i) for i in range(4)] +
            [row("hold", i) for i in range(4)] +
            [f"LOOP-TEST-SUMMARY phase={phase} code=24" for phase in ("pre", "train", "hold")] +
            ["LOOP-TEST-END result=OK"])


class FakePort:
    def __init__(self, output):
        self.output = output.encode()
        self.writes = []

    @property
    def in_waiting(self):
        return min(7, len(self.output))

    def write(self, data):
        self.writes.append(data)

    def flush(self):
        pass

    def read(self, count):
        chunk, self.output = self.output[:count], self.output[count:]
        return chunk


class AxisTests(unittest.TestCase):
    def test_independent_holdout_required(self):
        lines = run_lines()
        self.assertTrue(summarise_run(lines, "offset", 0.5, 4, window=2)["pass"])
        bad = [line.replace("doff_mc=0", "doff_mc=3000") if "phase=hold" in line else line
               for line in lines]
        result = summarise_run(bad, "offset", 0.5, 4, window=2)
        self.assertFalse(result["pass"])
        self.assertEqual(result["offset_mean"], 3)

    def test_holdout_and_inactive_coefficients_frozen(self):
        lines = run_lines()
        lines[11] = row("hold", 3, state="8")
        result = summarise_run(lines, "offset", 0.5, 4)
        self.assertFalse(result["pass"])
        self.assertFalse(result["hold_frozen"])
        self.assertFalse(result["inactive_frozen"])

    def test_missing_and_rejected_frames_not_success(self):
        for status in ("gain_range", "skew_range", "tone", "clipping"):
            lines = [row("pre", 0), row("train", 0)] + [row("hold", i, status=status) for i in range(4)]
            result = summarise_run(lines + ["LOOP-TEST-END result=OK"], "gain", .001, 4)
            self.assertFalse(result["pass"])
            self.assertIn(status, result["reject_reasons"])
        self.assertFalse(summarise_run([r for r in run_lines() if "phase=hold attempt=3" not in r], "offset", .5, 4)["pass"])

    def test_nan_and_rail_are_not_convergence(self):
        lines = [line.replace("doff_mc=0", "doff_mc=nan") if "phase=hold" in line else line
                 for line in run_lines()]
        self.assertFalse(summarise_run(lines, "offset", .5, 4)["pass"])
        lines = run_lines() + ["LOOP-TEST-ROW phase=train attempt=5 act=at-range-limit"]
        self.assertFalse(summarise_run(lines, "offset", .5, 4)["pass"])

    def test_step_repeats_and_restore(self):
        def sequence(points):
            return [row(f"step{i}", j, error=value) for i, value in enumerate(points)
                    for j in range(4)] + ["LOOP-TEST-END result=OK"]
        result = summarise_step(sequence([0, 1900, 0, 1900, 0]))
        self.assertTrue(result["pass"])
        self.assertEqual(result["repeats"][0]["step_ps"], 19)
        self.assertFalse(summarise_step(sequence([0, 0, 0, 0, 0]))["pass"])
        self.assertFalse(summarise_step(sequence([0, 1900, 0, -1900, 0]))["pass"])
        self.assertFalse(summarise_step(sequence([0, 1900, 0, 900, 0]))["pass"])
        self.assertFalse(summarise_step(sequence([0, -1900, 0, -1900, 0]))["pass"])
        self.assertFalse(summarise_step(sequence([0, 1900, 100, 1900, 100]))["pass"])

    def test_protocol_failures_stop_without_retry(self):
        for line in ("LOOP-SKEW-TXN result=RECOVERY_REQUIRED", "LOOP it=1 act=abort:direction",
                     "LOOP it=1 st=clipping", "Command Error: bad command"):
            port = FakePort(line + "\r\nLOOP-TEST-END result=ERROR\r\n")
            with self.assertRaises(BenchFault):
                Console(port, io.StringIO(), 0.1).require("adc -cal loop test restore")
            self.assertEqual(len(port.writes), 1)
        with self.assertRaises(BenchFault):
            Console(FakePort("LOOP-TEST-END result=ERROR\r\n"), io.StringIO(), .1).require("baseline")

    def test_fragmented_lines_and_point_validation(self):
        lines, ok = Console(FakePort("LOOP-TEST-END result=OK\r\n"), io.StringIO(), .1).command("status")
        self.assertTrue(ok)
        self.assertEqual(fields(lines[0])["result"], "OK")
        self.assertEqual(point_values("gain", "0x0c,13,0"), [12, 13, 0])
        self.assertEqual(point_values("skew", "-4,0,4"), [-4, 0, 4])
        for axis, text in (("gain", "9"), ("offset", "4097"), ("skew", "49")):
            with self.assertRaises(ValueError):
                point_values(axis, text)

    def test_full_sweep_protocol_and_exports(self):
        commands = []

        class ScriptedPort(FakePort):
            def __init__(self, *args, **kwargs):
                super().__init__("")

            def __enter__(self):
                return self

            def __exit__(self, *args):
                pass

            def reset_input_buffer(self):
                pass

            def write(self, data):
                command = data.decode().strip()
                commands.append(command)
                if "loop wave" in command:
                    reply = "LOOP-WAVE selected=0\n"
                elif "test baseline" in command:
                    reply = "LOOP-TEST-BASELINE valid=1 code=24\nLOOP-TEST-END result=OK\n"
                elif "test run" in command:
                    reply = "\n".join(run_lines()) + "\n"
                else:
                    reply = "LOOP-TEST-END result=OK\n"
                self.output += reply.encode()

        module = types.ModuleType("serial")
        module.Serial = ScriptedPort
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "experiment"
            args = parser().parse_args(["sweep", "--axis", "offset", "--points=0", "--repeats", "1",
                                        "--pre", "4", "--train", "4", "--hold", "4",
                                        "--out", str(out), "--execute"])
            with patch.dict("sys.modules", {"serial": module}), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(run(args), 0)
                with self.assertRaises(FileExistsError):
                    run(args)
            self.assertTrue(json.loads((out / "meta.json").read_text())["completed"])
            self.assertIn("True", (out / "summary.csv").read_text())
            self.assertEqual(commands.count("adc -cal loop test restore"), 2)
            self.assertEqual(commands[-1], "adc -cal loop test restore")
            self.assertEqual(sum("test baseline" in c for c in commands), 1)

    def test_clock_motion_during_holdout_fails(self):
        lines = [line.replace("code=24", "code=25") if "SUMMARY phase=hold" in line else line
                 for line in run_lines()]
        self.assertFalse(summarise_run(lines, "offset", .5, 4)["pass"])

    def test_quantization_is_distinct_from_range_limit(self):
        lines = [line.replace("skew_cps=0", "skew_cps=600") for line in run_lines()]
        lines[7] += " act=quantized-zero"
        result = summarise_run(lines, "skew", 10, 4)
        self.assertTrue(result["pass"])
        self.assertTrue(result["quantization_limited"])
        self.assertFalse(result["range_limit"])
        self.assertFalse(summarise_run(lines, "skew", 1, 4)["pass"])


if __name__ == "__main__":
    unittest.main()
