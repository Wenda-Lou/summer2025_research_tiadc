"""Isolated on-board calibration experiments. No hardware access without --execute.

Requires the new 'adc -cal loop test' firmware and the selected tone+dither DPG
vector. Budgets count attempts, not just qualified frames. The waveform is loaded
manually. Logs, per-frame CSV and independent holdout summaries are retained.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import statistics
import time


FIELDS = re.compile(r"([A-Za-z_][A-Za-z_0-9]*)=([^\s]+)")
IFC_CODES = {0, 10, 11, 12, 13, 14, 15}
AXIS_FIELDS = {"offset": ("doff_mc", 1e-3), "gain": ("tone_ppm", 1e-6),
               "skew": ("skew_cps", 0.01)}
STATE_FIELDS = ("corr_a_ppm", "corr_b_ppm", "offa_mc", "offb_mc")


def fields(line):
    return dict(FIELDS.findall(line))


def number(row, key, scale=1.0):
    try:
        return float(row[key]) * scale
    except (ValueError, KeyError):
        return math.nan


def stats(values):
    values = [v for v in values if math.isfinite(v)]
    if not values:
        return {"n": 0, "mean": math.nan, "sd": math.nan, "se": math.nan}
    sd = statistics.stdev(values) if len(values) > 1 else math.nan
    return {"n": len(values), "mean": statistics.mean(values), "sd": sd,
            "se": sd / math.sqrt(len(values))}


def within(s, tolerance):
    # Normal-approximation margin; keep the frame SD visible, not just the SE.
    return s["n"] >= 2 and abs(s["mean"]) + 1.96 * s["se"] <= tolerance


def rows_from(lines):
    return [fields(line) for line in lines if line.startswith("LOOP-TEST-ROW ")]


def summarise_run(lines, axis, tolerance, hold_budget, min_yield=0.8, window=20):
    rows = rows_from(lines)
    key, scale = AXIS_FIELDS[axis]
    hold_all = [r for r in rows if r.get("phase") == "hold"]
    hold = [r for r in hold_all if r.get("st") == "ok"]
    pre = [r for r in rows if r.get("phase") == "pre" and r.get("st") == "ok"]
    train = [r for r in rows if r.get("phase") == "train" and r.get("st") == "ok"]
    result = {"axis": axis, "tolerance": tolerance,
              "hold_accepted": len(hold), "hold_budget": hold_budget,
              "hold_yield": len(hold) / hold_budget,
              "pre_mean": stats(number(r, key, scale) for r in pre)["mean"]}
    result["pre_observed_mean"] = stats(
        number(r, key, scale) for r in rows if r.get("phase") == "pre" and
        r.get("st") in ("ok", "gain_range", "skew_range"))["mean"]
    for metric, (field, unit) in AXIS_FIELDS.items():
        for name, value in stats(number(r, field, unit) for r in hold).items():
            result[f"{metric}_{name}"] = value
    rejected = {}
    for r in rows:
        if r.get("st") != "ok":
            reason = r.get("st", "unknown")
            rejected[reason] = rejected.get(reason, 0) + 1
    rejected["capture"] = rejected.get("capture", 0) + sum(
        line.startswith("LOOP-TEST-CAPTURE ") for line in lines)
    result["reject_reasons"] = json.dumps({k: v for k, v in rejected.items() if v}, sort_keys=True)
    result["hold_frozen"] = bool(hold_all) and all(
        r.get(k) == hold_all[0].get(k) for r in hold_all for k in STATE_FIELDS)
    # Check the holdout really starts with the final trained coefficients.
    train_all = [r for r in rows if r.get("phase") == "train"]
    result["hold_continues_train"] = bool(train_all and hold_all) and all(
        train_all[-1].get(k) == hold_all[0].get(k) for k in STATE_FIELDS)
    frozen = STATE_FIELDS[:2] if axis == "offset" else (
        STATE_FIELDS[2:] if axis == "gain" else STATE_FIELDS)
    result["inactive_frozen"] = bool(rows) and all(
        r.get(k) == rows[0].get(k) for r in rows for k in frozen)
    phase_codes = {fields(line).get("phase"): fields(line).get("code") for line in lines
                   if line.startswith("LOOP-TEST-SUMMARY ")}
    result["hold_clock_frozen"] = (phase_codes.get("train") is not None and
                                   phase_codes.get("train") == phase_codes.get("hold") and
                                   len({r["code"] for r in hold_all if "code" in r}) <= 1)
    result["inactive_skew_frozen"] = axis == "skew" or (
        len(set(phase_codes.values())) == 1 and not any("act=move:" in line for line in lines))
    result["last_code"] = next((fields(line).get("code") for line in reversed(lines)
                                if line.startswith("LOOP-TEST-SUMMARY ")), None)
    # A range limit is a separate outcome even if the last residual happens to
    # satisfy a loose user tolerance. The log retains the complete trajectory.
    result["range_limit"] = any("act=at-range-limit" in line for line in lines)
    result["quantization_limited"] = any("act=quantized-zero" in line for line in lines)
    result["last_train_action"] = next((r.get("act") for r in reversed(rows)
                                        if r.get("phase") == "train" and "act" in r), None)
    result["convergence_attempt"] = None
    last_bad = -1
    windows = []
    for i in range(window - 1, len(train)):
        good = within(stats(number(r, key, scale) for r in train[i + 1 - window:i + 1]), tolerance)
        windows.append((i, good))
        if not good:
            last_bad = i
    if windows and windows[-1][1]:
        first = next(i for i, good in windows if good and i > last_bad)
        result["convergence_attempt"] = int(train[first]["attempt"]) + 1
    selected = stats(number(r, key, scale) for r in hold)
    ended = any(line.startswith("LOOP-TEST-END result=OK") for line in lines)
    finite = selected["n"] == len(hold)
    result["pass"] = (ended and len(hold_all) == hold_budget and
                      len(hold) >= math.ceil(min_yield * hold_budget) and finite and
                      within(selected, tolerance) and result["hold_frozen"] and
                      result["hold_continues_train"] and result["inactive_frozen"] and
                      result["hold_clock_frozen"] and result["inactive_skew_frozen"] and
                      not result["range_limit"])
    return result


def summarise_step(lines, frame_budget=None, min_yield=0.8):
    rows = rows_from(lines)
    states = []
    for i in range(5):
        selected = [r for r in rows if r.get("phase") == f"step{i}" and r.get("st") == "ok"]
        states.append(stats(number(r, "skew_cps", 0.01) for r in selected))
    repeats = []
    restore = []
    for i in (1, 3):
        before, after = states[i - 1], states[i]
        delta = after["mean"] - before["mean"]
        se = math.hypot(before["se"], after["se"])
        repeats.append({"step_ps": delta, "se_ps": se,
                        "significant": before["n"] >= 2 and after["n"] >= 2 and abs(delta) > 1.96 * se})
    for i in (2, 4):
        delta = states[i]["mean"] - states[0]["mean"]
        se = math.hypot(states[i]["se"], states[0]["se"])
        restore.append({"delta_ps": delta, "se_ps": se,
                        "consistent": states[i]["n"] >= 2 and abs(delta) <= max(0.02, 1.96 * se)})
    result = {"states": states, "repeats": repeats, "restores": restore}
    result["repeats_consistent"] = abs(repeats[0]["step_ps"] - repeats[1]["step_ps"]) <= max(
        0.02, 1.96 * math.hypot(repeats[0]["se_ps"], repeats[1]["se_ps"]))
    result["coefficients_frozen"] = bool(rows) and all(
        r.get(k) == rows[0].get(k) for r in rows for k in STATE_FIELDS)
    result["control_sign_ok"] = all(r["step_ps"] > 0 for r in repeats)
    enough = frame_budget is None or all(s["n"] >= math.ceil(min_yield * frame_budget) for s in states)
    result["pass"] = (any(line.startswith("LOOP-TEST-END result=OK") for line in lines) and
                      enough and result["coefficients_frozen"] and result["repeats_consistent"] and
                      result["control_sign_ok"] and
                      all(r["significant"] for r in repeats) and
                      repeats[0]["step_ps"] * repeats[1]["step_ps"] > 0 and
                      all(r["consistent"] for r in restore))
    return result


class BenchFault(RuntimeError):
    """A failed write/direction guard: do not issue automatic recovery writes."""


class Console:
    def __init__(self, serial_port, log, timeout):
        self.serial = serial_port
        self.log = log
        self.timeout = timeout
        self.pending = ""

    def command(self, command, marker="LOOP-TEST-END"):
        self.log.write(f"# command: {command}\n")
        self.log.flush()
        self.serial.write((command + "\r").encode("ascii"))
        self.serial.flush()
        deadline = time.monotonic() + self.timeout
        lines = []
        while time.monotonic() < deadline:
            self.pending += self.serial.read(self.serial.in_waiting or 1).decode(errors="replace")
            while "\n" in self.pending:
                line, self.pending = self.pending.split("\n", 1)
                line = line.rstrip("\r")
                print(line)
                self.log.write(line + "\n")
                self.log.flush()
                lines.append(line)
                if ("RECOVERY_REQUIRED" in line or "act=abort:direction" in line or
                        "act=move:FAILED" in line or "st=clipping" in line or
                        "fault=1" in line):
                    raise BenchFault("Bench fault/clipping; stopped without retry or automatic restore.")
                if "Command Error" in line:
                    raise BenchFault("Firmware rejected the command; check that the new ELF is loaded.")
                if line.startswith(marker):
                    return lines, not ("result=ERROR" in line)
        raise BenchFault(f"Timeout waiting for {marker}; no further commands issued.")

    def require(self, command, marker="LOOP-TEST-END"):
        lines, ok = self.command(command, marker)
        if not ok:
            raise BenchFault(f"Setup/restore failed: {command}; see the log.")
        return lines


def write_csv(path, records):
    if not records:
        return
    columns = list(dict.fromkeys(key for row in records for key in row))
    with path.open("x", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns)
        writer.writeheader()
        writer.writerows(records)


def point_values(axis, text):
    values = [int(s.strip(), 0) for s in text.split(",")]
    if not values:
        raise ValueError("at least one point is required")
    if axis == "gain" and any(v not in IFC_CODES for v in values):
        raise ValueError("gain points must be IFC codes 0,10,11,12,13,14,15")
    if axis == "offset" and any(abs(v) > 4096 for v in values):
        raise ValueError("software offset must be within +/-4096 native ADC codes")
    if axis == "skew" and any(abs(v) > 48 for v in values):
        raise ValueError("skew points are relative control codes, within +/-48")
    return values


def run(args):
    if not 0 < args.min_yield <= 1 or args.window < 2 or getattr(args, "repeats", 1) < 1:
        raise ValueError("require 0 < min-yield <= 1, window >= 2 and repeats >= 1")
    if (not 1 <= args.gain_gate_ppm <= 2000000 or not 1 <= args.skew_gate_milli <= 500 or
            not 2 <= args.batch <= 256 or not math.isfinite(args.timeout) or args.timeout <= 0 or
            not math.isfinite(args.deadband_ps) or not 0 <= args.deadband_ps <= 1000 or
            not math.isfinite(args.step_ps) or not 0.01 <= args.step_ps <= 1000):
        raise ValueError("invalid gate/batch/deadband/step/timeout; no bench access performed")
    if args.mode == "step" and args.code is not None and not 0 <= args.code <= 47:
        raise ValueError("step base must be 0..47")
    axis = getattr(args, "axis", None)
    defaults = {"offset": "-64,-16,-4,0,4,16,64", "gain": "12,13,11,14,10,15,0",
                "skew": "-8,-4,-2,-1,0,1,2,4,8"}
    points = point_values(axis, args.points or defaults[axis]) if axis else []
    tolerance = args.tolerance if args.tolerance is not None else {
        "offset": 0.5, "gain": 0.001, "skew": 10.0, None: 10.0}[axis]
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("tolerance must be positive and finite (codes, ratio, or ps)")
    config = (f"adc -cal loop test config {args.gain_gate_ppm} {args.skew_gate_milli} "
              f"{round(args.deadband_ps * 100)} {round(args.step_ps * 100)} {args.batch}")
    print(f"mode={args.mode} uart={args.uart} wave={args.wave}; load the matching DPG vector manually.")
    print(config)
    print(f"baseline={args.baseline_train}+{args.baseline_hold} attempts; "
          f"pre/train/hold={args.pre}/{args.train}/{args.hold}; points={points}")
    if not args.execute:
        print("PLAN ONLY. Add --execute to connect and perform register/clock writes.")
        return 0
    import serial

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    records, summaries = [], []
    arguments = {key: value for key, value in vars(args).items() if key != "func"}
    meta = {"arguments": arguments, "tolerance": tolerance, "points": points,
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "completed": False, "units": {"offset": "calibrated output codes", "gain": "B/A - 1", "skew": "ps"}}
    try:
        with (out / "console.log").open("x", encoding="utf-8") as log:
            log.write("# arguments: " + json.dumps(arguments, sort_keys=True) + "\n")
            with serial.Serial(args.uart, 115200, timeout=0.5) as port:
                port.reset_input_buffer()
                console = Console(port, log, args.timeout)
                console.require(f"adc -cal loop wave {args.wave}", "LOOP-WAVE")
                console.require(config)
                console.require("adc -cal loop test offset A 0")
                console.require("adc -cal loop test offset B 0")
                initial = console.require(f"adc -cal loop test baseline {args.baseline_train} {args.baseline_hold}")
                records.extend(dict(r, experiment="baseline") for r in rows_from(initial))
                baseline = next(fields(line) for line in initial if line.startswith("LOOP-TEST-BASELINE "))
                base_code = int(baseline["code"])
                meta["baseline"] = baseline
                if axis == "skew" and any(not 0 <= base_code + p <= 48 for p in points):
                    raise ValueError(f"baseline code={base_code}; requested skew points exceed 0..48 (no clamping)")
                if args.mode == "step":
                    code = base_code if args.code is None else args.code
                    if not 0 <= code <= 47:
                        raise ValueError("step base must be 0..47")
                    lines, _ = console.command(f"adc -cal loop test step {code} {args.frames}")
                    records.extend(dict(r, experiment="step") for r in rows_from(lines))
                    meta["step"] = summarise_step(lines, args.frames, args.min_yield)
                    console.require("adc -cal loop test restore")
                else:
                    for repeat in range(args.repeats):
                        for point in points:
                            console.require("adc -cal loop test restore")
                            if axis == "skew":
                                inject = f"park {base_code + point}"
                            elif axis == "gain":
                                inject = f"ifc {args.channel} {point}"
                            else:
                                inject = f"offset {args.channel} {point}"
                            console.require("adc -cal loop test " + inject)
                            lines, _ = console.command(f"adc -cal loop test run {axis} {args.pre} {args.train} {args.hold}")
                            records.extend(dict(r, experiment=axis, point=point, repeat=repeat)
                                           for r in rows_from(lines))
                            result = summarise_run(lines, axis, tolerance, args.hold, args.min_yield, args.window)
                            result.update(point=point, repeat=repeat, baseline_code=base_code)
                            summaries.append(result)
                            # Preserve completed points even if a later transaction fails.
                            log.write("# result: " + json.dumps(result, sort_keys=True) + "\n")
                    console.require("adc -cal loop test restore")
                meta["completed"] = True
    except BaseException as exc:
        meta["stop_reason"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        meta["ended_utc"] = datetime.now(timezone.utc).isoformat()
        write_csv(out / "frames.csv", records)
        write_csv(out / "summary.csv", summaries)
        with (out / "meta.json").open("x", encoding="utf-8") as fh:
            json.dump(meta, fh, indent=2)
        print(f"Results: {out}")
    if args.mode == "step":
        print(json.dumps(meta["step"], indent=2))
        return 0 if meta["step"]["pass"] else 1
    for summary in summaries:
        print(f"point={summary['point']} repeat={summary['repeat']} "
              f"{'PASS' if summary['pass'] else 'FAIL'} "
              f"residual={summary[axis + '_mean']:.6g} {meta['units'][axis]}")
    return 0 if all(s["pass"] for s in summaries) else 1


def analyse(args):
    lines = Path(args.log).read_text(encoding="utf-8").splitlines()
    # For a single test command recorded with board_loop_log.py.
    result = summarise_step(lines) if args.axis == "step" else summarise_run(
        lines, args.axis, args.tolerance, args.hold, args.min_yield, args.window)
    print(json.dumps(result, indent=2))
    return 0 if result["pass"] else 1


def parser():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="mode", required=True)
    for name in ("sweep", "step"):
        sp = sub.add_parser(name)
        sp.add_argument("--uart", default="COM7")
        sp.add_argument("--wave", type=int, choices=(0, 1), default=0)
        sp.add_argument("--out", default="calibration_out/axis_" + time.strftime("%Y%m%d_%H%M%S"))
        sp.add_argument("--execute", action="store_true")
        sp.add_argument("--timeout", type=float, default=1800.0)
        for opt, default in (("pre", 40), ("train", 300), ("hold", 100),
                             ("baseline-train", 600), ("baseline-hold", 100)):
            sp.add_argument("--" + opt, type=int, choices=range(2, 2001), default=default,
                            metavar="2..2000")
        sp.add_argument("--gain-gate-ppm", type=int, default=200000)
        sp.add_argument("--skew-gate-milli", type=int, default=250)
        sp.add_argument("--deadband-ps", type=float, default=10.0)
        sp.add_argument("--step-ps", type=float, default=19.1)
        sp.add_argument("--batch", type=int, default=20)
        sp.add_argument("--tolerance", type=float)
        sp.add_argument("--min-yield", type=float, default=0.8)
        sp.add_argument("--window", type=int, default=20)
        sp.set_defaults(func=run)
        if name == "sweep":
            sp.add_argument("--axis", choices=tuple(AXIS_FIELDS), required=True)
            sp.add_argument("--channel", choices=("A", "B"), default="B")
            sp.add_argument("--points", help="comma-separated; skew relative codes, gain IFC register codes")
            sp.add_argument("--repeats", type=int, default=5)
        else:
            sp.add_argument("--code", type=int, help="base of paired step (default: converged baseline code)")
            sp.add_argument("--frames", type=int, choices=range(2, 2001), default=55, metavar="2..2000")
    sp = sub.add_parser("analyse")
    sp.add_argument("--log", required=True)
    sp.add_argument("--axis", choices=(*AXIS_FIELDS, "step"), required=True)
    sp.add_argument("--tolerance", type=float, default=0.5)
    sp.add_argument("--hold", type=int, default=100)
    sp.add_argument("--min-yield", type=float, default=0.8)
    sp.add_argument("--window", type=int, default=20)
    sp.set_defaults(func=analyse)
    return ap


if __name__ == "__main__":
    try:
        arguments = parser().parse_args()
        raise SystemExit(arguments.func(arguments))
    except (BenchFault, ValueError, OSError) as exc:
        raise SystemExit(str(exc)) from exc
