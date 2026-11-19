"""Isolated on-board calibration experiments. No hardware access without --execute.

Requires reference-free 'adc -cal loop test' firmware and shared A/B dither.
The suite command automates offset, gain and skew injection from one baseline.
Budgets count attempts, not just qualified frames. Input sources remain manual.
Logs, per-frame CSV, frozen output snapshots and holdout summaries are retained.
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
IFC_FULL_SCALE_ORDER = (10, 11, 12, 13, 14, 15, 0)
IFC_FULL_SCALE_VPP = dict(zip(IFC_FULL_SCALE_ORDER, (1.36, 1.47, 1.59, 1.70, 1.81, 1.93, 2.04)))
TOLERANCES = {"offset": 0.5, "gain": 0.001, "skew": 10.0}
SUITE_DEFAULT_POINTS = {"offset": tuple(range(32, -33, -1)),
                        "gain": IFC_FULL_SCALE_ORDER, "skew": tuple(range(33))}
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


def audit_routes(rows):
    accepted = [r for r in rows if r.get("st") == "ok"]
    sources = {r.get("gsrc", "") for r in accepted}
    routes = {r.get("skew_route", "") for r in accepted}
    legacy = (not any("skew_route" in r or "used_gain_ppm" in r for r in accepted) and
              sources <= {"", "tone"})
    consistent = legacy or (
        len(sources) == len(routes) == 1 and
        (next(iter(sources)), next(iter(routes))) in
        (("tone", "phase"), ("dither_mag", "dither_phase"),
         ("dither_spectrum", "dither_cross_spectrum")) and
        (routes == {"phase"} or all("used_gain_ppm" in r for r in accepted)))
    return {"gain_source": ",".join(sorted(sources)),
            "skew_route": ",".join(sorted(routes)), "routes_consistent": consistent}


def summarise_run(lines, axis, tolerance, hold_budget, min_yield=0.8, window=20):
    rows = rows_from(lines)
    metrics = dict(AXIS_FIELDS)
    if any("used_gain_ppm" in r for r in rows):
        metrics["gain"] = ("used_gain_ppm", 1e-6)
    key, scale = metrics[axis]
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
    # Audit the observable throughout PRE/TRAIN/HOLD; a fallback must not look
    # like convergence. Old tone-only logs without route fields remain readable.
    result.update(audit_routes(rows))
    for metric, (field, unit) in metrics.items():
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
                      not result["range_limit"] and result["routes_consistent"])
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
    result.update(audit_routes(rows))
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
                      all(r["consistent"] for r in restore) and result["routes_consistent"])
    return result


class BenchFault(RuntimeError):
    """A failed write/direction guard: do not issue automatic recovery writes."""


class Console:
    def __init__(self, serial_port, log, timeout):
        self.serial = serial_port
        self.log = log
        self.timeout = timeout
        self.pending = ""
        self.records = []
        self.context = {}
        self.last_lines = []

    def command(self, command, marker="LOOP-TEST-END", timeout=None):
        if command.startswith(('adc -cal loop test baseline ', 'adc -cal loop test run ',
                               'adc -cal loop test step ')) and not command.endswith(' full'):
            command += ' full'
        lines = []
        self.last_lines = lines
        self.log.write(f"# command: {command}\n")
        self.log.flush()
        self.serial.write((command + "\r").encode("ascii"))
        self.serial.flush()
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        fault_seen = False
        dma_fault = None
        while time.monotonic() < deadline:
            self.pending += self.serial.read(self.serial.in_waiting or 1).decode(errors="replace")
            while "\n" in self.pending:
                line, self.pending = self.pending.split("\n", 1)
                line = line.rstrip("\r")
                if line.startswith('uart-cmd$:'):
                    line = line.split(':', 1)[1].strip()
                payload = line.startswith('LOOP-TEST-ROW ')
                signal_payload = line.startswith('LOOP-SIGNAL-DATA ')
                if payload or line.startswith('LOOP-TEST-CAPTURE '):
                    self.records.append(dict(fields(line), **self.context))
                if not payload and not signal_payload:
                    print(line)
                    self.log.write(line + "\n")
                    self.log.flush()
                lines.append(line)
                if ("RECOVERY_REQUIRED" in line or "act=abort:direction" in line or
                        "act=move:FAILED" in line or "st=clipping" in line or
                        "fault=1" in line or
                        (line.startswith('LOOP-SKEW-TXN ') and fields(line).get('result') != 'OK')):
                    fault_seen = True
                    if payload:
                        row = fields(line)
                        notice = (f"LOOP-TEST-FAULT phase={row.get('phase')} attempt={row.get('attempt')} "
                                  f"st={row.get('st')} act={row.get('act')} clips={row.get('clips')}")
                        print(notice)
                        self.log.write(notice + '\n')
                        self.log.flush()
                if "Command Error" in line:
                    if "DMA capture timeout" in line:
                        # The board can still be inside the running phase.
                        # Retain its remaining records until END; never send
                        # another test or an automatic recovery transaction.
                        fault_seen = True
                        dma_fault = line
                    else:
                        raise BenchFault(f"Firmware error: {line}; no further commands issued.")
                if line.startswith(marker):
                    if dma_fault:
                        raise BenchFault(f"DMA capture timed out during this test: {dma_fault}; "
                                         "command ended, partial data retained; no retry or automatic restore.")
                    if fault_seen:
                        raise BenchFault("Bench fault/clipping; data retained, no retry or automatic restore.")
                    return lines, not ("result=ERROR" in line)
        reason = f" after {dma_fault}" if dma_fault else ""
        raise BenchFault(f"Timeout waiting for {marker}{reason}; no further commands issued.")

    def require(self, command, marker="LOOP-TEST-END", timeout=None):
        lines, ok = self.command(command, marker, timeout)
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
    if len(values) != len(set(values)):
        raise ValueError("points must be unique; use --repeats to repeat a point")
    if axis == "gain" and any(v not in IFC_CODES for v in values):
        raise ValueError("gain points must be IFC codes 0,10,11,12,13,14,15")
    if axis == "offset" and any(abs(v) > 4096 for v in values):
        raise ValueError("software offset must be within +/-4096 native ADC codes")
    if axis == "skew" and any(abs(v) > 48 for v in values):
        raise ValueError("skew points are relative control codes, within +/-48")
    return values


def suite_points(args, baseline_code=None, baseline_ifc=None):
    axes = getattr(args, 'axes', 'offset,gain,skew').split(',')
    if (not axes or len(axes) != len(set(axes)) or
            any(axis not in AXIS_FIELDS for axis in axes)):
        raise ValueError("axes must be unique comma-separated names: offset,gain,skew")
    points = {axis: point_values(axis, text) if text is not None else list(SUITE_DEFAULT_POINTS[axis])
              for axis, text in (("offset", args.offset_points),
                                 ("gain", args.gain_points), ("skew", args.skew_points))}
    points = {axis: values if axis in axes else [] for axis, values in points.items()}
    if args.skew_mode == 'absolute' and any(not 0 <= p <= 48 for p in points['skew']):
        raise ValueError("absolute skew points must be hardware control codes 0..48")
    if baseline_code is None:
        return points
    if baseline_ifc not in IFC_CODES:
        raise ValueError("Saved baseline has no supported channel IFC; no injection sent")
    return points


def skew_target(args, baseline_code, point):
    return point if args.mode == 'suite' and args.skew_mode == 'absolute' else baseline_code + point


def quality_rejection_only(lines):
    """A completed, rejected acquisition phase is a measured FAIL, not a write fault."""
    phases = {fields(line).get('phase'): fields(line) for line in lines
              if line.startswith('LOOP-TEST-PHASE ')}
    summaries = [fields(line) for line in lines if line.startswith('LOOP-TEST-SUMMARY ')]
    rows = rows_from(lines)
    allowed = {'ok', 'margin', 'events', 'a_nan', 'b_nan', 'gain_range', 'skew_range'}
    if (not phases or len(phases) != len(summaries) or not rows or
            any(r.get('st') not in allowed for r in rows) or
            not any(r.get('st') != 'ok' for r in rows) or
            any(line.startswith('LOOP-TEST-CAPTURE ') for line in lines)):
        return False
    for summary in summaries:
        phase = summary.get('phase')
        if phase not in phases or summary.get('abort', '0') != '0':
            return False
        attempts = summary.get('attempts')
        selected = [r for r in rows if r.get('phase') == phase]
        accepted = summary.get('accepted', '')
        if (attempts != phases[phase].get('budget') or not attempts or not attempts.isdecimal() or
                not accepted.isdecimal() or int(accepted) != sum(r.get('st') == 'ok' for r in selected) or
                [r.get('attempt') for r in selected] != [str(i) for i in range(int(attempts))]):
            return False
    return len(rows) == sum(int(s['attempts']) for s in summaries)


def save_point(console, lines, log_path, result, args, read_signal=True):
    """Save calibration before reading its separate frozen output snapshot."""
    if __package__:
        from .axis_result_export import save_test_results
    else:
        from axis_result_export import save_test_results
    advertised = next((fields(line) for line in reversed(lines)
                       if line.startswith('LOOP-OUTPUT ')), {})
    result["signal_status"] = advertised.get("status", "not-advertised")
    result["signal_downloaded"] = False
    result["signal_skipped"] = args.no_signal or not read_signal
    csv_path, json_path, _ = save_test_results(lines, log_path, result)
    signal_path = None
    try:
        if (read_signal and not args.no_signal and int(advertised.get("samples", "0")) > 0 and
                advertised.get("status") != "unavailable"):
            if __package__:
                from .board_signal_output import SignalDecoder
            else:
                from board_signal_output import SignalDecoder
            output_lines = console.require("adc -cal loop output", "LOOP-SIGNAL-END",
                                           timeout=min(args.timeout, 120.0))
            decoder = SignalDecoder()
            for line in output_lines:
                if line.startswith('LOOP-SIGNAL-'):
                    decoder.feed(line)
            if (not decoder.done or decoder.meta["status"] != advertised.get("status") or
                    int(decoder.meta["samples"]) != int(advertised["samples"])):
                raise ValueError("Saved signal differs from advertised snapshot; calibration files retained")
            signal_path = log_path.with_name(log_path.stem + '_signal.csv')
            signal_meta = decoder.save(signal_path)
            result["signal_status"] = signal_meta["status"]
            result["signal_downloaded"] = True
            result["signal_csv"] = signal_path.name
            print(f"SIGNAL {signal_meta['status']}: {signal_path} (independent of HOLD)")
    except Exception as exc:
        result['signal_error'] = str(exc)
        raise
    finally:
        # Keep the HOLD assessment even if waveform transport fails afterward.
        saved = json.loads(json_path.read_text(encoding='utf-8'))
        saved["assessment"].update({k: v for k, v in result.items() if k.startswith('signal_')})
        json_path.write_text(json.dumps(saved, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    if not args.no_viewer:
        if __package__:
            from .loop_viewer_export import write_viewer
        else:
            from loop_viewer_export import write_viewer
        write_viewer(csv_path, signal=signal_path)
    return result


def run(args):
    suite = args.mode == "suite"
    if suite and args.wave is not None:
        raise ValueError("suite uses reference-free calibration; omit --wave")
    if suite and not 150 <= args.acquire_phase_mrad <= 750:
        raise ValueError("acquire-phase-mrad must be 150..750; HOLD stays at 150")
    if args.reuse_baseline and args.wave is not None:
        raise ValueError("reuse-baseline requires reference-free mode; omit --wave")
    if args.wave not in (None, 2, 3):
        raise ValueError("Calibration requires dither-only wave 2 or 3; tone calibration is disabled")
    if args.skew_gate_milli is None:
        args.skew_gate_milli = 500
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
    points = suite_points(args) if suite else (
        point_values(axis, args.points or defaults[axis]) if axis else [])
    tolerance = args.tolerance if args.tolerance is not None else {
        "offset": 0.5, "gain": 0.001, "skew": 10.0, None: 10.0}[axis]
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("tolerance must be positive and finite (codes, ratio, or ps)")
    config = (f"adc -cal loop test config {args.gain_gate_ppm} {args.skew_gate_milli} "
              f"{round(args.deadband_ps * 100)} {round(args.step_ps * 100)} {args.batch}")
    print(f"mode={args.mode} uart={args.uart}; reference-free calibration from shared A/B dither.")
    print(config)
    print(f"baseline={'reuse saved board baseline' if args.reuse_baseline else str(args.baseline_train) + '+' + str(args.baseline_hold)}; "
          f"pre/train/hold={args.pre}/{args.train}/{args.hold}; points={points}")
    if suite:
        print(f"PRE/TRAIN spectral phase gate: {args.acquire_phase_mrad} mrad; HOLD: 150 mrad.")
        print(f"Order: offset -> IFC full scale -> skew ({args.skew_mode} control codes).")
        print(f"Requested tests per repeat: {sum(map(len, points.values()))}; hardware skew range: 0..48.")
    if not args.execute:
        print("PLAN ONLY. Add --execute to connect and perform register/clock writes.")
        return 0
    import serial
    import matplotlib
    matplotlib.use('Agg')

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    records, summaries, skipped = [], [], []
    arguments = {key: value for key, value in vars(args).items() if key != "func"}
    meta = {"arguments": arguments, "tolerance": tolerance, "points": points,
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "completed": False, "restored": False,
            "units": {"offset": "calibrated output codes", "gain": "B/A - 1", "skew": "ps"}}
    if suite:
        meta["tolerance"] = TOLERANCES
    try:
        with (out / "console.log").open("x", encoding="utf-8") as log:
            log.write("# arguments: " + json.dumps(arguments, sort_keys=True) + "\n")
            with serial.Serial(args.uart, 115200, timeout=0.5) as port:
                port.reset_input_buffer()
                console = Console(port, log, args.timeout)
                records = console.records
                if suite:
                    preflight = console.require("adc -cal loop test status")
                    routes = next((fields(line) for line in preflight
                                   if line.startswith('LOOP-TEST-ROUTES ')), {})
                    state = next((fields(line) for line in preflight
                                  if line.startswith('LOOP-TEST-BASELINE ')), {})
                    if (state.get('fault') != '0' or routes.get('reference_free') != '1' or
                            routes.get('tone_free') != '1' or routes.get('gain') != 'dither_spectrum' or
                            routes.get('skew') != 'dither_cross_spectrum' or
                            routes.get('spectral_line_rejection') != '1' or
                            routes.get('gain_freeze') != 'training_mean'):
                        raise ValueError("suite requires the current reference-free gain-freeze ELF and no board fault; no writes sent")
                    if args.acquire_phase_mrad != 150 and routes.get('acquisition_phase_gate') != '1':
                        raise ValueError("Requested acquisition phase gate requires the updated ELF; no writes sent")
                    quality_supported = routes.get('acquisition_phase_gate') == '1'
                if args.wave is not None:  # optional compatibility with older firmware
                    console.require(f"adc -cal loop wave {args.wave}", "LOOP-WAVE")
                if args.reuse_baseline:
                    # Read-only preflight: do not change a valid baseline's gates
                    # or clear the previous injection until status is verified.
                    initial = preflight if suite else console.require("adc -cal loop test status")
                    baseline = next((fields(line) for line in initial
                                     if line.startswith("LOOP-TEST-BASELINE ")), {})
                    routes = next((fields(line) for line in initial
                                   if line.startswith("LOOP-TEST-ROUTES ")), {})
                    current = next((fields(line) for line in initial
                                    if line.startswith("LOOP-TEST-CONFIG ")), {})
                    if (baseline.get("valid") != "1" or baseline.get("snapshot") != "1" or
                            baseline.get("fault") != "0" or not 0 <= int(baseline.get("code", -1)) <= 48):
                        raise ValueError("No valid saved baseline; establish baseline first. No test writes sent.")
                    if (routes.get("reference_free") != "1" or routes.get("tone_free") != "1" or
                            routes.get("gain") != "dither_spectrum" or
                            routes.get("skew") != "dither_cross_spectrum"):
                        raise ValueError("Saved baseline must use reference-free dither routes")
                    expected = dict(zip(("gain_ppm", "skew_milli", "deadband_cps", "step_cps", "batch"),
                                        config.split()[-5:]))
                    if any(current.get(k) != v for k, v in expected.items()):
                        raise ValueError("Board config differs from requested config; use matching CLI settings. No config writes sent.")
                    meta["reused_baseline"] = True
                    meta["board_config"] = current
                else:
                    console.require(config)
                    console.require("adc -cal loop test offset A 0")
                    console.require("adc -cal loop test offset B 0")
                    console.context = {"experiment": "baseline"}
                    baseline_command = f"adc -cal loop test baseline {args.baseline_train} {args.baseline_hold}"
                    if suite:
                        initial, baseline_ok = console.command(baseline_command)
                        if __package__:
                            from .axis_result_export import save_test_results
                        else:
                            from axis_result_export import save_test_results
                        save_test_results(initial, out / 'baseline.log')
                        if not baseline_ok:
                            raise BenchFault("Baseline admission failed; files retained, no injections or retries sent")
                    else:
                        initial = console.require(baseline_command)
                    baseline = next(fields(line) for line in initial if line.startswith("LOOP-TEST-BASELINE "))
                base_code = int(baseline["code"])
                if baseline.get('valid') != '1' or not 0 <= base_code <= 48:
                    raise ValueError("Baseline is not valid; no injections sent")
                meta["baseline"] = baseline
                baseline_ifc = None
                if suite:
                    snapshots = {fields(line).get('channel'): fields(line) for line in initial
                                 if line.startswith('LOOP-TEST-SNAPSHOT ')}
                    saved = snapshots.get('1' if args.channel == 'B' else '0', {})
                    if 'ifc' not in saved or not 0 <= int(saved['ifc']) <= 255:
                        raise ValueError("Missing baseline channel IFC snapshot; no injections sent")
                    baseline_ifc = int(saved['ifc']) & 0x0F
                    points = suite_points(args, base_code, baseline_ifc)
                    meta['points'] = points
                    meta['baseline_ifc'] = baseline_ifc
                    executable = {a: list(p) for a, p in points.items()}
                    executable['skew'] = [p for p in points['skew'] if 0 <= skew_target(args, base_code, p) <= 48]
                    for repeat in range(args.repeats):
                        for p in points['skew']:
                            if not 0 <= skew_target(args, base_code, p) <= 48:
                                skipped.append(dict(axis='skew', point=p, repeat=repeat,
                                                    baseline_code=base_code, target_code=skew_target(args, base_code, p),
                                                    status='SKIP', reason='outside hardware code range 0..48'))
                    meta['skew_point_mode'] = args.skew_mode
                    meta['executable_points'] = executable
                    meta['skipped_points'] = skipped
                    meta['coverage_complete'] = not skipped
                    meta['requested_tests'] = sum(map(len, points.values())) * args.repeats
                    meta['skipped_tests'] = len(skipped)
                    print(f"Resolved suite points: {points}; baseline code={base_code}, IFC=0x{baseline_ifc:02X}")
                    for skip in skipped:
                        notice = (f"SKIP skew={skip['point']} repeat={skip['repeat']+1}: "
                                  f"target={skip['target_code']} outside 0..48 (baseline={base_code})")
                        print(notice)
                        log.write(notice + '\n')
                    if quality_supported:
                        quality = console.require(f"adc -cal loop test quality {args.acquire_phase_mrad}")
                        meta['board_quality'] = next(fields(line) for line in quality
                                                     if line.startswith('LOOP-TEST-QUALITY '))
                if axis == "skew" and any(not 0 <= base_code + p <= 48 for p in points):
                    raise ValueError(f"baseline code={base_code}; requested skew points exceed 0..48 (no clamping)")
                if args.mode == "step":
                    code = base_code if args.code is None else args.code
                    if not 0 <= code <= 47:
                        raise ValueError("step base must be 0..47")
                    console.context = {"experiment": "step"}
                    lines, _ = console.command(f"adc -cal loop test step {code} {args.frames}")
                    meta["step"] = summarise_step(lines, args.frames, args.min_yield)
                    if args.wave in (None, 2, 3):
                        meta["step"]["requested_routes_match"] = (
                            (meta["step"]["gain_source"], meta["step"]["skew_route"]) in
                            (("dither_mag", "dither_phase"), ("dither_spectrum", "dither_cross_spectrum")))
                        meta["step"]["pass"] &= meta["step"]["requested_routes_match"]
                    if __package__:
                        from .axis_result_export import save_test_results
                    else:
                        from axis_result_export import save_test_results
                    save_test_results(lines, out / 'step.log', meta['step'])
                    console.require("adc -cal loop test restore")
                else:
                    experiments = ([(a, p) for a in AXIS_FIELDS for p in executable[a]] if suite else
                                   [(axis, p) for p in points])
                    for repeat in range(args.repeats):
                        for point_axis, point in experiments:
                            meta['active_point'] = dict(axis=point_axis, point=point, repeat=repeat+1)
                            console.require("adc -cal loop test restore")
                            if point_axis == "skew":
                                target_code = skew_target(args, base_code, point)
                                inject = f"park {target_code}"
                            elif point_axis == "gain":
                                inject = f"ifc {args.channel} {point}"
                            else:
                                inject = f"offset {args.channel} {point}"
                            injection_lines = console.require("adc -cal loop test " + inject)
                            if suite and point_axis == 'skew' and target_code != base_code:
                                transactions = [fields(line) for line in injection_lines
                                                if line.startswith('LOOP-SKEW-TXN ')]
                                if (not transactions or
                                        transactions[-1].get('code_after') != str(target_code)):
                                    raise BenchFault("Skew injection lacks the expected final ACK code; no further writes sent")
                            console.context = {"experiment": point_axis, "point": point, "repeat": repeat}
                            point_dir = out / point_axis if suite else out
                            point_dir.mkdir(exist_ok=True)
                            point_log = point_dir / f"{point_axis}_point{point}_repeat{repeat+1}.log"
                            command = f"adc -cal loop test run {point_axis} {args.pre} {args.train} {args.hold}"
                            try:
                                lines, ended_ok = console.command(command)
                            except BaseException as exc:
                                # Keep a complete failed test export when END was received;
                                # frames.csv also retains partial rows on timeout/interrupt.
                                if suite and any(line.startswith('LOOP-TEST-END ') for line in console.last_lines):
                                    failed = summarise_run(console.last_lines, point_axis, TOLERANCES[point_axis],
                                                           args.hold, args.min_yield, args.window)
                                    failed.update(point=point, repeat=repeat, baseline_code=base_code,
                                                  aborted=True, stop_reason=str(exc))
                                    summaries.append(failed)
                                    try:
                                        save_point(console, console.last_lines, point_log, failed, args, read_signal=False)
                                    except (ValueError, OSError) as export_error:
                                        log.write(f"# partial export: {export_error}\n")
                                raise
                            point_tolerance = TOLERANCES[point_axis] if suite else tolerance
                            result = summarise_run(lines, point_axis, point_tolerance, args.hold, args.min_yield, args.window)
                            if args.wave in (None, 2, 3):
                                result["requested_routes_match"] = (
                                    (result["gain_source"], result["skew_route"]) in
                                    (("dither_mag", "dither_phase"), ("dither_spectrum", "dither_cross_spectrum")))
                                result["pass"] &= result["requested_routes_match"]
                            result.update(point=point, repeat=repeat, baseline_code=base_code)
                            if point_axis == 'gain':
                                result['nominal_ifc_vpp'] = IFC_FULL_SCALE_VPP[point]
                            elif point_axis == 'skew':
                                result['target_code'] = target_code
                                result['relative_code_offset'] = target_code - base_code
                                result['skew_point_mode'] = args.skew_mode if suite else 'relative'
                            rejected_acquisition = not ended_ok and quality_rejection_only(lines)
                            result['measurement_rejected'] = rejected_acquisition
                            if rejected_acquisition:
                                result['failure_reason'] = 'acquisition range/quality gate rejected the point'
                            if suite:
                                if point_axis == 'gain':
                                    result['is_control'] = point == baseline_ifc
                                elif point_axis == 'skew':
                                    result['is_control'] = target_code == base_code
                                else:
                                    result['is_control'] = point == 0
                            summaries.append(result)
                            save_point(console, lines, point_log, result, args)
                            if suite and not ended_ok and not rejected_acquisition:
                                raise BenchFault("Test ended with ERROR; records retained, no further injections or automatic restore")
                            outcome = ('acquisition rejected; HOLD not qualified' if rejected_acquisition else
                                       f"HOLD={result[point_axis + '_mean']:.6g}")
                            print(f"POINT {'PASS' if result['pass'] else 'FAIL'}: {point_axis}={point} "
                                  f"repeat={repeat+1}/{args.repeats} "
                                  f"{outcome}; files saved", flush=True)
                            # Preserve completed points even if a later transaction fails.
                            log.write("# result: " + json.dumps(result, sort_keys=True) + "\n")
                    console.require("adc -cal loop test restore")
                    meta['restored'] = True
                    meta['active_point'] = None
                meta["completed"] = True
    except BaseException as exc:
        meta["stop_reason"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        meta["ended_utc"] = datetime.now(timezone.utc).isoformat()
        meta['recorded_tests'] = len(summaries)
        write_csv(out / "frames.csv", records)
        write_csv(out / "summary.csv", summaries)
        write_csv(out / "skipped.csv", skipped)
        with (out / "meta.json").open("x", encoding="utf-8") as fh:
            json.dump(meta, fh, indent=2)
        print(f"Results: {out}")
    if args.mode == "step":
        print(json.dumps(meta["step"], indent=2))
        return 0 if meta["step"]["pass"] else 1
    for summary in summaries:
        selected_axis = summary['axis']
        print(f"axis={selected_axis} point={summary['point']} repeat={summary['repeat']} "
              f"{'PASS' if summary['pass'] else 'FAIL'} "
              f"residual={summary[selected_axis + '_mean']:.6g} {meta['units'][selected_axis]}")
    if suite:
        print(f"Sweep processed: requested={meta['requested_tests']} executed={len(summaries)} "
              f"SKIP={len(skipped)}")
        if skipped:
            print("See skipped.csv/meta.json for unreachable relative skew codes")
    if suite and not args.no_viewer:
        if __package__:
            from .loop_viewer_export import write_viewer
        else:
            from loop_viewer_export import write_viewer
        print(f"viewer: {write_viewer(out, open_browser=True)}")
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
    for name in ("sweep", "step", "suite"):
        sp = sub.add_parser(name)
        sp.add_argument("--uart", default="COM5")
        if name != 'suite':
            sp.add_argument("--wave", type=int, choices=(2, 3), default=None,
                            help="legacy firmware compatibility only; omit for reference-free calibration")
        else:
            sp.set_defaults(wave=None)
        sp.add_argument("--out", default="calibration_out/" +
                        ("axis_suite_" if name == 'suite' else "axis_") + time.strftime("%Y%m%d_%H%M%S"))
        sp.add_argument("--execute", action="store_true", help="execute serial commands; default: print plan only")
        sp.add_argument("--no-signal", action="store_true", help="skip saved output-waveform downloads")
        sp.add_argument("--no-viewer", action="store_true", help="skip HTML generation and browser opening")
        sp.add_argument("--reuse-baseline", action="store_true",
                        help="validate and reuse the current board baseline; do not recalibrate or rewrite config")
        sp.add_argument("--timeout", type=float, default=1800.0)
        for opt, default in (("pre", 40), ("train", 300), ("hold", 100),
                             ("baseline-train", 600), ("baseline-hold", 100)):
            sp.add_argument("--" + opt, type=int, choices=range(2, 2001), default=default,
                            metavar="2..2000")
        sp.add_argument("--gain-gate-ppm", type=int, default=600000 if name == 'suite' else 200000,
                        help="acquisition gate: suite default 600000 covers all IFC ranges; HOLD limits unchanged")
        sp.add_argument("--skew-gate-milli", type=int,
                        help="default: 500 (reference-free acquisition range)")
        sp.add_argument("--deadband-ps", type=float, default=10.0)
        sp.add_argument("--step-ps", type=float, default=19.1)
        sp.add_argument("--batch", type=int, default=20)
        if name != 'suite':
            sp.add_argument("--tolerance", type=float)
        else:
            sp.set_defaults(tolerance=None)
        sp.add_argument("--min-yield", type=float, default=0.8)
        sp.add_argument("--window", type=int, default=20)
        sp.set_defaults(func=run)
        if name == "sweep":
            sp.add_argument("--axis", choices=tuple(AXIS_FIELDS), required=True)
            sp.add_argument("--channel", choices=("A", "B"), default="B")
            sp.add_argument("--points", help="comma-separated; skew relative codes, gain IFC register codes")
            sp.add_argument("--repeats", type=int, default=5)
        elif name == 'step':
            sp.add_argument("--code", type=int, help="base of paired step (default: converged baseline code)")
            sp.add_argument("--frames", type=int, choices=range(2, 2001), default=55, metavar="2..2000")
        else:
            sp.add_argument("--channel", choices=("A", "B"), default="B")
            sp.add_argument("--acquire-phase-mrad", type=int, default=150,
                            help="PRE/TRAIN spectral phase RMS limit, 150..750; HOLD stays 150 (updated ELF required)")
            sp.add_argument("--axes", default="offset,gain,skew",
                            help="comma-separated axes to run; default: offset,gain,skew")
            sp.add_argument("--offset-points", help="ADC codes; default: 32,31,...,-32")
            sp.add_argument("--gain-points", help="IFC codes; default: 10,11,12,13,14,15,0 (1.36..2.04 Vpp)")
            sp.add_argument("--skew-mode", choices=('absolute', 'relative'), default='absolute',
                            help="default: absolute hardware codes; relative adds the baseline code")
            sp.add_argument("--skew-points", help="default: 0,1,...,32 absolute hardware codes")
            sp.add_argument("--repeats", type=int, default=1)
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
