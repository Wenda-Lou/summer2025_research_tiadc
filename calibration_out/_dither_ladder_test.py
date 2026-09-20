"""Instrument check for tools/dither_ladder.py, including a positive control.

The ladder tool is what turns a set of captured states into the answer to "how far can the
dither be pushed".  Its linearity check therefore has to be able to *fail* when the chain really
does compress -- a check that only ever passes proves nothing.  Two ladders are synthesized here
from the real generator and the bench model:

  * a clean ladder (2000 -> 30000 LSB): every state is linear, so every check must pass;
  * a compressing ladder: the top two states lose 5 % and 15 % of their analog gain, which is
    what a saturating path looks like.  The tool must report the deficit and fail those states.

The frames carry the real pulse geometry, the real 130-sample period, the real balanced polarity
sequence, quantisation, noise and a random DMA rotation, so the measurement itself is exercised,
not just the arithmetic on top of it.

    python calibration_out/_dither_ladder_test.py
"""

from __future__ import annotations

import io
import os
import shutil
import sys
import contextlib
import tempfile

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "tools"))

from calibration_loop.dither import DitherConfig, write_dac_files     # noqa: E402
from calibration_loop.simulate import BenchModel                      # noqa: E402
import dither_ladder                                                  # noqa: E402

LADDER = (2000.0, 8000.0, 16000.0, 24000.0, 30000.0)
FRAMES = 12
BENCH_SCALE = 0.0245
"""Bench-like path gain: 0.0245 ADC codes pk-pk per DAC LSB.

Measured 2026-09-19 on the bench, where the folded replica was 48.7 codes at 2000 LSB and 391.3
codes at 16000 LSB -- i.e. the analog path passes the impulse about twice as strongly as the
model's nominal ``adc_scale`` of 0.0125 suggests.  Getting this wrong is not cosmetic: at half
the amplitude the 2000 LSB state is noise-dominated, its per-event range inflates and the
ladder's lowest step reads as compression that is not there."""


def build(root: str, compression: dict | None = None) -> tuple[list[str], list[str]]:
    """One directory of frames and one waveform JSON per ladder state."""
    states, jsons, amps = [], [], []
    for amp in LADDER:
        cfg = DitherConfig(amp_dbfs=-120.0, dither_edge_dac=4, dither_top_dac=4,
                           dither_scale_lsb=amp)
        cfg.validate()
        d = os.path.join(root, f"a{int(amp)}")
        os.makedirs(d, exist_ok=True)
        loss = (compression or {}).get(amp, 1.0)
        bench = BenchModel(cfg=cfg, adc_scale=BENCH_SCALE * loss, noise_rms_codes=3.0,
                           seed=11)
        for i in range(FRAMES):
            with open(os.path.join(d, f"code24_frame_{i:03d}.bin"), "wb") as fh:
                fh.write(bench.capture())
        meta = write_dac_files(cfg, d, stem=f"w06_amp{int(amp)}")
        states.append(f"{int(amp)}={d}")
        jsons.append(f"{int(amp)}={meta['json']}")
        amps.append(amp)
    return states, jsons


def run_tool(states, jsons, out: str) -> tuple[int, str]:
    argv = ["--out", out, "--min-margin", "6.0"]
    for s in states:
        argv += ["--state", s]
    for j in jsons:
        argv += ["--state-json", j]
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        code = dither_ladder.main(argv)
    return code, buf.getvalue()


def read_table(out_dir: str) -> dict:
    """Read the tool's CSV, not its printed table.

    The printed table is for a human and gained a column between two runs of this test, which
    silently shifted a position-based reader onto the wrong numbers.  The CSV names its columns,
    so this test reads that.  (It also checks the CSV is named correctly -- the first version
    labelled the state column ``amplitude_lsb``.)
    """
    import csv as _csv
    path = os.path.join(out_dir, "dither_ladder.csv")
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(_csv.DictReader(fh))
    if not rows or "state" not in rows[0]:
        raise AssertionError(f"{path} has no 'state' column: {list(rows[0]) if rows else 'empty'}")
    return {r["state"]: {k: (float(v) if v not in ("", None) else float("nan"))
                         for k, v in r.items() if k not in ("state", "json")}
            for r in rows}


def reported(table: dict, state: str) -> dict:
    if state not in table:
        raise AssertionError(f"state {state} missing from {sorted(table)}")
    r = table[state]
    return {
        "rep_a": r.get("rep_pp_a", float("nan")),
        "rep_b": r.get("rep_pp_b", float("nan")),
        "rep_frame": r.get("rep_pp_a_frame", float("nan")),
        "fwhm": r.get("fwhm_a", float("nan")),
        "slope": r.get("slope_a", float("nan")),
        "gain_mag": r.get("gain_mag", float("nan")),
        "dt_phase": r.get("dt_phase", float("nan")),
        "dt_sd": r.get("dt_phase_sd", float("nan")),
    }


def main() -> int:
    checks: list[tuple[str, bool, str]] = []
    root = tempfile.mkdtemp(prefix="dither_ladder_")
    try:
        # ---- a clean ladder: everything must pass ---------------------------
        states, jsons = build(os.path.join(root, "clean"))
        out_clean = os.path.join(root, "out_clean")
        code, text = run_tool(states, jsons, out_clean)
        print(text)
        print("=" * 100)
        checks.append(("the tool passes a linear ladder", code == 0,
                       f"exit code {code}"))

        table = read_table(out_clean)
        lo = reported(table, "16000")
        hi = reported(table, "24000")
        top = reported(table, "30000")
        checks.append(("the measured x1.5 step tracks the command",
                       abs((hi["rep_a"] / lo["rep_a"]) / 1.5 - 1.0) < 0.02,
                       f"rep {lo['rep_a']:.1f} -> {hi['rep_a']:.1f} codes "
                       f"(x{hi['rep_a'] / lo['rep_a']:.4f} against a commanded x1.5)"))
        checks.append(("and so does the x1.875 step",
                       abs((top["rep_a"] / lo["rep_a"]) / 1.875 - 1.0) < 0.02,
                       f"rep {lo['rep_a']:.1f} -> {top['rep_a']:.1f} codes "
                       f"(x{top['rep_a'] / lo['rep_a']:.4f} against a commanded x1.875)"))
        checks.append(("the per-frame timing scatter falls as the replica grows",
                       top["dt_sd"] < lo["dt_sd"],
                       f"{lo['dt_sd']:.1f} ps at {lo['rep_a']:.0f} codes -> "
                       f"{top['dt_sd']:.1f} ps at {top['rep_a']:.0f} codes"))
        checks.append(("the replica is reported as a fraction of ADC full scale",
                       top["rep_a"] > 0 and top["fwhm"] > 0,
                       f"top state: rep {top['rep_a']:.1f} codes pk-pk, "
                       f"FWHM {top['fwhm']:.1f} samples, slope {top['slope']:.1f} "
                       f"codes/sample"))

        # ---- a compressing ladder: the check must fail ----------------------
        states_c, jsons_c = build(os.path.join(root, "compressed"),
                                 compression={24000.0: 0.95, 30000.0: 0.82})
        out_comp = os.path.join(root, "out_comp")
        code_c, text_c = run_tool(states_c, jsons_c, out_comp)
        print(text_c)
        checks.append(("the tool fails a ladder that compresses",
                       code_c != 0, f"exit code {code_c}"))
        table_c = read_table(out_comp)
        comp = reported(table_c, "30000")
        checks.append(("and it locates the compression in the amplitude statement",
                       abs((comp["rep_a"] / reported(table_c, "16000")["rep_a"]) / 1.875 - 1.0)
                       > 0.05,
                       f"30000 measures x"
                       f"{comp['rep_a'] / reported(table_c, '16000')['rep_a']:.3f} "
                       f"against a commanded x1.875"))
    finally:
        shutil.rmtree(root, ignore_errors=True)

    print()
    ok = True
    for name, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name} -- {detail}")
        ok = ok and passed
    print(f"\nLADDER TOOL: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
