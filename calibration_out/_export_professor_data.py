"""Scratch: package the raw time-domain captures for the professor.

Copies the two comparable frame sets (neutral code 24 and converged code 31) into
`calibration_out/professor_data/raw/`, decodes them with the standalone decoder that
ships alongside, writes an NPZ per set, and writes the README that documents the frame
format, the acquisition setup and the numbers he can recompute from the raw bytes.

    python calibration_out/_export_professor_data.py
"""

from __future__ import annotations

import glob
import os
import shutil
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

OUT = os.path.join(REPO, "calibration_out", "professor_data")
sys.path.insert(0, OUT)

from decode_frames import (ab_difference_spur, decode_frame, _corr,  # noqa: E402
                           FS_ADC, F0, FRAME_BYTES)

SETS = [
    ("neutral_code24", os.path.join(REPO, "calibration_out", "frames_neutral"),
     "delay actuator at control code 24 (neutral); skew residual about -44 ps"),
    ("converged_code31", os.path.join(REPO, "calibration_out", "dither_vs_tone_frames"),
     "delay actuator at control code 31 (loop converged); skew residual about -9.7 ps"),
]

README = """# Raw time-domain ADC captures — AD9695 on ZCU102

Two sets of raw frames, taken on the bench on 17 September 2026. They are the same
measurement at two states of the timing actuator, so the improvement reported in the
update can be recomputed from the raw bytes without trusting any of our analysis code.

| set | frames | state | A-B difference spur |
|---|---|---|---|
| `raw/neutral_code24/` | {n_a} | delay control code 24 (neutral), skew residual ~-44 ps | **{spur_a}** |
| `raw/converged_code31/` | {n_b} | delay control code 31 (converged loop), skew residual ~-9.7 ps | **{spur_b}** |

The spur is the amplitude of the main tone in the channel difference `A-B` relative to the
tone amplitude in A, i.e. `20*log10(|tone(A-B)| / |tone(A)|)` in dBc. It is computed from
these very files by `decode_frames.py`, which uses nothing but numpy. **Difference between
the two states: {delta} dB.** The residual skews above are what the analytic relation
`20*log10|2 sin(pi f dt)|` predicts for those two states (-25.2 and -38.4 dBc), which is the
point of the comparison: the measured improvement tracks the skew, not the calibration code.

## Frame format

One frame is the UDP payload the board sends after `dma -w`: **{frame_bytes} bytes**, little-endian
signed 16-bit words.

    words : int16[2047] = [A0 A1 A2 A3 B0 B1 B2 B3] [A4 A5 A6 A7 B4 ... ] ...
    codes : sample >> 2      (the 14-bit converter value is left-aligned in the int16)

255 complete groups of 8 words (2040 words, 4080 bytes) are used; the trailing 7 words and
the odd final byte are ignored. The DMA restart places the 8-word group boundary at an
arbitrary phase, so each frame needs its own rotation: try all 8 and keep the one that
maximises `|corr(A, B)|`. That is the physical criterion — at the correct phase the two
converters sample the same instant and the streams are proportional (`|corr| ~ 1`), while
the half-group-shifted candidate compares them 4 samples apart and scales the correlation
by `cos(2*pi*4*f0) = 0.756`.

`corr(A, B) ~ -1` before normalisation: one branch of the splitter is inverted, so the
aligned pair comes out anti-proportional. The decoder negates channel B in that case to
restore one common sense before anything is subtracted — the sign is stable, unlike a DC
level. (Verify by correlating `-ch_b` against `ch_a`.) Subtracting without this step leaves
`A - B` at twice the tone amplitude, i.e. about +6 dBc, which is what a first version of
this decoder produced until the inversion was handled.

## Acquisition configuration

| quantity | value |
|---|---|
| ADC sample rate | {fs_adc_msps:.3f} MS/s |
| DAC update rate (AD9164 DPG) | 2600 MS/s (ratio 2, so the dither geometry is exact) |
| main tone | 199.375 MHz ({sig_cycles} cycles per {loop_samples}-sample ADC loop) |
| impulse repetition | one pulse every 130 ADC samples, 64 pulses per loop |
| impulse polarity | fixed +/- sequence, balanced over the 64 events |
| channels | A and B are **parallel** (same sampling instant), not 2x interleaved |
| samples per frame | 1020 per channel after de-framing |

## Files

- `raw/<set>/frame_XXX.bin` — the frames exactly as received, 4095 bytes each.
- `decoded/<set>.npz` — `ch_a`, `ch_b` [codes, float64, shape (frames, 1020)], `rotation`.
- `decode_frames.py` — standalone decoder + the spur computation (numpy only):

      python decode_frames.py raw/neutral_code24
      python decode_frames.py raw/converged_code31 --csv converged_first_frame.csv

## What these sets are not

They are not interleaved-ADC captures (both channels sample the same instant, so there is
no fs/2 image to look at), and the correction in the closed loop is applied host-side — the
raw frames here are uncorrected. `raw/` holds only the two comparable states; earlier
pre-neutralisation captures exist in the working tree but are from a different actuator
state and are not part of this package.
"""


def main() -> int:
    os.makedirs(os.path.join(OUT, "raw"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "decoded"), exist_ok=True)

    summary = []
    for name, src, note in SETS:
        files = sorted(glob.glob(os.path.join(src, "frame_*.bin")))
        if not files:
            print(f"missing frames for {name} in {src}")
            return 1
        dst = os.path.join(OUT, "raw", name)
        os.makedirs(dst, exist_ok=True)
        ch_a, ch_b, rots, spurs, corrs = [], [], [], [], []
        for i, path in enumerate(files):
            raw = open(path, "rb").read()
            if len(raw) != FRAME_BYTES:
                print(f"  ! {path} is {len(raw)} bytes, expected {FRAME_BYTES}")
            a, b, r = decode_frame(raw)
            ch_a.append(a)
            ch_b.append(b)
            rots.append(r)
            spurs.append(ab_difference_spur(a, b))
            corrs.append(_corr(a, b))
            shutil.copyfile(path, os.path.join(dst, f"frame_{i:03d}.bin"))
        np.savez_compressed(os.path.join(OUT, "decoded", f"{name}.npz"),
                            ch_a=np.array(ch_a), ch_b=np.array(ch_b),
                            rotation=np.array(rots))
        sp = np.array(spurs)
        summary.append((name, len(files), sp, np.array(corrs), note))
        print(f"{name}: {len(files)} frames -> decoded/{name}.npz   "
              f"spur {sp.mean():+.2f} +- {sp.std(ddof=1):.2f} dBc   "
              f"corr(A,B) {np.mean(corrs):+.3f}")

    (a_name, a_n, a_sp, a_c, a_note), (b_name, b_n, b_sp, b_c, b_note) = summary
    with open(os.path.join(OUT, "README.md"), "w", encoding="utf-8") as fh:
        fh.write(README.format(
            n_a=a_n, n_b=b_n, frame_bytes=FRAME_BYTES,
            spur_a=f"{a_sp.mean():+.2f} +- {a_sp.std(ddof=1):.2f} dBc",
            spur_b=f"{b_sp.mean():+.2f} +- {b_sp.std(ddof=1):.2f} dBc",
            delta=f"{b_sp.mean() - a_sp.mean():+.2f}",
            fs_adc_msps=FS_ADC / 1e6, sig_cycles=1276, loop_samples=8320))

    with open(os.path.join(OUT, "summary.txt"), "w", encoding="utf-8") as fh:
        fh.write("A-B difference spur computed from the raw frames in raw/\n")
        fh.write("(20*log10 of the tone amplitude in (A-B) over the tone amplitude in A)\n\n")
        for name, n, sp, c, note in summary:
            fh.write(f"{name}\n  frames          {n}\n  spur            "
                     f"{sp.mean():+.3f} +- {sp.std(ddof=1):.3f} dBc "
                     f"(median {np.median(sp):+.3f})\n  corr(A,B)       "
                     f"{c.mean():+.4f}\n  state           {note}\n\n")
        fh.write(f"difference between the two states: "
                 f"{b_sp.mean() - a_sp.mean():+.2f} dB\n")

    total = sum(os.path.getsize(f) for f in glob.glob(os.path.join(OUT, "**", "*"),
                                                      recursive=True))
    print(f"\nwrote {OUT}  ({total / 1e6:.2f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
