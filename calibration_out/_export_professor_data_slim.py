"""Scratch: repackage the raw captures as a data-only deliverable (two files + one page).

The full archive (`professor_data/`) keeps every frame as its own file plus decoded npz;
that is useful internally but reads as clutter when the ask is simply "raw time-domain ADC
outputs".  This builds the minimal package: one binary per actuator state, plus a one-page
format note.

Frames are 4095-byte payloads, which is an odd length -- concatenating them verbatim would
shift every other frame's word alignment.  Each frame is therefore written as a 4096-byte
record with one zero pad byte, so `np.fromfile(...).reshape(n, 2048)` decodes directly.  The
pad byte is exactly the byte the decoder already discarded (255 complete groups use 4080 of
the 4095 bytes).

    python calibration_out/_export_professor_data_slim.py
"""

from __future__ import annotations

import glob
import os
import sys
import zipfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

FULL = os.path.join(REPO, "calibration_out", "professor_data")
OUT = os.path.join(REPO, "calibration_out", "professor_data_min")
PAYLOAD = 4095
RECORD = 4096

README = """Raw ADC captures -- AD9695 on ZCU102, 17 September 2026

  adc_raw_neutral_code24.bin     100 frames, 409600 bytes
  adc_raw_converged_code31.bin    40 frames, 163840 bytes

Two states of the timing actuator, nothing else changed: channel B's sample-clock delay at
control code 24 (uncalibrated, skew residual about -44 ps) and at control code 31 (loop
converged, about -9.7 ps).  Both sets are uncorrected raw captures, in the order taken.

FORMAT
  Each frame is a 4095-byte payload stored as a {record}-byte record, so records stay
  word-aligned: the payload is followed by one zero pad byte.

      int16 little-endian words
      every 8 words = 4 samples of channel A, then 4 samples of channel B
      codes = word >> 2         (14-bit value left-aligned in the int16)
      255 complete groups = 1020 samples per channel per frame

  The DMA restart puts the 8-word group boundary at an arbitrary phase in each frame, so
  every frame needs its own rotation: try all 8 and keep the one that maximises
  |corr(A, B)| -- about 1 at the correct phase, and 0.756 for the half-group-shifted
  candidate (that candidate compares the two converters 4 samples apart).  One branch of
  the splitter is inverted, so negate channel B whenever that correlation is negative,
  before the two are compared or subtracted.

      import numpy as np
      x = np.fromfile('adc_raw_neutral_code24.bin', '<i2').reshape(100, 2048)
      g = x[:, :2040].reshape(100, 255, 8)          # [frame, group, 8 words]
      a = (g[:, :, :4].astype(np.int32) >> 2)       # [frame, group, 4 samples]
      b = (g[:, :, 4:].astype(np.int32) >> 2)

ACQUISITION
  ADC 1.300 GS/s, DAC 2.600 GS/s (ratio exactly 2)
  main tone 199.375 MHz
  impulse dither: one pulse every 130 ADC samples, 64 pulses per 8320-sample loop,
  fixed +/- polarity sequence
  channels A and B are parallel (same sampling instant), not 2x interleaved

CHECK VALUE
  A-B difference spur = 20*log10( |tone(A-B)| / |tone(A)| ) at 199.375 MHz, averaged over
  the frames in each file:  neutral_code24  {spur_a}   converged_code31  {spur_b}
  i.e. a difference of {delta} dB between the two states.
"""


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    spur = {}
    for name, src in (("neutral_code24", "frames_neutral"),
                      ("converged_code31", "dither_vs_tone_frames")):
        files = sorted(glob.glob(os.path.join(REPO, "calibration_out", src, "frame_*.bin")))
        path = os.path.join(OUT, f"adc_raw_{name}.bin")
        with open(path, "wb") as fh:
            for f in files:
                payload = open(f, "rb").read()
                if len(payload) != PAYLOAD:
                    print(f"  ! {f} is {len(payload)} bytes")
                fh.write(payload + b"\x00" * (RECORD - len(payload)))
        spur[name] = len(files)
        print(f"adc_raw_{name}.bin: {len(files)} frames -> {os.path.getsize(path)} bytes")

    # reuse the measured spurs rather than recomputing them here
    summary = open(os.path.join(FULL, "summary.txt"), encoding="utf-8").read()
    vals = {}
    for name in ("neutral_code24", "converged_code31"):
        block = summary.split(f"{name}\n")[1].split("\n")
        vals[name] = next(l.split("spur", 1)[1].strip() for l in block if "spur" in l)
    delta = float(vals["converged_code31"].split()[0]) - float(vals["neutral_code24"].split()[0])

    with open(os.path.join(OUT, "README.txt"), "w", encoding="utf-8", newline="\r\n") as fh:
        fh.write(README.format(record=RECORD, spur_a=vals["neutral_code24"],
                               spur_b=vals["converged_code31"], delta=f"{delta:+.1f}"))

    zip_path = os.path.join(REPO, "calibration_out", "professor_data_min.zip")
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(os.listdir(OUT)):
            z.write(os.path.join(OUT, f), f)
    print(f"\n{zip_path}  {os.path.getsize(zip_path) / 1e6:.2f} MB, "
          f"{len(os.listdir(OUT))} files: {', '.join(sorted(os.listdir(OUT)))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
