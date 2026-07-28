# calibration_loop

Impulse-dither background calibration of gain, offset and timing skew for the
ZCU102 + AD9164 (DPG) + AD9695 test bench.

A short flat-topped impulse with balanced random polarity is summed into the DPG
vector alongside the main tone. Averaging the dither windows with the polarity
weight gives gain and skew; averaging without it gives offset. All three come
from one capture, and the loop pushes the skew correction back into the AD9695
sample-clock delay.

Runs against the existing firmware unchanged.

## Install

```bash
pip install numpy matplotlib pyserial
```

`pyserial` is only needed for the hardware modes.

## Usage

Generate the DPG vector (one signed integer per line, no header):

```bash
python -m calibration_loop.run_calibration gen --out waveforms
```

Run the loop against the built-in bench model, no hardware required:

```bash
python -m calibration_loop.run_calibration sim --iterations 60
```

Check the hardware without driving anything — run this before closing the loop:

```bash
python -m calibration_loop.run_calibration probe --uart COM3 --frames 10 --plot
```

Close the loop on the board:

```bash
python -m calibration_loop.run_calibration bench --uart COM3 --iterations 300
```

Add `--interleaved --skew-target-ps <Ts/2>` once the clock path provides a
half-period offset between the channels. `--help` lists the waveform and loop
parameters, all of which can be overridden on the command line.

Each run writes a CSV log, a JSON metadata file and a learning-curve plot.

## Requirements on the waveform

`DitherConfig.validate()` enforces these; it will tell you which one failed.

- `adc_ratio = fs_dac / fs_adc` must be an integer, and the vector length, the
  dither period, position and pulse geometry must all be multiples of it.
  Otherwise the impulses do not land on a fixed ADC sample phase.
- `sig_cycles` must advance the tone by a useful fraction of a cycle between one
  impulse and the next, so the tone averages out across the events in a capture.
- The pulse must fit inside its slot, with a flat top and edges at least two ADC
  samples each.
- Main tone plus dither must not clip.

## Modules

| File | Contents |
|---|---|
| `dither.py` | DPG vector generation, ADC-rate pulse and derivative templates |
| `estimator.py` | De-framing, loop alignment, joint estimate, block-LMS state |
| `metrics.py` | SNDR, SFDR, ENOB, interleaving spurs, A−B residual |
| `capture.py` | UART-triggered capture, UDP frame collection, clock-delay actuator |
| `simulate.py` | Full-chain bench model with known ground truth |
| `loop.py` | Closed loop, CSV logging, learning curves |
| `run_calibration.py` | CLI: `gen`, `sim`, `probe`, `bench` |

## Notes on the firmware

Two things in the current firmware affect this code:

- `ad9695_adc_super_fine_delay()` in `ad9695_api.c` writes to
  `AD9695_CLK_FINE_DELAY_REG` (0x0112) rather than
  `AD9695_CLK_SUPER_FINE_DELAY_REG` (0x0111), so the 0.25 ps field is never
  programmed. `capture.py` defaults to the 1.725 ps step; pass `--super-fine`
  after fixing it.
- `xemacif_input()` runs after the blocking `uart_get_line()` in `main.c`, so an
  incoming UDP packet is not serviced until the next console line. `capture.py`
  sends a bare newline after any clock-delay write.

A `cal` console command wrapping the existing `adc_capture_frame()` and
`udp_send_mem()` would remove two UART round trips per iteration. Not required.

See `BENCH_GUIDE.md` for the bench procedure.
