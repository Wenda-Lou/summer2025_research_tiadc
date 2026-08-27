from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .frame import reconstruct_adc_bytes


def plot_adc_csv(csv_file, reconstruct=True):
    """
    Read a saved CSV and plot the ADC waveform interactively.

    Keyboard:
        ← / → : Move left/right
        ↑ / ↓ : Zoom in/out Y
        z / x : Zoom in/out X
        r     : Reset view

    Mouse:
        Scroll wheel : Zoom at cursor
        Toolbar Pan  : Left-drag to move
    """

    raw = pd.read_csv(csv_file)["byte"].to_numpy(dtype=np.uint8)

    if reconstruct:
        adc = reconstruct_adc_bytes(raw.tobytes())
    else:
        raw_even = raw[:-1] if raw.size % 2 else raw
        adc = (raw_even.view("<i2") >> 2).astype(np.int32)

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------
    adc_float = adc.astype(np.float64)

    sample_count = int(adc.size)
    sample_sum = float(np.sum(adc_float))
    mean_val = float(np.mean(adc_float))
    absolute_offset = float(abs(mean_val))

    min_val = int(np.min(adc))
    max_val = int(np.max(adc))
    peak_val = float((max_val - min_val) / 2.0)

    rms_val = float(
        np.sqrt(np.mean(adc_float ** 2))
    )

    # ------------------------------------------------------------------
    # Plot
    # ------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(14, 6))
    fig.subplots_adjust(right=0.78)

    ax.plot(
        adc,
        linewidth=0.8,
        label="ADC samples",
    )

    ax.axhline(
        0.0,
        linewidth=1.0,
        linestyle="--",
        label="Ideal mean = 0",
    )

    ax.axhline(
        mean_val,
        linewidth=1.0,
        linestyle=":",
        label=f"Measured mean = {mean_val:.4f}",
    )

    ax.grid(True)
    ax.set_xlabel("Sample")
    ax.set_ylabel("ADC Code")
    ax.legend(loc="upper right")

    ax.set_title(
        "Reconstructed ADC Waveform"
        if reconstruct
        else "Raw ADC Samples"
    )

    statistics = (
        "ADC Statistics\n"
        "────────────────────\n"
        f"Samples : {sample_count}\n"
        f"Sum     : {sample_sum:.1f}\n"
        f"Mean    : {mean_val:.6f}\n"
        f"|Offset|: {absolute_offset:.6f}\n"
        f"Min     : {min_val}\n"
        f"Max     : {max_val}\n"
        f"Peak    : {peak_val:.2f}\n"
        f"RMS     : {rms_val:.6f}\n\n"
        "Keyboard Controls\n"
        "────────────────────\n"
        "←  Move Left\n"
        "→  Move Right\n"
        "↑  Zoom In (Y)\n"
        "↓  Zoom Out (Y)\n"
        "z  Zoom In (X)\n"
        "x  Zoom Out (X)\n"
        "r  Reset View\n\n"
        "Mouse Controls\n"
        "────────────────────\n"
        "Scroll Wheel : Zoom\n"
        "Toolbar Pan  : Drag"
    )

    fig.text(
        0.80,
        0.95,
        statistics,
        fontsize=10,
        va="top",
        family="monospace",
        bbox={
            "facecolor": "whitesmoke",
            "edgecolor": "gray",
            "boxstyle": "round,pad=0.5",
        },
    )

    original_xlim = ax.get_xlim()
    original_ylim = ax.get_ylim()

    # ------------------------------------------------------------------
    # Keyboard
    # ------------------------------------------------------------------

    def on_key(event):
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()

        x_range = xlim[1] - xlim[0]
        y_range = ylim[1] - ylim[0]

        x_shift = x_range * 0.10

        if event.key == "left":
            ax.set_xlim(xlim[0] - x_shift, xlim[1] - x_shift)

        elif event.key == "right":
            ax.set_xlim(xlim[0] + x_shift, xlim[1] + x_shift)

        elif event.key == "up":
            center = (ylim[0] + ylim[1]) / 2
            new = y_range * 0.8
            ax.set_ylim(center - new / 2, center + new / 2)

        elif event.key == "down":
            center = (ylim[0] + ylim[1]) / 2
            new = y_range * 1.25
            ax.set_ylim(center - new / 2, center + new / 2)

        elif event.key == "z":
            center = (xlim[0] + xlim[1]) / 2
            new = x_range * 0.8
            ax.set_xlim(center - new / 2, center + new / 2)

        elif event.key == "x":
            center = (xlim[0] + xlim[1]) / 2
            new = x_range * 1.25
            ax.set_xlim(center - new / 2, center + new / 2)

        elif event.key == "r":
            ax.set_xlim(original_xlim)
            ax.set_ylim(original_ylim)

        fig.canvas.draw_idle()

    # ------------------------------------------------------------------
    # Mouse wheel zoom (zoom at cursor)
    # ------------------------------------------------------------------

    def on_scroll(event):
        if event.inaxes != ax:
            return

        base_scale = 1.2

        if event.button == "up":
            scale = 1 / base_scale
        elif event.button == "down":
            scale = base_scale
        else:
            return

        xlim = ax.get_xlim()
        ylim = ax.get_ylim()

        xdata = event.xdata
        ydata = event.ydata

        new_width = (xlim[1] - xlim[0]) * scale
        new_height = (ylim[1] - ylim[0]) * scale

        relx = (xlim[1] - xdata) / (xlim[1] - xlim[0])
        rely = (ylim[1] - ydata) / (ylim[1] - ylim[0])

        ax.set_xlim(
            xdata - new_width * (1 - relx),
            xdata + new_width * relx,
        )

        ax.set_ylim(
            ydata - new_height * (1 - rely),
            ydata + new_height * rely,
        )

        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("scroll_event", on_scroll)

    plt.show()

    return adc


def plot_ifc_sweep_capture(csv_file, summary_row=None):
    """
    Plot one processed IFC sweep CSV.

    Required columns:
        sample_index
        adc_code
    """

    csv_file = Path(csv_file)

    if not csv_file.exists():
        raise FileNotFoundError(
            f"CSV file not found:\n{csv_file}"
        )

    df = pd.read_csv(csv_file)

    required_columns = {
        "sample_index",
        "adc_code",
    }

    missing = required_columns.difference(df.columns)

    if missing:
        raise ValueError(
            "This is not a processed IFC sweep CSV. "
            "Missing columns: "
            + ", ".join(sorted(missing))
        )

    sample_index = df["sample_index"].to_numpy()
    adc_code = df["adc_code"].to_numpy(dtype=np.float64)

    measured_mean = float(np.mean(adc_code))
    sample_sum = float(np.sum(adc_code))
    absolute_offset = float(abs(measured_mean))

    fig, ax = plt.subplots(figsize=(14, 6))
    fig.subplots_adjust(right=0.78)

    ax.plot(
        sample_index,
        adc_code,
        linewidth=0.8,
        label="ADC samples",
    )

    ax.axhline(
        0.0,
        linewidth=1.0,
        linestyle="--",
        label="Ideal mean = 0",
    )

    ax.axhline(
        measured_mean,
        linewidth=1.0,
        linestyle=":",
        label=f"Measured mean = {measured_mean:.4f}",
    )

    ax.set_xlabel("Sample index")
    ax.set_ylabel("ADC code")
    ax.grid(True)
    ax.legend(loc="upper right")

    if summary_row is not None:
        ifc_vpp = float(summary_row["ifc_vpp"])
        peak_val = float(summary_row["peak"])
        rms_val = float(summary_row["rms"])

        needs_calibration = bool(
            summary_row["needs_calibration"]
        )

        status_text = (
            "CALIBRATION NEEDED"
            if needs_calibration
            else "PASS"
        )

        title = f"IFC Sweep Capture — {ifc_vpp:.2f} Vpp"

        info = (
            f"IFC: {ifc_vpp:.2f} Vpp\n"
            f"Samples: {int(summary_row['sample_count'])}\n"
            f"Sum: {float(summary_row['sample_sum']):.1f}\n"
            f"Mean: {float(summary_row['mean']):.6f}\n"
            f"|Offset|: "
            f"{float(summary_row['absolute_offset']):.6f}\n"
            f"Positive mean: "
            f"{float(summary_row['pos_mean']):.6f}\n"
            f"Negative mean: "
            f"{float(summary_row['neg_mean']):.6f}\n"
            f"Peak: {peak_val:.2f}\n"
            f"RMS: {rms_val:.6f}\n"
            f"Status: {status_text}"
        )

    else:
        title = csv_file.name

        info = (
            f"Samples: {adc_code.size}\n"
            f"Sum: {sample_sum:.1f}\n"
            f"Mean: {measured_mean:.6f}\n"
            f"|Offset|: {absolute_offset:.6f}\n"
            f"Min: {np.min(adc_code):.0f}\n"
            f"Max: {np.max(adc_code):.0f}"
        )

    ax.set_title(title)

    fig.text(
        0.80,
        0.95,
        info,
        fontsize=10,
        va="top",
        family="monospace",
        bbox={
            "facecolor": "whitesmoke",
            "edgecolor": "gray",
            "boxstyle": "round,pad=0.5",
        },
    )

    original_xlim = ax.get_xlim()
    original_ylim = ax.get_ylim()

    def on_key(event):
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()

        x_range = xlim[1] - xlim[0]
        y_range = ylim[1] - ylim[0]

        if event.key == "left":
            shift = x_range * 0.10
            ax.set_xlim(xlim[0] - shift, xlim[1] - shift)

        elif event.key == "right":
            shift = x_range * 0.10
            ax.set_xlim(xlim[0] + shift, xlim[1] + shift)

        elif event.key == "up":
            center = sum(ylim) / 2
            new_range = y_range * 0.8
            ax.set_ylim(
                center - new_range / 2,
                center + new_range / 2,
            )

        elif event.key == "down":
            center = sum(ylim) / 2
            new_range = y_range * 1.25
            ax.set_ylim(
                center - new_range / 2,
                center + new_range / 2,
            )

        elif event.key == "z":
            center = sum(xlim) / 2
            new_range = x_range * 0.8
            ax.set_xlim(
                center - new_range / 2,
                center + new_range / 2,
            )

        elif event.key == "x":
            center = sum(xlim) / 2
            new_range = x_range * 1.25
            ax.set_xlim(
                center - new_range / 2,
                center + new_range / 2,
            )

        elif event.key == "r":
            ax.set_xlim(original_xlim)
            ax.set_ylim(original_ylim)

        fig.canvas.draw_idle()

    def on_scroll(event):
        if event.inaxes != ax:
            return

        scale = 1 / 1.2 if event.button == "up" else 1.2

        xlim = ax.get_xlim()
        ylim = ax.get_ylim()

        xdata = event.xdata
        ydata = event.ydata

        new_width = (xlim[1] - xlim[0]) * scale
        new_height = (ylim[1] - ylim[0]) * scale

        relx = (xlim[1] - xdata) / (xlim[1] - xlim[0])
        rely = (ylim[1] - ydata) / (ylim[1] - ylim[0])

        ax.set_xlim(
            xdata - new_width * (1 - relx),
            xdata + new_width * relx,
        )

        ax.set_ylim(
            ydata - new_height * (1 - rely),
            ydata + new_height * rely,
        )

        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("scroll_event", on_scroll)

    plt.show()

