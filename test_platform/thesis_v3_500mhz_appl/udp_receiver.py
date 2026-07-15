import socket
import numpy as np
import pandas as pd
from pathlib import Path
from datetime import datetime
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import filedialog, messagebox

def receive_adc_data(
    bind_ip="0.0.0.0",
    port=6666,
    expected_packets=8,
    packet_size=512,
    timeout=15.0,
):
    save_dir = SAVE_DIR

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_ip, port))
    sock.settimeout(timeout)

    print(f"Listening on {bind_ip}:{port}")
    print(f"Waiting for {expected_packets} packets...")

    packets = []

    try:
        for i in range(expected_packets):
            data, addr = sock.recvfrom(2048)

            print(f"Packet {i+1}/{expected_packets}: {len(data)} bytes from {addr}")

            if len(data) != packet_size:
                print(f"Warning: expected {packet_size} bytes, got {len(data)} bytes")

            packets.append(data)

    except socket.timeout:
        print(f"\nTimeout: only received {len(packets)}/{expected_packets} packets.")
        print("This often happens on the first FPGA UDP run after boot.")
        print("Try running the UDP command again, or add a dummy warm-up transfer.")

        sock.close()
        return None

    sock.close()

    raw = np.frombuffer(b"".join(packets), dtype=np.uint8)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = save_dir / f"adc_capture_{timestamp}.csv"

    pd.DataFrame({"byte": raw}).to_csv(filename, index=False)

    print(f"\nSaved {len(raw)} bytes")
    print(f"File: {filename}")

    return filename

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

    # Convert bytes -> signed 14-bit ADC samples
    samples = raw.view("<i2")
    samples = (samples >> 2).astype(np.int16)

    # Remove trailing garbage
    samples = samples[:-8]

    if reconstruct:
        words = samples.reshape(-1, 8)

        pos = words[:, :4].reshape(-1)
        neg = (-words[:, 4:]).reshape(-1)

        adc = np.empty(pos.size + neg.size, dtype=np.int32)
        adc[0::2] = pos
        adc[1::2] = neg
    else:
        adc = samples

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

def gui_receive():
    try:
        csv_file = receive_adc_data(
            expected_packets=8,
            packet_size=512,
            timeout=5.0,
        )

        if csv_file is None:
            messagebox.showwarning(
                "Timeout",
                "Receive timed out. This often happens on the first FPGA UDP run after boot.\n\n"
                "Run the FPGA udp command again."
            )
        else:
            messagebox.showinfo("Done", f"ADC data saved:\n{csv_file}")

    except Exception as e:
        messagebox.showerror("Error", str(e))

def gui_plot():
    csv_file = filedialog.askopenfilename(
        initialdir=SAVE_DIR,
        title="Select ADC CSV file",
        filetypes=[
            ("CSV files", "*.csv"),
            ("All files", "*.*"),
        ],
    )

    if not csv_file:
        return

    try:
        df = pd.read_csv(csv_file, nrows=5)

        if "byte" in df.columns:
            plot_adc_csv(
                csv_file,
                reconstruct=True,
            )

        elif {
            "sample_index",
            "adc_code",
        }.issubset(df.columns):
            plot_ifc_sweep_capture(csv_file)

        else:
            raise ValueError(
                "Unsupported CSV format.\n\n"
                "Expected either:\n"
                "• a raw capture with a 'byte' column, or\n"
                "• a sweep capture with 'sample_index' and "
                "'adc_code' columns."
            )

    except Exception as exc:
        messagebox.showerror(
            "Plot Error",
            str(exc),
        )

def gui_receive_and_plot():
    try:
        csv_file = receive_adc_data(expected_packets=8, packet_size=512)
        plot_adc_csv(csv_file, reconstruct=True)
    except Exception as e:
        messagebox.showerror("Error", str(e))

def receive_ifc_sweep(
    bind_ip="0.0.0.0",
    port=6666,
    expected_packets=8,
    packet_size=512,
    timeout=15.0,
    reconstruct=True,
    offset_threshold_codes=2.0,
):
    """
    Receive seven IFC sweep captures and calculate DC-offset metrics.

    Metrics:
        sample_sum:
            Sum of all reconstructed ADC samples.

        mean:
            Signed average ADC code.

        absolute_offset:
            Absolute value of the mean.

        pos_mean / neg_mean:
            Mean values of the positive and reconstructed-negative
            branches before interleaving. These help determine whether
            the observed offset is caused by branch asymmetry.

        needs_calibration:
            True when absolute_offset exceeds offset_threshold_codes.
    """

    ifc_values = [
        "2.04",
        "1.93",
        "1.81",
        "1.70",
        "1.59",
        "1.47",
        "1.36",
    ]

    sweep_dir = SAVE_DIR / (
        f"ifc_sweep_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    sweep_dir.mkdir(parents=True, exist_ok=True)

    results = []
    saved_files = []

    def empty_result(index, ifc, status):
        return {
            "index": index,
            "ifc_vpp": float(ifc),
            "sample_count": None,
            "sample_sum": None,
            "mean": None,
            "absolute_offset": None,
            "pos_mean": None,
            "neg_mean": None,
            "branch_mean_difference": None,
            "needs_calibration": None,
            "min": None,
            "max": None,
            "peak": None,
            "rms": None,
            "csv_file": status,
        }

    for index, ifc in enumerate(ifc_values, start=1):
        print(
            f"\nWaiting for sweep frame {index}/{len(ifc_values)}, "
            f"IFC = {ifc} Vpp"
        )

        csv_file = receive_adc_data(
            bind_ip=bind_ip,
            port=port,
            expected_packets=expected_packets,
            packet_size=packet_size,
            timeout=timeout,
        )

        if csv_file is None:
            results.append(
                empty_result(index, ifc, "TIMEOUT")
            )
            continue

        try:
            raw_df = pd.read_csv(csv_file)

            if "byte" not in raw_df.columns:
                raise ValueError(
                    f"Raw capture does not contain a 'byte' column: {csv_file}"
                )

            raw = raw_df["byte"].to_numpy(dtype=np.uint8)

            if raw.size < 2:
                raise ValueError(
                    f"Capture for IFC {ifc} Vpp is too short."
                )

            if raw.size % 2:
                print(
                    f"Warning: IFC {ifc} Vpp has an odd byte count. "
                    "Dropping the last byte."
                )
                raw = raw[:-1]

            # Convert little-endian words to signed 14-bit samples.
            samples = raw.view("<i2")
            samples = (samples >> 2).astype(np.int16)

            # Remove known trailing invalid words.
            if samples.size <= 8:
                raise ValueError(
                    f"Capture for IFC {ifc} Vpp contains too few samples."
                )

            samples = samples[:-8]

            pos_mean = np.nan
            neg_mean = np.nan
            branch_mean_difference = np.nan

            if reconstruct:
                remainder = samples.size % 8

                if remainder:
                    usable_length = samples.size - remainder

                    print(
                        f"Warning: IFC {ifc} Vpp contains "
                        f"{samples.size} words, which is not divisible by 8. "
                        f"Using the first {usable_length} words."
                    )

                    samples = samples[:usable_length]

                if samples.size == 0:
                    raise ValueError(
                        f"No reconstructable samples for IFC {ifc} Vpp."
                    )

                words = samples.reshape(-1, 8)

                pos = (
                    words[:, :4]
                    .reshape(-1)
                    .astype(np.int32)
                )

                neg = (
                    -words[:, 4:]
                    .reshape(-1)
                    .astype(np.int32)
                )

                pos_mean = float(np.mean(pos))
                neg_mean = float(np.mean(neg))

                # Difference between the two reconstructed branches.
                branch_mean_difference = float(pos_mean - neg_mean)

                adc = np.empty(
                    pos.size + neg.size,
                    dtype=np.int32,
                )

                adc[0::2] = pos
                adc[1::2] = neg

                comparison = compare_to_reference(adc)

            else:
                adc = samples.astype(np.int32)

            if adc.size == 0:
                raise ValueError(
                    f"No ADC samples available for IFC {ifc} Vpp."
                )

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

            needs_calibration = bool(
                absolute_offset > offset_threshold_codes
            )

            out_csv = sweep_dir / (
                f"sweep_{index:02d}_"
                f"ifc_{ifc.replace('.', 'p')}_vpp.csv"
            )

            pd.DataFrame({
                "sample_index": np.arange(
                    adc.size,
                    dtype=np.int32,
                ),
                "adc_code": adc,
                "ifc_vpp": float(ifc),
                "sweep_index": index,
            }).to_csv(out_csv, index=False)

            saved_files.append(out_csv)

            results.append({
                "index": index,
                "ifc_vpp": float(ifc),
                "sample_count": sample_count,
                "sample_sum": sample_sum,
                "mean": mean_val,
                "absolute_offset": absolute_offset,
                "pos_mean": pos_mean,
                "neg_mean": neg_mean,
                "branch_mean_difference": branch_mean_difference,
                "needs_calibration": needs_calibration,
                "min": min_val,
                "max": max_val,
                "peak": peak_val,
                "rms": rms_val,
                "csv_file": str(out_csv),
                "rmse": None if comparison is None else comparison["rmse"],
                "max_error": None if comparison is None else comparison["max_error"],
                "mean_error": None if comparison is None else comparison["mean_error"],
                "correlation": None if comparison is None else comparison["correlation"],
                "lag": None if comparison is None else comparison["lag"],
            })

            status_text = (
                "CALIBRATION NEEDED"
                if needs_calibration
                else "PASS"
            )

            print(
                f"IFC {ifc} Vpp: "
                f"sum={sample_sum:.1f}, "
                f"mean={mean_val:.6f}, "
                f"|offset|={absolute_offset:.6f}, "
                f"pos mean={pos_mean:.6f}, "
                f"neg mean={neg_mean:.6f}, "
                f"status={status_text}"
            )

        except Exception as exc:
            print(
                f"Processing failed for IFC {ifc} Vpp: {exc}"
            )

            results.append(
                empty_result(index, ifc, "PROCESSING_ERROR")
            )

        finally:
            # Remove the temporary raw-byte CSV.
            try:
                Path(csv_file).unlink()
            except OSError:
                pass

    summary_df = pd.DataFrame(results)

    summary_file = sweep_dir / "ifc_sweep_summary.csv"
    summary_df.to_csv(summary_file, index=False)

    return (
        sweep_dir,
        summary_file,
        summary_df,
        saved_files,
    )

def gui_receive_ifc_sweep():

    if messagebox.askyesno(
        "Reference",
        "Import a DPG reference TXT for waveform comparison?"
    ):
        load_reference_txt()

    try:
        (
            sweep_dir,
            summary_file,
            summary_df,
            saved_files,
        ) = receive_ifc_sweep(
            expected_packets=8,
            packet_size=512,
            timeout=15.0,
            reconstruct=True,
            offset_threshold_codes=2.0,
        )

        show_ifc_summary_window(
            sweep_dir=sweep_dir,
            summary_df=summary_df,
            summary_file=summary_file,
        )

    except Exception as exc:
        messagebox.showerror(
            "IFC Sweep Error",
            str(exc),
        )

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

def show_ifc_summary_window(
    sweep_dir,
    summary_df,
    summary_file=None,
):
    """
    Display an IFC sweep summary table and provide controls for plotting
    any capture stored in the sweep folder.
    """

    sweep_dir = Path(sweep_dir)

    if summary_file is None:
        summary_file = sweep_dir / "ifc_sweep_summary.csv"
    else:
        summary_file = Path(summary_file)

    top = tk.Toplevel(root)
    top.title(f"IFC Sweep Summary — {sweep_dir.name}")
    top.geometry("1350x620")

    # --------------------------------------------------------------
    # Summary table
    # --------------------------------------------------------------
    text_frame = tk.Frame(top)
    text_frame.pack(
        fill="both",
        expand=True,
        padx=8,
        pady=8,
    )

    summary_text = tk.Text(
        text_frame,
        wrap="none",
        font=("Consolas", 10),
    )

    vertical_scrollbar = tk.Scrollbar(
        text_frame,
        orient="vertical",
        command=summary_text.yview,
    )

    horizontal_scrollbar = tk.Scrollbar(
        text_frame,
        orient="horizontal",
        command=summary_text.xview,
    )

    summary_text.configure(
        yscrollcommand=vertical_scrollbar.set,
        xscrollcommand=horizontal_scrollbar.set,
    )

    summary_text.grid(
        row=0,
        column=0,
        sticky="nsew",
    )

    vertical_scrollbar.grid(
        row=0,
        column=1,
        sticky="ns",
    )

    horizontal_scrollbar.grid(
        row=1,
        column=0,
        sticky="ew",
    )

    text_frame.rowconfigure(0, weight=1)
    text_frame.columnconfigure(0, weight=1)

    # Hide the long CSV path in the visible table.
    visible_columns = [
        column
        for column in summary_df.columns
        if column != "csv_file"
    ]

    visible_df = summary_df[visible_columns].copy()

    summary_text.insert(
        "end",
        "IFC Sweep Summary\n",
    )

    summary_text.insert(
        "end",
        "=" * 150 + "\n",
    )

    summary_text.insert(
        "end",
        visible_df.to_string(index=False),
    )

    summary_text.insert(
        "end",
        f"\n\nSweep folder:\n{sweep_dir}\n",
    )

    summary_text.insert(
        "end",
        f"\nSummary file:\n{summary_file}\n",
    )

    summary_text.configure(state="disabled")

    # --------------------------------------------------------------
    # Find valid capture files
    # --------------------------------------------------------------
    if "csv_file" not in summary_df.columns:
        messagebox.showwarning(
            "Invalid Summary",
            "The summary CSV does not contain a csv_file column.",
            parent=top,
        )
        return

    valid_results = summary_df[
        summary_df["csv_file"].apply(
            lambda value: (
                isinstance(value, str)
                and value not in {
                    "TIMEOUT",
                    "PROCESSING_ERROR",
                }
            )
        )
    ].copy()

    def resolve_capture_path(csv_value):
        """
        Resolve paths saved in the summary.

        This also works when the sweep folder was moved after creation.
        """
        original_path = Path(str(csv_value))

        if original_path.exists():
            return original_path

        local_path = sweep_dir / original_path.name

        if local_path.exists():
            return local_path

        return None

    valid_results["resolved_csv"] = valid_results[
        "csv_file"
    ].apply(resolve_capture_path)

    valid_results = valid_results[
        valid_results["resolved_csv"].notna()
    ]

    if valid_results.empty:
        messagebox.showwarning(
            "No Captures",
            "No valid sweep capture CSV files were found in this folder.",
            parent=top,
        )
        return

    # --------------------------------------------------------------
    # Select lowest absolute offset
    # --------------------------------------------------------------
    best_row = None

    if "absolute_offset" in valid_results.columns:
        offset_rows = valid_results.dropna(
            subset=["absolute_offset"]
        )

        if not offset_rows.empty:
            best_index = offset_rows["absolute_offset"].idxmin()
            best_row = offset_rows.loc[best_index]

    if best_row is not None:
        best_ifc = float(best_row["ifc_vpp"])
        best_mean = float(best_row["mean"])
        best_offset = float(best_row["absolute_offset"])

        status = (
            "CALIBRATION NEEDED"
            if bool(best_row.get("needs_calibration", False))
            else "PASS"
        )

        result_frame = tk.LabelFrame(
            top,
            text="Lowest Absolute Offset",
            padx=10,
            pady=8,
        )

        result_frame.pack(
            fill="x",
            padx=10,
            pady=(0, 8),
        )

        tk.Label(
            result_frame,
            text=(
                f"IFC: {best_ifc:.2f} Vpp    "
                f"Mean: {best_mean:.6f} codes    "
                f"Absolute offset: {best_offset:.6f} codes    "
                f"Status: {status}"
            ),
            font=("Arial", 10, "bold"),
            anchor="w",
        ).pack(fill="x")

    # --------------------------------------------------------------
    # Plot handlers
    # --------------------------------------------------------------
    def find_summary_row(csv_path):
        csv_path = Path(csv_path)

        for _, row in valid_results.iterrows():
            resolved_path = row["resolved_csv"]

            if (
                resolved_path is not None
                and resolved_path.resolve() == csv_path.resolve()
            ):
                return row

        return None

    def plot_lowest_offset():
        if best_row is None:
            messagebox.showwarning(
                "No Offset Result",
                "No valid absolute-offset result is available.",
                parent=top,
            )
            return

        try:
            plot_ifc_sweep_capture(
                best_row["resolved_csv"],
                summary_row=best_row,
            )
        except Exception as exc:
            messagebox.showerror(
                "Plot Error",
                str(exc),
                parent=top,
            )

    def select_and_plot():
        csv_file = filedialog.askopenfilename(
            parent=top,
            initialdir=sweep_dir,
            title="Select IFC sweep capture",
            filetypes=[
                ("CSV files", "*.csv"),
                ("All files", "*.*"),
            ],
        )

        if not csv_file:
            return

        if Path(csv_file).name == "ifc_sweep_summary.csv":
            messagebox.showwarning(
                "Select Capture",
                "Select a sweep capture CSV, not ifc_sweep_summary.csv.",
                parent=top,
            )
            return

        try:
            plot_ifc_sweep_capture(
                csv_file,
                summary_row=find_summary_row(csv_file),
            )
        except Exception as exc:
            messagebox.showerror(
                "Plot Error",
                str(exc),
                parent=top,
            )

    def plot_all():
        try:
            for _, row in valid_results.iterrows():
                plot_ifc_sweep_capture(
                    row["resolved_csv"],
                    summary_row=row,
                )
        except Exception as exc:
            messagebox.showerror(
                "Plot Error",
                str(exc),
                parent=top,
            )

    def open_folder():
        try:
            import os
            os.startfile(str(sweep_dir))
        except Exception as exc:
            messagebox.showerror(
                "Open Folder Error",
                str(exc),
                parent=top,
            )

    # --------------------------------------------------------------
    # Buttons
    # --------------------------------------------------------------
    button_frame = tk.Frame(top)
    button_frame.pack(pady=(0, 10))

    tk.Button(
        button_frame,
        text="Plot Lowest Offset",
        width=20,
        command=plot_lowest_offset,
    ).pack(side="left", padx=5)

    tk.Button(
        button_frame,
        text="Plot Any Capture",
        width=20,
        command=select_and_plot,
    ).pack(side="left", padx=5)

    tk.Button(
        button_frame,
        text="Plot All Captures",
        width=20,
        command=plot_all,
    ).pack(side="left", padx=5)

    tk.Button(
        button_frame,
        text="Open Sweep Folder",
        width=20,
        command=open_folder,
    ).pack(side="left", padx=5)

def gui_open_ifc_sweep_folder():
    """
    Let the user select an existing IFC sweep folder, load its summary
    CSV, and display the summary table.
    """

    selected_folder = filedialog.askdirectory(
        initialdir=SAVE_DIR,
        title="Select IFC Sweep Folder",
    )

    if not selected_folder:
        return

    sweep_dir = Path(selected_folder)
    summary_file = sweep_dir / "ifc_sweep_summary.csv"

    if messagebox.askyesno(
        "Reference",
        "Import a DPG reference TXT for waveform comparison?"
    ):
        load_reference_txt()

    if not summary_file.exists():
        messagebox.showerror(
            "Summary Not Found",
            (
                "The selected folder does not contain:\n\n"
                f"{summary_file.name}"
            ),
        )
        return

    try:
        summary_df = pd.read_csv(summary_file)

        if REFERENCE_SIGNAL is not None:
            for idx, row in summary_df.iterrows():
                csv_path = Path(str(row["csv_file"]))
                if not csv_path.exists():
                    csv_path = sweep_dir / csv_path.name
                if not csv_path.exists():
                    continue
                df = pd.read_csv(csv_path)
                comp = compare_to_reference(df["adc_code"].to_numpy(np.int32))
                if comp is None:
                    continue
                for k,v in comp.items():
                    summary_df.loc[idx,k]=v
            summary_df.to_csv(summary_file,index=False)

        show_ifc_summary_window(
            sweep_dir=sweep_dir,
            summary_df=summary_df,
            summary_file=summary_file,
        )

    except Exception as exc:
        messagebox.showerror(
            "Open Sweep Error",
            str(exc),
        )

REFERENCE_SIGNAL = None

def load_reference_txt():
    global REFERENCE_SIGNAL

    filename = filedialog.askopenfilename(
        title="Select DPG TXT file",
        filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
    )

    if not filename:
        return False

    try:
        ref = np.loadtxt(filename, comments='#', ndmin=1)

        if ref.ndim == 2:
            ref = ref[:, -1]          # last column

        REFERENCE_SIGNAL = ref.astype(np.float64)

        print(f"Loaded reference waveform ({REFERENCE_SIGNAL.size} samples)")
        return True

    except Exception as e:
        messagebox.showerror("Reference Load Error", str(e))
        return False

def compare_to_reference(adc):

    if REFERENCE_SIGNAL is None:
        return None

    ref = REFERENCE_SIGNAL.astype(np.float64).copy()
    adc = adc.astype(np.float64)

    adc -= np.mean(adc)
    ref -= np.mean(ref)

    adc_max = np.max(np.abs(adc))
    ref_max = np.max(np.abs(ref))
    if adc_max == 0 or ref_max == 0:
        return None

    adc /= adc_max
    ref /= ref_max

    corr = np.correlate(adc, ref, mode="full")
    lag = np.argmax(corr) - (len(ref) - 1)

    if lag > 0:
        adc = adc[lag:]
        ref = ref[:len(adc)]
    elif lag < 0:
        ref = ref[-lag:]
        adc = adc[:len(ref)]

    n = min(len(adc), len(ref))
    if n < 10:
        return None

    adc = adc[:n]
    ref = ref[:n]
    error = adc - ref

    correlation = np.corrcoef(adc, ref)[0,1]
    if np.isnan(correlation):
        correlation = 0.0

    return {
        "rmse": float(np.sqrt(np.mean(error**2))),
        "max_error": float(np.max(np.abs(error))),
        "mean_error": float(np.mean(error)),
        "correlation": float(correlation),
        "lag": int(lag),
    }


PROJECT_DIR = Path(r"C:\TIDIAC\summer2025_research_tiadc\test_platform\thesis_v3_500mhz_appl")
SAVE_DIR = PROJECT_DIR / "adc_data"
SAVE_DIR.mkdir(parents=True, exist_ok=True)

root = tk.Tk()
root.title("ADC UDP Receiver")
root.geometry("380x400")

title = tk.Label(root, text="ADC UDP Receiver Tool", font=("Arial", 14, "bold"))
title.pack(pady=15)

btn_receive = tk.Button(root, text="Receive ADC Data", width=25, command=gui_receive)
btn_receive.pack(pady=8)

btn_plot = tk.Button(root, text="Plot CSV File", width=25, command=gui_plot)
btn_plot.pack(pady=8)

btn_both = tk.Button(root, text="Receive + Plot", width=25, command=gui_receive_and_plot)
btn_both.pack(pady=8)

btn_sweep = tk.Button(root, text="Receive IFC Sweep", width=25, command=gui_receive_ifc_sweep)
btn_sweep.pack(pady=8)

btn_open_sweep = tk.Button(root, text="Open Existing IFC Sweep", width=28, command=gui_open_ifc_sweep_folder)
btn_open_sweep.pack(pady=8)

btn_exit = tk.Button(root, text="Exit", width=25, command=root.destroy)
btn_exit.pack(pady=8)
root.mainloop()