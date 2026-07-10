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
    # Plot
    # ------------------------------------------------------------------

    fig, ax = plt.subplots(figsize=(14, 6))
    fig.subplots_adjust(right=0.78)

    ax.plot(adc, lw=0.8)

    ax.grid(True)
    ax.set_xlabel("Sample")
    ax.set_ylabel("ADC Code")

    ax.set_title(
        "Reconstructed ADC Waveform"
        if reconstruct
        else "Raw ADC Samples"
    )

    original_xlim = ax.get_xlim()
    original_ylim = ax.get_ylim()

    controls = (
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
        "Toolbar Pan  : Drag\n"
        "Toolbar Zoom : Box Zoom\n"
        "Home Button  : Reset"
    )

    fig.text(
        0.80,
        0.95,
        controls,
        fontsize=10,
        va="top",
        family="monospace",
        bbox=dict(
            facecolor="whitesmoke",
            edgecolor="gray",
            boxstyle="round,pad=0.5",
        ),
    )

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
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
    )

    if csv_file:
        try:
            plot_adc_csv(csv_file, reconstruct=True)
        except Exception as e:
            messagebox.showerror("Error", str(e))

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
    Receive all seven IFC sweep captures, reconstruct ADC samples,
    calculate DC-offset metrics, save each processed capture, and
    create a summary CSV.

    DC-offset metrics:
        sample_sum:
            Sum of all reconstructed ADC samples.

        mean:
            Signed average ADC code.

        offset_error:
            Absolute DC offset error = abs(mean).

        offset_percent_of_peak:
            Absolute offset as a percentage of measured peak amplitude.

        needs_calibration:
            True when offset_error exceeds offset_threshold_codes.
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
            results.append({
                "index": index,
                "ifc_vpp": float(ifc),
                "sample_count": None,
                "sample_sum": None,
                "mean": None,
                "offset_error": None,
                "offset_percent_of_peak": None,
                "needs_calibration": None,
                "min": None,
                "max": None,
                "peak": None,
                "rms": None,
                "csv_file": "TIMEOUT",
            })
            continue

        try:
            raw = (
                pd.read_csv(csv_file)["byte"]
                .to_numpy(dtype=np.uint8)
            )

            if raw.size < 2:
                raise ValueError(
                    f"Capture for IFC {ifc} Vpp is too short."
                )

            # Ensure an even number of bytes before converting to int16.
            if raw.size % 2 != 0:
                print(
                    f"Warning: odd byte count for IFC {ifc} Vpp. "
                    "Dropping the final byte."
                )
                raw = raw[:-1]

            # Convert little-endian 16-bit words to signed 14-bit samples.
            samples = raw.view("<i2")
            samples = (samples >> 2).astype(np.int16)

            # Remove the known invalid values at the end of the DMA frame.
            if samples.size <= 8:
                raise ValueError(
                    f"Capture for IFC {ifc} Vpp contains too few samples."
                )

            samples = samples[:-8]

            if reconstruct:
                remainder = samples.size % 8

                if remainder != 0:
                    usable_length = samples.size - remainder

                    print(
                        f"Warning: IFC {ifc} Vpp contains "
                        f"{samples.size} words, not divisible by 8. "
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

                adc = np.empty(
                    pos.size + neg.size,
                    dtype=np.int32,
                )

                adc[0::2] = pos
                adc[1::2] = neg

            else:
                adc = samples.astype(np.int32)

            if adc.size == 0:
                raise ValueError(
                    f"No ADC samples available for IFC {ifc} Vpp."
                )

            adc_float = adc.astype(np.float64)

            # ------------------------------------------------------
            # DC-offset statistics
            # ------------------------------------------------------
            sample_count = int(adc.size)
            sample_sum = float(np.sum(adc_float))
            mean_val = float(np.mean(adc_float))

            # Absolute error relative to ideal mean = 0.
            offset_error = float(abs(mean_val))

            min_val = int(np.min(adc))
            max_val = int(np.max(adc))

            peak_val = float(
                (max_val - min_val) / 2.0
            )

            rms_val = float(
                np.sqrt(
                    np.mean(adc_float ** 2)
                )
            )

            offset_percent_of_peak = (
                float(100.0 * offset_error / peak_val)
                if peak_val > 0
                else np.nan
            )

            needs_calibration = bool(
                offset_error > offset_threshold_codes
            )

            # ------------------------------------------------------
            # Save processed ADC capture
            # ------------------------------------------------------
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
                "offset_error": offset_error,
                "offset_percent_of_peak": offset_percent_of_peak,
                "needs_calibration": needs_calibration,
                "min": min_val,
                "max": max_val,
                "peak": peak_val,
                "rms": rms_val,
                "csv_file": str(out_csv),
            })

            calibration_text = (
                "CALIBRATION NEEDED"
                if needs_calibration
                else "PASS"
            )

            print(
                f"IFC {ifc} Vpp processed: "
                f"sum={sample_sum:.3f}, "
                f"mean={mean_val:.6f}, "
                f"offset error={offset_error:.6f}, "
                f"offset={offset_percent_of_peak:.3f}% of peak, "
                f"status={calibration_text}"
            )

        except Exception as exc:
            print(
                f"Processing failed for IFC {ifc} Vpp: {exc}"
            )

            results.append({
                "index": index,
                "ifc_vpp": float(ifc),
                "sample_count": None,
                "sample_sum": None,
                "mean": None,
                "offset_error": None,
                "offset_percent_of_peak": None,
                "needs_calibration": None,
                "min": None,
                "max": None,
                "peak": None,
                "rms": None,
                "csv_file": "PROCESSING_ERROR",
            })

        finally:
            # Remove temporary raw-byte CSV created by receive_adc_data().
            try:
                Path(csv_file).unlink()
            except OSError:
                pass

    summary_df = pd.DataFrame(results)

    summary_file = sweep_dir / "ifc_sweep_summary.csv"
    summary_df.to_csv(summary_file, index=False)

    print("\nIFC Sweep Summary")
    print("=" * 150)
    print(summary_df.to_string(index=False))

    print(f"\nSweep folder:\n{sweep_dir}")
    print(f"\nSummary file:\n{summary_file}")

    return (
        sweep_dir,
        summary_file,
        summary_df,
        saved_files,
    )

def gui_receive_ifc_sweep():
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
        )

        # ----------------------------------------------------------
        # Create summary window
        # ----------------------------------------------------------
        top = tk.Toplevel(root)
        top.title("IFC Sweep Summary")
        top.geometry("1250x560")

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

        summary_text.insert(
            "end",
            "IFC Sweep Summary\n",
        )

        summary_text.insert(
            "end",
            "=" * 130 + "\n",
        )

        summary_text.insert(
            "end",
            summary_df.to_string(index=False),
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

        # ----------------------------------------------------------
        # Keep only valid sweep results
        # ----------------------------------------------------------
        required_metric_columns = [
            "mean",
            "mae",
            "residual_rms",
            "fitted_amplitude",
            "peak",
            "rms",
        ]

        missing_columns = [
            column
            for column in required_metric_columns
            if column not in summary_df.columns
        ]

        if missing_columns:
            raise ValueError(
                "Summary DataFrame is missing columns: "
                + ", ".join(missing_columns)
            )

        valid_results = summary_df.dropna(
            subset=required_metric_columns
        ).copy()

        valid_results = valid_results[
            valid_results["csv_file"].apply(
                lambda value: (
                    isinstance(value, str)
                    and value not in {
                        "TIMEOUT",
                        "PROCESSING_ERROR",
                    }
                    and Path(value).exists()
                )
            )
        ]

        if valid_results.empty:
            messagebox.showwarning(
                "IFC Sweep",
                (
                    "The IFC sweep finished, but no valid captures "
                    "are available for plotting."
                ),
                parent=top,
            )
            return

        # ----------------------------------------------------------
        # Select best capture
        #
        # Best is defined as the smallest residual MAE relative to
        # the fitted sine wave.
        # ----------------------------------------------------------
        best_index = valid_results["mae"].idxmin()
        best_row = valid_results.loc[best_index]

        best_csv = Path(best_row["csv_file"])

        best_ifc = float(best_row["ifc_vpp"])
        best_mean = float(best_row["mean"])
        best_mae = float(best_row["mae"])
        best_residual_rms = float(best_row["residual_rms"])
        best_amplitude = float(best_row["fitted_amplitude"])
        best_offset = float(best_row["fitted_offset"])
        best_peak = float(best_row["peak"])
        best_rms = float(best_row["rms"])

        # ----------------------------------------------------------
        # Best result information
        # ----------------------------------------------------------
        best_result_frame = tk.LabelFrame(
            top,
            text="Best IFC Result",
            padx=10,
            pady=8,
        )

        best_result_frame.pack(
            fill="x",
            padx=10,
            pady=(0, 8),
        )

        best_result_text = (
            f"IFC: {best_ifc:.2f} Vpp    "
            f"Mean: {best_mean:.6f}    "
            f"Residual MAE: {best_mae:.6f}    "
            f"Residual RMS: {best_residual_rms:.6f}\n"
            f"Fitted amplitude: {best_amplitude:.6f}    "
            f"Fitted offset: {best_offset:.6f}    "
            f"Peak: {best_peak:.6f}    "
            f"Waveform RMS: {best_rms:.6f}"
        )

        best_result_label = tk.Label(
            best_result_frame,
            text=best_result_text,
            font=("Arial", 10, "bold"),
            justify="left",
            anchor="w",
        )

        best_result_label.pack(
            fill="x",
        )

        # ----------------------------------------------------------
        # Button handlers
        # ----------------------------------------------------------
        def plot_best_capture():
            try:
                plot_ifc_sweep_capture(
                    best_csv,
                    summary_row=best_row,
                )

            except Exception as exc:
                messagebox.showerror(
                    "Plot Error",
                    str(exc),
                    parent=top,
                )

        def select_and_plot_capture():
            csv_file = filedialog.askopenfilename(
                parent=top,
                initialdir=sweep_dir,
                title="Select an IFC sweep CSV file",
                filetypes=[
                    ("CSV files", "*.csv"),
                    ("All files", "*.*"),
                ],
            )

            if not csv_file:
                return

            try:
                selected_path = str(Path(csv_file))

                matched_rows = summary_df[
                    summary_df["csv_file"] == selected_path
                ]

                selected_summary_row = (
                    matched_rows.iloc[0]
                    if not matched_rows.empty
                    else None
                )

                plot_ifc_sweep_capture(
                    csv_file,
                    summary_row=selected_summary_row,
                )

            except Exception as exc:
                messagebox.showerror(
                    "Plot Error",
                    str(exc),
                    parent=top,
                )

        def open_sweep_folder():
            try:
                import os

                os.startfile(sweep_dir)

            except Exception as exc:
                messagebox.showerror(
                    "Open Folder Error",
                    str(exc),
                    parent=top,
                )

        # ----------------------------------------------------------
        # Buttons
        # ----------------------------------------------------------
        button_frame = tk.Frame(top)

        button_frame.pack(
            pady=(0, 10),
        )

        tk.Button(
            button_frame,
            text="Plot Best Capture",
            width=22,
            command=plot_best_capture,
        ).pack(
            side="left",
            padx=5,
        )

        tk.Button(
            button_frame,
            text="Select Another Capture",
            width=22,
            command=select_and_plot_capture,
        ).pack(
            side="left",
            padx=5,
        )

        tk.Button(
            button_frame,
            text="Open Sweep Folder",
            width=22,
            command=open_sweep_folder,
        ).pack(
            side="left",
            padx=5,
        )

        # Make sure the summary window appears before the dialog.
        top.update_idletasks()

        # ----------------------------------------------------------
        # Ask whether to plot the best result immediately
        # ----------------------------------------------------------
        answer = messagebox.askyesno(
            "Plot Best IFC Capture",
            (
                "The IFC sweep has finished.\n\n"
                "The smallest fitted-sine residual MAE is:\n\n"
                f"IFC: {best_ifc:.2f} Vpp\n"
                f"Mean: {best_mean:.6f}\n"
                f"Residual MAE: {best_mae:.6f}\n"
                f"Residual RMS: {best_residual_rms:.6f}\n"
                f"Fitted amplitude: {best_amplitude:.6f}\n"
                f"Fitted offset: {best_offset:.6f}\n"
                f"Peak: {best_peak:.6f}\n"
                f"Waveform RMS: {best_rms:.6f}\n\n"
                "Plot the measured waveform, fitted sine, "
                "and residual error now?"
            ),
            parent=top,
        )

        if answer:
            plot_best_capture()

    except Exception as exc:
        messagebox.showerror(
            "IFC Sweep Error",
            str(exc),
        )

def plot_ifc_sweep_capture(csv_file, summary_row=None):
    csv_file = Path(csv_file)

    if not csv_file.exists():
        raise FileNotFoundError(f"CSV file not found:\n{csv_file}")

    df = pd.read_csv(csv_file)

    required_columns = {
        "sample_index",
        "adc_code",
        "fitted_sine",
        "residual_error",
    }

    missing = required_columns.difference(df.columns)

    if missing:
        raise ValueError(
            "CSV is missing columns: "
            + ", ".join(sorted(missing))
        )

    sample_index = df["sample_index"].to_numpy()
    adc_code = df["adc_code"].to_numpy()
    fitted_sine = df["fitted_sine"].to_numpy()
    residual = df["residual_error"].to_numpy()

    # Waveform plot
    fig1, ax1 = plt.subplots(figsize=(13, 6))

    ax1.plot(
        sample_index,
        adc_code,
        linewidth=0.8,
        label="Measured ADC",
    )

    ax1.plot(
        sample_index,
        fitted_sine,
        linewidth=1.2,
        label="Fitted sine",
    )

    ax1.set_xlabel("Sample index")
    ax1.set_ylabel("ADC code")
    ax1.grid(True)
    ax1.legend()

    if summary_row is not None:
        ifc_vpp = float(summary_row["ifc_vpp"])
        mean_val = float(summary_row["mean"])
        mae_val = float(summary_row["mae"])
        residual_rms = float(summary_row["residual_rms"])
        fitted_amplitude = float(summary_row["fitted_amplitude"])

        ax1.set_title(
            f"IFC Sweep Capture — {ifc_vpp:.2f} Vpp"
        )

        info = (
            f"IFC: {ifc_vpp:.2f} Vpp\n"
            f"Mean: {mean_val:.6f}\n"
            f"Residual MAE: {mae_val:.6f}\n"
            f"Residual RMS: {residual_rms:.6f}\n"
            f"Fitted amplitude: {fitted_amplitude:.6f}"
        )

        ax1.text(
            0.02,
            0.97,
            info,
            transform=ax1.transAxes,
            va="top",
            family="monospace",
            bbox={
                "boxstyle": "round,pad=0.5",
                "facecolor": "white",
                "edgecolor": "gray",
                "alpha": 0.9,
            },
        )
    else:
        ax1.set_title(csv_file.name)

    fig1.tight_layout()

    # Residual-error plot
    fig2, ax2 = plt.subplots(figsize=(13, 4))

    ax2.plot(
        sample_index,
        residual,
        linewidth=0.8,
    )

    ax2.axhline(
        0,
        linewidth=0.8,
        linestyle="--",
    )

    ax2.set_xlabel("Sample index")
    ax2.set_ylabel("Residual error")
    ax2.set_title("ADC Error Relative to Fitted Sine")
    ax2.grid(True)

    fig2.tight_layout()
    plt.show()

PROJECT_DIR = Path(r"C:\TIDIAC\summer2025_research_tiadc\test_platform\thesis_v3_500mhz_appl")
SAVE_DIR = PROJECT_DIR / "adc_data"
SAVE_DIR.mkdir(parents=True, exist_ok=True)

root = tk.Tk()
root.title("ADC UDP Receiver")
root.geometry("350x220")

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

btn_exit = tk.Button(root, text="Exit", width=25, command=root.destroy)
btn_exit.pack(pady=8)
root.mainloop()