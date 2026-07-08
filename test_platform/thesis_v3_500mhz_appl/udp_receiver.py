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
):
    ifc_values = ["2.04", "1.93", "1.81", "1.70", "1.59", "1.47", "1.36"]

    sweep_dir = SAVE_DIR / f"ifc_sweep_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    sweep_dir.mkdir(parents=True, exist_ok=True)

    results = []
    saved_files = []

    for index, ifc in enumerate(ifc_values, start=1):
        print(f"\nWaiting for sweep frame {index}/7, IFC = {ifc} Vpp")

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
                "ifc_vpp": ifc,
                "mean_error": None,
                "mean": None,
                "min": None,
                "max": None,
                "peak": None,
                "rms": None,
                "csv_file": "TIMEOUT",
            })
            continue

        raw = pd.read_csv(csv_file)["byte"].to_numpy(dtype=np.uint8)

        samples = raw.view("<i2")
        samples = (samples >> 2).astype(np.int16)
        samples = samples[:-8]

        if reconstruct:
            words = samples.reshape(-1, 8)

            pos = words[:, :4].reshape(-1)
            neg = (-words[:, 4:]).reshape(-1)

            adc = np.empty(pos.size + neg.size, dtype=np.int32)
            adc[0::2] = pos
            adc[1::2] = neg
        else:
            adc = samples.astype(np.int32)

        mean_val = float(np.mean(adc))
        mean_error = mean_val          # target mean is 0
        min_val = int(np.min(adc))
        max_val = int(np.max(adc))
        peak_val = float((max_val - min_val) / 2)
        rms_val = float(np.sqrt(np.mean(adc.astype(float) ** 2)))

        out_csv = sweep_dir / f"sweep_{index:02d}_ifc_{ifc.replace('.', 'p')}_vpp.csv"

        pd.DataFrame({
            "sample_index": np.arange(len(adc)),
            "adc_code": adc,
            "ifc_vpp": ifc,
            "sweep_index": index,
        }).to_csv(out_csv, index=False)

        saved_files.append(out_csv)

        results.append({
            "index": index,
            "ifc_vpp": ifc,
            "mean_error": mean_error,
            "mean": mean_val,
            "min": min_val,
            "max": max_val,
            "peak": peak_val,
            "rms": rms_val,
            "csv_file": str(out_csv),
        })

        # remove temporary raw byte csv created by receive_adc_data()
        try:
            Path(csv_file).unlink()
        except Exception:
            pass

    summary_df = pd.DataFrame(results)
    summary_file = sweep_dir / "ifc_sweep_summary.csv"
    summary_df.to_csv(summary_file, index=False)

    print("\nIFC Sweep Summary")
    print(summary_df.to_string(index=False))

    print(f"\nSweep folder: {sweep_dir}")
    print(f"Summary file: {summary_file}")

    return sweep_dir, summary_file, summary_df, saved_files

def gui_receive_ifc_sweep():
    try:
        sweep_dir, summary_file, summary_df, saved_files = receive_ifc_sweep(
            expected_packets=8,
            packet_size=512,
            timeout=15.0,
        )

        top = tk.Toplevel(root)
        top.title("IFC Sweep Summary")
        top.geometry("900x360")

        text = tk.Text(top, wrap="none", font=("Consolas", 10))
        text.pack(fill="both", expand=True)

        text.insert("end", "IFC Sweep Summary\n")
        text.insert("end", "=" * 80 + "\n")
        text.insert("end", summary_df.to_string(index=False))
        text.insert("end", "\n\n")
        text.insert("end", f"Sweep folder:\n{sweep_dir}\n\n")
        text.insert("end", f"Summary file:\n{summary_file}\n")

        def ask_plot():
            answer = messagebox.askyesno(
                "Plot Sweep Result",
                "Do you want to plot one of the sweep CSV files?"
            )

            if not answer:
                return

            csv_file = filedialog.askopenfilename(
                initialdir=sweep_dir,
                title="Select one sweep CSV file to plot",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
            )

            if csv_file:
                df = pd.read_csv(csv_file)

                fig, ax = plt.subplots(figsize=(12, 5))
                ax.plot(df["sample_index"], df["adc_code"], lw=0.8)
                ax.grid(True)
                ax.set_xlabel("Sample")
                ax.set_ylabel("ADC Code")
                ax.set_title(Path(csv_file).name)
                plt.show()

        btn_plot_one = tk.Button(
            top,
            text="Plot One Sweep CSV",
            command=ask_plot,
        )
        btn_plot_one.pack(pady=8)

    except Exception as e:
        messagebox.showerror("Error", str(e))

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