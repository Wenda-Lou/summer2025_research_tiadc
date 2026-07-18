from pathlib import Path

import pandas as pd
import tkinter as tk
from tkinter import filedialog, messagebox

from .plot import plot_adc_csv, plot_ifc_sweep_capture
from .receive_data import SAVE_DIR, receive_adc_data, receive_timing_captures
from .sweep_test import receive_ifc_sweep
from .reference_upload import (
    FPGA_IP,
    FPGA_PORT,
    REFERENCE_SAMPLE_COUNT,
    clear_reference,
    load_reference_txt,
    send_reference,
)

root = None


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
        csv_file = receive_adc_data(
            expected_packets=8,
            packet_size=512,
        )

        if csv_file is None:
            messagebox.showwarning(
                "Timeout",
                "No complete ADC capture was received.",
            )
            return

        plot_adc_csv(csv_file, reconstruct=True)

    except Exception as exc:
        messagebox.showerror("Error", str(exc))


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



def gui_receive_timing_captures():
    """Run the frame-to-frame timing repeatability receiver."""
    dialog = tk.Toplevel(root)
    dialog.title("ADC Timing Repeatability Test")
    dialog.geometry("520x260")
    dialog.transient(root)
    dialog.grab_set()

    tk.Label(
        dialog,
        text=(
            "Start this receiver first, then run on UART:\n"
            "adc -timing <frame count>\n\n"
            "This test aligns every capture to frame 1.\n"
            "A DAC TXT file is not required."
        ),
        justify="center",
    ).pack(pady=12)

    form = tk.Frame(dialog)
    form.pack(pady=5)

    tk.Label(form, text="Frame count:").grid(
        row=0, column=0, padx=5, pady=5, sticky="e"
    )
    frame_entry = tk.Entry(form, width=12)
    frame_entry.insert(0, "20")
    frame_entry.grid(row=0, column=1, padx=5, pady=5)

    tk.Label(form, text="Timeout per packet (s):").grid(
        row=1, column=0, padx=5, pady=5, sticky="e"
    )
    timeout_entry = tk.Entry(form, width=12)
    timeout_entry.insert(0, "20")
    timeout_entry.grid(row=1, column=1, padx=5, pady=5)

    def start_receive():
        try:
            frame_count = int(frame_entry.get())
            timeout = float(timeout_entry.get())
            if frame_count <= 0:
                raise ValueError("Frame count must be positive.")
            if timeout <= 0:
                raise ValueError("Timeout must be positive.")

            dialog.destroy()
            timing_dir, summary_file, summary_df = receive_timing_captures(
                frame_count=frame_count,
                timeout=timeout,
            )
            messagebox.showinfo(
                "Timing Test Complete",
                f"Processed {len(summary_df)} complete frames.\n\n"
                f"Results folder:\n{timing_dir}\n\n"
                f"Summary:\n{summary_file}",
            )
        except Exception as exc:
            messagebox.showerror("Timing Test Error", str(exc))

    tk.Button(
        dialog,
        text="Start Receiver",
        width=18,
        command=start_receive,
    ).pack(pady=12)


def gui_upload_reference_txt():
    """
    Select a DAC/reference TXT file, extract exactly 2032 samples,
    convert them to signed int16, and upload them to the FPGA.
    """
    txt_file = filedialog.askopenfilename(
        parent=root,
        initialdir=SAVE_DIR,
        title="Select DAC Reference TXT File",
        filetypes=[
            ("Text files", "*.txt"),
            ("Data files", "*.dat"),
            ("All files", "*.*"),
        ],
    )

    if not txt_file:
        return

    try:
        all_samples = load_reference_txt(txt_file)

        if all_samples.size < REFERENCE_SAMPLE_COUNT:
            raise ValueError(
                f"The selected file contains only {all_samples.size} samples.\n\n"
                f"At least {REFERENCE_SAMPLE_COUNT} samples are required."
            )

    except Exception as exc:
        messagebox.showerror(
            "Reference File Error",
            str(exc),
            parent=root,
        )
        return

    dialog = tk.Toplevel(root)
    dialog.title("Upload FPGA Reference")
    dialog.geometry("620x500")
    dialog.transient(root)
    dialog.grab_set()

    max_start = all_samples.size - REFERENCE_SAMPLE_COUNT
    selected_path = Path(txt_file)

    tk.Label(
        dialog,
        text="FPGA Reference Upload",
        font=("Arial", 13, "bold"),
    ).pack(pady=(14, 8))

    file_frame = tk.LabelFrame(
        dialog,
        text="Selected File",
        padx=10,
        pady=8,
    )
    file_frame.pack(fill="x", padx=12, pady=6)

    tk.Label(
        file_frame,
        text=(
            f"File: {selected_path.name}\n"
            f"Total numeric samples: {all_samples.size}\n"
            f"Samples sent to FPGA: {REFERENCE_SAMPLE_COUNT}"
        ),
        justify="left",
        anchor="w",
    ).pack(fill="x")

    selection_frame = tk.LabelFrame(
        dialog,
        text="Sample Selection",
        padx=10,
        pady=8,
    )
    selection_frame.pack(fill="x", padx=12, pady=6)

    tk.Label(
        selection_frame,
        text="Start sample index:",
    ).grid(
        row=0,
        column=0,
        padx=5,
        pady=5,
        sticky="e",
    )

    start_entry = tk.Entry(selection_frame, width=16)
    start_entry.insert(0, "0")
    start_entry.grid(
        row=0,
        column=1,
        padx=5,
        pady=5,
        sticky="w",
    )

    tk.Label(
        selection_frame,
        text=f"Allowed range: 0 to {max_start}",
    ).grid(
        row=1,
        column=0,
        columnspan=3,
        padx=5,
        pady=2,
    )

    preview_var = tk.StringVar()

    def get_selected_samples():
        try:
            start_index = int(start_entry.get().strip())
        except ValueError as exc:
            raise ValueError("Start sample index must be an integer.") from exc

        if not 0 <= start_index <= max_start:
            raise ValueError(
                f"Start sample index must be between 0 and {max_start}."
            )

        end_index = start_index + REFERENCE_SAMPLE_COUNT
        selected = all_samples[start_index:end_index].copy()

        if selected.size != REFERENCE_SAMPLE_COUNT:
            raise ValueError(
                f"Expected {REFERENCE_SAMPLE_COUNT} selected samples, "
                f"received {selected.size}."
            )

        return start_index, selected

    def update_preview():
        try:
            start_index, selected = get_selected_samples()
            end_index = start_index + REFERENCE_SAMPLE_COUNT - 1

            preview_var.set(
                f"Selected range: {start_index} to {end_index}\n"
                f"Minimum: {int(selected.min())}\n"
                f"Maximum: {int(selected.max())}\n"
                f"First sample: {int(selected[0])}\n"
                f"Last sample: {int(selected[-1])}"
            )
        except Exception as exc:
            preview_var.set(str(exc))

    tk.Button(
        selection_frame,
        text="Update Preview",
        command=update_preview,
    ).grid(
        row=0,
        column=2,
        padx=8,
        pady=5,
    )

    tk.Label(
        selection_frame,
        textvariable=preview_var,
        justify="left",
        anchor="w",
    ).grid(
        row=2,
        column=0,
        columnspan=3,
        sticky="w",
        padx=5,
        pady=8,
    )

    connection_frame = tk.LabelFrame(
        dialog,
        text="FPGA Connection",
        padx=10,
        pady=8,
    )
    connection_frame.pack(fill="x", padx=12, pady=6)

    tk.Label(connection_frame, text="FPGA IP:").grid(
        row=0,
        column=0,
        padx=5,
        pady=5,
        sticky="e",
    )
    ip_entry = tk.Entry(connection_frame, width=20)
    ip_entry.insert(0, FPGA_IP)
    ip_entry.grid(row=0, column=1, padx=5, pady=5)

    tk.Label(connection_frame, text="UDP port:").grid(
        row=1,
        column=0,
        padx=5,
        pady=5,
        sticky="e",
    )
    port_entry = tk.Entry(connection_frame, width=20)
    port_entry.insert(0, str(FPGA_PORT))
    port_entry.grid(row=1, column=1, padx=5, pady=5)

    tk.Label(connection_frame, text="Packet delay (s):").grid(
        row=2,
        column=0,
        padx=5,
        pady=5,
        sticky="e",
    )
    delay_entry = tk.Entry(connection_frame, width=20)
    delay_entry.insert(0, "0.01")
    delay_entry.grid(row=2, column=1, padx=5, pady=5)

    status_var = tk.StringVar(
        value="Ready. Confirm the result using the FPGA UART output."
    )

    tk.Label(
        dialog,
        textvariable=status_var,
        justify="left",
        anchor="w",
        wraplength=580,
    ).pack(fill="x", padx=16, pady=10)

    button_frame = tk.Frame(dialog)
    button_frame.pack(pady=8)

    def upload_reference():
        try:
            start_index, selected = get_selected_samples()
            fpga_ip = ip_entry.get().strip()
            fpga_port = int(port_entry.get().strip())
            packet_delay = float(delay_entry.get().strip())

            if not fpga_ip:
                raise ValueError("FPGA IP address cannot be empty.")
            if not 1 <= fpga_port <= 65535:
                raise ValueError("UDP port must be between 1 and 65535.")
            if packet_delay < 0:
                raise ValueError("Packet delay cannot be negative.")

            upload_button.configure(state="disabled")
            status_var.set("Sending reference packets...")
            dialog.update_idletasks()

            sent_count = send_reference(
                selected,
                fpga_ip=fpga_ip,
                fpga_port=fpga_port,
                packet_delay_s=packet_delay,
                require_full_buffer=True,
            )

            end_index = start_index + REFERENCE_SAMPLE_COUNT - 1

            status_var.set(
                f"Sent {sent_count} samples. "
                "Check UART for 'Reference uploaded successfully'."
            )

            messagebox.showinfo(
                "Reference Sent",
                (
                    f"Sent {sent_count} samples to "
                    f"{fpga_ip}:{fpga_port}.\n\n"
                    f"Source range: {start_index} to {end_index}\n"
                    f"First sample: {int(selected[0])}\n"
                    f"Last sample: {int(selected[-1])}\n\n"
                    "Confirm the upload result on the FPGA UART."
                ),
                parent=dialog,
            )

        except Exception as exc:
            status_var.set("Reference upload failed.")
            messagebox.showerror(
                "Reference Upload Error",
                str(exc),
                parent=dialog,
            )

        finally:
            upload_button.configure(state="normal")

    def clear_board_reference():
        try:
            fpga_ip = ip_entry.get().strip()
            fpga_port = int(port_entry.get().strip())

            clear_reference(
                fpga_ip=fpga_ip,
                fpga_port=fpga_port,
            )

            status_var.set(
                "Reference-clear command sent. Confirm it on the FPGA UART."
            )

        except Exception as exc:
            messagebox.showerror(
                "Reference Clear Error",
                str(exc),
                parent=dialog,
            )

    upload_button = tk.Button(
        button_frame,
        text="Send Reference",
        width=18,
        command=upload_reference,
    )
    upload_button.pack(side="left", padx=5)

    tk.Button(
        button_frame,
        text="Clear Reference",
        width=18,
        command=clear_board_reference,
    ).pack(side="left", padx=5)

    tk.Button(
        button_frame,
        text="Close",
        width=12,
        command=dialog.destroy,
    ).pack(side="left", padx=5)

    update_preview()


def create_root():
    global root
    root = tk.Tk()
    root.title('ADC UDP Receiver')
    root.geometry('420x540')

    title = tk.Label(root, text='ADC UDP Receiver Tool', font=('Arial', 14, 'bold'))
    title.pack(pady=15)

    btn_receive = tk.Button(root, text='Receive ADC Data', width=25, command=gui_receive)
    btn_receive.pack(pady=8)

    btn_plot = tk.Button(root, text='Plot CSV File', width=25, command=gui_plot)
    btn_plot.pack(pady=8)

    btn_both = tk.Button(root, text='Receive + Plot', width=25, command=gui_receive_and_plot)
    btn_both.pack(pady=8)

    btn_sweep = tk.Button(root, text='Receive IFC Sweep', width=25, command=gui_receive_ifc_sweep)
    btn_sweep.pack(pady=8)

    btn_open_sweep = tk.Button(root, text='Open Existing IFC Sweep', width=28, command=gui_open_ifc_sweep_folder)
    btn_open_sweep.pack(pady=8)

    btn_timing = tk.Button(
        root,
        text='Receive Timing Captures',
        width=28,
        command=gui_receive_timing_captures,
    )
    btn_timing.pack(pady=8)

    btn_reference = tk.Button(
        root,
        text='Upload Reference TXT',
        width=28,
        command=gui_upload_reference_txt,
    )
    btn_reference.pack(pady=8)

    btn_exit = tk.Button(root, text='Exit', width=25, command=root.destroy)
    btn_exit.pack(pady=8)

    return root


def main():
    app = create_root()
    app.mainloop()
