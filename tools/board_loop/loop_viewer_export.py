"""Write a self-contained viewer for calibration runs or final signal outputs.

board_loop_log.py calls this after every export, so each run gets a page beside
its CSV (`<stem>.html`) that opens straight onto that run: no file picking, no
network. It can also be run by hand for an existing CSV or a sweep folder:

    python tools/board_loop/loop_viewer_export.py calibration_out/dither_run300.csv
    python tools/board_loop/loop_viewer_export.py calibration_out/axis_20261028_203116 --open
"""
from __future__ import annotations

import argparse
import csv
import io
import json
from pathlib import Path
import re
import sys
import webbrowser

TEMPLATE = Path(__file__).with_name('loop_viewer.html')
EMBEDDED = re.compile(r'(<script id="embedded" type="application/json">)(.*?)(</script>)', re.S)


def _sweep_datasets(name, files):
    """The viewer expects one axis per sweep; suite summaries contain all three."""
    if 'summary.csv' not in files:
        return [{'name': name, 'files': files}]
    reader = csv.DictReader(io.StringIO(files['summary.csv']))
    summary_columns = reader.fieldnames
    summary = list(reader)
    axes = list(dict.fromkeys(r.get('axis') for r in summary if r.get('axis')))
    if not axes:
        return [{'name': name, 'files': files}]
    frame_reader = csv.DictReader(io.StringIO(files.get('frames.csv', '')))
    frame_columns, frames = frame_reader.fieldnames, list(frame_reader)

    def encoded(columns, rows):
        stream = io.StringIO(newline='')
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
        return stream.getvalue()

    result = []
    for axis in axes:
        selected = {'summary.csv': encoded(summary_columns, [r for r in summary if r.get('axis') == axis])}
        if frame_columns:
            selected_frames = [r for r in frames if r.get('experiment', axis) == axis]
            selected['frames.csv'] = encoded(frame_columns, selected_frames)
        result.append({'name': f'{name} / {axis}' if len(axes) > 1 else name, 'files': selected})
    return result


def _datasets(source: Path):
    """Include signal status sidecars; folders may contain runs and waveforms."""
    if source.is_file():
        files = {source.name: source.read_text(encoding='utf-8-sig')}
        sidecar = source.with_suffix('.json')
        if source.suffix.lower() == '.csv' and sidecar.is_file():
            files[sidecar.name] = sidecar.read_text(encoding='utf-8-sig')
        # board_loop_log.py's final ADC waveform for this run, if it was captured over UDP
        wave = source.with_name(source.stem + '_waveform.csv')
        if wave.is_file():
            files[wave.name] = wave.read_text(encoding='utf-8')
        return [{'name': source.parent.name or 'run',
                 'files': files}]
    out = []
    files = {}
    for name in ('summary.csv', 'frames.csv'):
        if (source / name).is_file():
            files[name] = (source / name).read_text(encoding='utf-8')
    if files:
        out.extend(_sweep_datasets(source.name, files))
    for csv in sorted(source.glob('*.csv')):
        if csv.name in ('summary.csv', 'frames.csv'):
            continue
        with csv.open(encoding='utf-8-sig') as stream:
            header = stream.readline().strip()
        if header.startswith('capture,iteration,status') or header == 'sample,time_s,output_a_codes,output_b_codes':
            out.extend(_datasets(csv))
    for sub in sorted(p for p in source.iterdir() if p.is_dir()):
        out.extend(_datasets(sub))
    return out


def write_viewer(source, out=None, open_browser=False, signal=None):
    """Embed `source` (a run CSV or a sweep folder) into a copy of loop_viewer.html."""
    source = Path(source)
    datasets = _datasets(source)
    if signal is not None:
        datasets.extend(_datasets(Path(signal)))
    if not datasets:
        raise ValueError(f'nothing to show in {source}')
    template = TEMPLATE.read_text(encoding='utf-8')
    if not EMBEDDED.search(template):
        raise ValueError(f'{TEMPLATE} has no embedded-data block')
    blob = json.dumps({'active': 0, 'datasets': datasets}, ensure_ascii=False).replace('</', '<\\/')
    page = EMBEDDED.sub(lambda m: m.group(1) + blob + m.group(3), template, count=1)
    if out is None:
        out = source.with_suffix('.html') if source.is_file() else source / 'viewer.html'
    out = Path(out)
    out.write_text(page, encoding='utf-8', newline='\n')
    if open_browser:
        webbrowser.open(out.resolve().as_uri())
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('source', help='calibration/signal CSV, or a run/axis_*/gain_skew_* folder')
    ap.add_argument('--out', help='HTML path (default: beside the CSV, or viewer.html in the folder)')
    ap.add_argument('--open', action='store_true', help='open the page in the default browser')
    args = ap.parse_args(argv)
    try:
        print(f'viewer: {write_viewer(args.source, args.out, args.open)}')
    except (OSError, ValueError) as exc:
        print(f'error: {exc}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
