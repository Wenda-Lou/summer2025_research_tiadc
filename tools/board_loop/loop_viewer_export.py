"""Write a self-contained loop_viewer.html page for one exported run CSV.

board_loop_log.py calls this after every export, so each run gets a page beside
its CSV (`<stem>.html`) that opens straight onto that run: no file picking, no
network. It can also be run by hand for an existing CSV or a sweep folder:

    python tools/board_loop/loop_viewer_export.py calibration_out/dither_run300.csv
    python tools/board_loop/loop_viewer_export.py calibration_out/axis_20261028_203116 --open
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
import webbrowser

TEMPLATE = Path(__file__).with_name('loop_viewer.html')
EMBEDDED = re.compile(r'(<script id="embedded" type="application/json">)(.*?)(</script>)', re.S)


def _datasets(source: Path):
    """CSV file -> one run dataset; folder -> its sweep (summary/frames) and/or run CSVs."""
    if source.is_file():
        return [{'name': source.parent.name or 'run',
                 'files': {source.name: source.read_text(encoding='utf-8')}}]
    files = {}
    for name in ('summary.csv', 'frames.csv'):
        if (source / name).is_file():
            files[name] = (source / name).read_text(encoding='utf-8')
    if files:
        return [{'name': source.name, 'files': files}]
    # a gain_skew_* folder holds one sweep per axis
    out = []
    for sub in sorted(p for p in source.iterdir() if p.is_dir()):
        out.extend(d for d in _datasets(sub) if 'summary.csv' in d['files'])
    return out


def write_viewer(source, out=None, open_browser=False):
    """Embed `source` (a run CSV or a sweep folder) into a copy of loop_viewer.html."""
    source = Path(source)
    datasets = _datasets(source)
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
    ap.add_argument('source', help='run CSV from board_loop_log.py, or an axis_*/gain_skew_* folder')
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
