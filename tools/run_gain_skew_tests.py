"""Run gain then skew sweeps serially, reusing the saved reference-free baseline."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uart', default='COM5')
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--out', default='calibration_out/gain_skew_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be positive')
    root = Path(__file__).resolve().parents[1]
    out = Path(args.out).resolve()
    commands = {axis: [sys.executable, '-u', str(root / 'tools/board_axis_test.py'),
                      'sweep', '--axis', axis, '--uart', args.uart, '--reuse-baseline',
                      '--repeats', str(args.repeats), '--out', str(out / axis)]
                for axis in ('gain', 'skew')}
    if not args.execute:
        for cmd in commands.values():
            print(subprocess.list2cmdline(cmd + ['--execute']))
        print('PLAN ONLY; add --execute to run both sweeps.')
        return 0
    out.mkdir(parents=True, exist_ok=False)
    status = {'completed': False, 'uart': args.uart, 'repeats': args.repeats, 'runs': {}}
    def save():
        (out / 'sequence.json').write_text(json.dumps(status, indent=2), encoding='utf-8')
    save()
    try:
        for axis, cmd in commands.items():
            status['active_axis'] = axis
            save()
            print(f'START {axis}: {out / axis}', flush=True)
            result = subprocess.run(cmd + ['--execute'], cwd=root)
            meta_path = out / axis / 'meta.json'
            meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
            status['runs'][axis] = {'exit_code': result.returncode,
                                    'completed': meta.get('completed', False),
                                    'stop_reason': meta.get('stop_reason')}
            # Exit 1 can mean either failed HOLD grades or a hardware error.
            # Continue only when the full sweep AND its final restore completed.
            if not meta.get('completed') or result.returncode not in (0, 1):
                status['stop_reason'] = f'{axis} did not complete; subsequent sweep not started'
                save()
                return 2
            save()
        status['active_axis'] = None
        status['completed'] = True
        save()
        return int(any(r['exit_code'] for r in status['runs'].values()))
    except BaseException as exc:
        status['stop_reason'] = f'{type(exc).__name__}: {exc}'
        save()
        raise


if __name__ == '__main__':
    raise SystemExit(main())
