"""Save a complete single-axis test, preserving PRE/TRAIN/HOLD boundaries."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

if __package__:
    from .loop_result_export import CSV_COLUMNS, canonical, fields, metric_stats, row_to_units
    from .loop_result_plot import save_plots
else:
    from loop_result_export import CSV_COLUMNS, canonical, fields, metric_stats, row_to_units
    from loop_result_plot import save_plots


def _json_safe(value):
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value


def decode_test_results(lines, assessment=None):
    lines = [canonical(line) for line in lines]
    phases = [fields(line) for line in lines if line.startswith('LOOP-TEST-PHASE ')]
    summaries = [fields(line) for line in lines if line.startswith('LOOP-TEST-SUMMARY ')]
    ends = [fields(line) for line in lines if line.startswith('LOOP-TEST-END ')]
    wire = [fields(line) for line in lines
            if line.startswith(('LOOP-TEST-ROW ', 'LOOP-TEST-CAPTURE '))]
    if not phases or not summaries or not ends or not wire:
        raise ValueError('no complete test records; use the matching ELF and a test command with full')
    rows = []
    for i, raw in enumerate(wire):
        raw['capture'] = str(i + 1)
        rows.append(row_to_units(raw))
    seen = [(row['phase'], row['attempt']) for row in rows]
    if len(seen) != len(set(seen)):
        raise ValueError('duplicate test phase/attempt records')
    budgets = {p['phase']: int(p['budget']) for p in phases}
    if any(s['phase'] not in budgets for s in summaries):
        raise ValueError('test summary has no matching phase header')
    phase_stats = {}
    complete = len(summaries) == len(phases)
    for s in summaries:
        name = s['phase']
        selected = [r for r in rows if r['phase'] == name]
        expected = int(s['attempts'])
        complete &= (len(selected) == expected and
                     [r['attempt'] for r in selected] == list(range(expected)))
        accepted = [r for r in selected if r['status'] == 'ok']
        complete &= len(accepted) == int(s['accepted'])
        phase_stats[name] = {'attempts': expected, 'budget': budgets[name],
            'accepted': len(accepted),
            **{key: metric_stats(accepted, key) for key in (
                'gain_error_ppm', 'offset_error_codes', 'skew_error_ps')}}
    train = next((p for p in phases if p['phase'] == 'train'), None)
    axis = {1: 'offset', 2: 'gain', 4: 'skew'}.get(int(train['axes'])) if train else None
    complete &= len(rows) == sum(int(s['attempts']) for s in summaries)
    if assessment is None and axis and 'hold' in budgets:
        # CLI module import is deferred so its sweep can use this exporter too.
        if __package__:
            from .board_axis_test import summarise_run
        else:
            from board_axis_test import summarise_run
        assessment = summarise_run(lines, axis, {'offset': .5, 'gain': .001, 'skew': 10}[axis],
                                   budgets.get('hold', 0))
    elif assessment is None and axis:
        assessment = {'pass': False, 'reason': 'HOLD not completed'}
    accepted = [r for r in rows if r['status'] == 'ok']
    hold = [r for r in accepted if r['phase'] in ('hold', 'baseline_hold')]
    tail = (hold or accepted)[-20:]
    final = next((r for r in reversed(rows) if r['gain_corr_a'] is not None), rows[-1])
    phase_feedback = {p['phase']: ('frozen' if int(p['axes']) == 0 else 'selected axes') for p in phases}
    ended_ok = ends[-1].get('result') == 'OK'
    attempted_all = all(s['attempts'] == s['budget'] for s in phase_stats.values())
    result = {'schema_version': 1, 'source': 'single_axis_test', 'axis': axis,
        'metadata': {'mode': 'single_axis', 'feedback_by_phase': phase_feedback},
        'requested': sum(budgets.values()), 'qualified': len(accepted), 'budget_unit': 'attempts',
        'captures': len(rows), 'rejected': len(rows) - len(accepted),
        'exported_rows': len(rows), 'total_rows': sum(int(s['attempts']) for s in summaries),
        'history_complete': bool(complete), 'dropped_rows': max(0,
            sum(int(s['attempts']) for s in summaries) - len(rows)),
        'run_completed': ended_ok and attempted_all,
        'run_healthy': ended_ok and complete and attempted_all and
            (assessment is None or bool(assessment.get('pass'))),
        'actuator_code': int(summaries[-1]['code']), 'phase_statistics': phase_stats,
        'assessment': assessment,
        'correction': {key: final[key] for key in (
            'gain_corr_a', 'gain_corr_b', 'offset_a_codes', 'offset_b_codes')},
        'tail': {'window': len(tail), 'feedback': 'frozen' if hold or
                 all(value == 'frozen' for value in phase_feedback.values()) else 'selected axes',
                 **{key: metric_stats(tail, key) for key in (
                     'gain_error_ppm', 'offset_error_codes', 'skew_error_ps')}},
        'precision_validation': 'independent_holdout' if hold else 'no_holdout',
    }
    return rows, _json_safe(result)


def save_test_results(lines, log_path, assessment=None):
    rows, result = decode_test_results(lines, assessment)
    path = Path(log_path)
    with path.with_suffix('.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    result['plots'] = save_plots(rows, result, path)
    path.with_suffix('.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    if not path.exists():
        path.write_text('\n'.join(line for line in lines if not line.startswith('LOOP-TEST-ROW ')) + '\n',
                        encoding='utf-8')
    return path.with_suffix('.csv'), path.with_suffix('.json'), result
