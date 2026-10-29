"""Decode board loop results; CSV uses ADC codes, ps and multiplicative gains."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import re
import statistics


WIRE_COLUMNS = ('capture,it,st,used_gain_ppm,doff_mc,skew_cps,corr_a_ppm,'
                'corr_b_ppm,offa_mc,offb_mc,code,act,steps,batch_used,batch_n,'
                'bmean_cps,clips,n0,rot,margin_x100').split(',')
CSV_COLUMNS = ['capture', 'iteration', 'status', 'gain_error_ppm',
               'offset_error_codes', 'skew_error_ps', 'gain_corr_a', 'gain_corr_b',
               'offset_a_codes', 'offset_b_codes', 'actuator_code', 'skew_action',
               'skew_step_codes', 'batch_used', 'batch_frames', 'batch_skew_ps',
               'clipped_samples', 'n0', 'rotation', 'alignment_margin', 'phase', 'attempt']


def canonical(line):
    line = line.strip()
    return line.split(':', 1)[1].strip() if line.startswith('uart-cmd$:') else line


def fields(line):
    return dict(re.findall(r'(\w+)=([^\s]+)', line))


def number(value, scale=1.0, offset=0.0):
    if value is None:
        return None
    x = float(value)
    return x / scale + offset if math.isfinite(x) else None


def row_to_units(row):
    batch = row.get('batch', '').split('/')
    action = row.get('act', 'none')
    step = row.get('steps')
    if step is None:
        match = re.fullmatch(r'(move:[A-Za-z]+)([+-]\d+)', action)
        action, step = match.groups() if match else (action, '0')
    result = dict(zip(CSV_COLUMNS, [
        int(row['capture']) if row.get('capture') is not None else None,
        int(row['it']) if row.get('it') is not None else None,
        row.get('st', 'unknown'), number(row.get('used_gain_ppm')),
        number(row.get('doff_mc'), 1e3), number(row.get('skew_cps'), 100),
        number(row.get('corr_a_ppm'), 1e6, 1), number(row.get('corr_b_ppm'), 1e6, 1),
        number(row.get('offa_mc'), 1e3), number(row.get('offb_mc'), 1e3),
        int(row['code']) if row.get('code') is not None else None, action, int(step),
        int(row.get('batch_used', batch[0] if len(batch) == 2 else '0')),
        int(row.get('batch_n', batch[1] if len(batch) == 2 else '0')),
        number(row.get('bmean_cps'), 100),
        int(row['clips']) if row.get('clips') is not None else None,
        int(row['n0']) if row.get('n0') is not None else None,
        int(row['rot']) if row.get('rot') is not None else None,
        number(row.get('margin_x100'), 100),
    ]))
    result['phase'] = row.get('phase', '')
    result['attempt'] = int(row['attempt']) if row.get('attempt') is not None else None
    return result


def metric_stats(rows, key):
    values = [row[key] for row in rows if row[key] is not None]
    return {'n': len(values), 'mean': statistics.mean(values) if values else None,
            'sd': statistics.stdev(values) if len(values) > 1 else None}


def decode_results(lines):
    lines = [canonical(line) for line in lines]
    starts = [fields(line) for line in lines if line.startswith('LOOP-START ')]
    begins = [fields(line) for line in lines if line.startswith('LOOP-EXPORT-BEGIN ')]
    if begins:
        metas = [fields(line) for line in lines if line.startswith('LOOP-EXPORT-META ')]
        ends = [fields(line) for line in lines if line.startswith('LOOP-EXPORT-END ')]
        headers = [line[len('LOOP-EXPORT-HEADER '):] for line in lines
                   if line.startswith('LOOP-EXPORT-HEADER ')]
        if len(begins) != 1 or len(metas) != 1 or len(ends) != 1 or len(headers) != 1:
            raise ValueError('incomplete or mixed export envelope')
        begin, meta, end = begins[0], metas[0], ends[0]
        version = begin.get('version')
        if version not in ('1', '2') or headers[0].split(',') != WIRE_COLUMNS or (
                version == '2' and begin.get('mode') != 'stream'):
            raise ValueError('unsupported export schema; update the PC tool/firmware together')
        if end.get('result') != 'OK' or not begin.get('run_id') or not (
                begin['run_id'] == meta.get('run_id') == end.get('run_id')):
            raise ValueError('failed export or mismatched run IDs')
        if starts and starts[-1].get('run_id') not in (None, begin['run_id']):
            raise ValueError('export belongs to a different calibration run')
        if not {'requested', 'qualified', 'captures', 'code'} <= meta.keys():
            raise ValueError('export metadata is incomplete')
        if starts and starts[-1].get('samples') not in (None, meta['requested']):
            raise ValueError('export requested budget differs from the calibration run')
        wire = []
        for line in lines:
            if line.startswith('LOOP-EXPORT-DATA '):
                values = next(csv.reader([line[len('LOOP-EXPORT-DATA '):]]))
                if len(values) != len(WIRE_COLUMNS):
                    raise ValueError('damaged export row')
                wire.append(dict(zip(WIRE_COLUMNS, values)))
        if (version == '1' and len(wire) != int(begin['rows'])) or len(wire) != int(end['rows']):
            raise ValueError('export row count mismatch; raw UART log retained')
        indices = [int(row['capture']) for row in wire]
        if any(b <= a for a, b in zip(indices, indices[1:])):
            raise ValueError('duplicate or out-of-order capture indices')
        if indices and (indices[0] < 1 or indices[-1] > int(meta['captures'])):
            raise ValueError('capture index outside the reported run')
        accounting = end if version == '2' else begin
        dropped = int(accounting['dropped'])
        total = int(accounting['total_rows'])
        if dropped < 0 or total != len(wire) + dropped or total > int(meta['captures']):
            raise ValueError('export history accounting mismatch')
        if version == '2' and dropped != 0:
            raise ValueError('streaming export must contain every record')
        source = f'firmware_export_v{version}'
    else:
        if any(line.startswith('LOOP-EXPORT-') for line in lines):
            raise ValueError('export did not complete')
        # Older firmware logs already contain every row; no extra UART command.
        wire = [fields(line) for line in lines if line.startswith('LOOP it=')]
        statuses = [fields(line) for line in lines if line.startswith('LOOP-STATUS ')]
        endings = [fields(line) for line in lines if line.startswith('LOOP-END ')]
        if not wire or not statuses or not endings:
            raise ValueError('no completed loop results in log')
        meta = dict(starts[-1] if starts else {}, **endings[-1])
        meta.update(statuses[-1])
        meta['requested'] = meta.get('samples', endings[-1].get('qualified', '0/0').split('/')[-1])
        # Failed DMA reads have no estimator row; they are not lost export data.
        known_capture_failures = sum(line.startswith('LOOP-CAPTURE ') for line in lines)
        total = int(meta['captures']) - known_capture_failures
        if total < len(wire):
            raise ValueError('legacy log contains more rows than the reported captures')
        dropped = max(0, total - len(wire))
        source = 'legacy_uart_rows'
    if not {'requested', 'qualified', 'captures', 'code'} <= meta.keys():
        raise ValueError('result metadata is incomplete')
    rows = [row_to_units(row) for row in wire]
    accepted = [row for row in rows if row['status'] == 'ok']
    finite_accepted = sum(all(row[key] is not None for key in (
        'gain_error_ppm', 'offset_error_codes', 'skew_error_ps')) for row in accepted)
    tail = accepted[-20:]
    qualified = int(meta['qualified'].split('/')[0])
    requested = int(meta['requested'])
    aborted = int(meta.get('skew_abort', '0'))
    if source.startswith('firmware_export_v') and dropped == 0 and len(accepted) != qualified:
        raise ValueError('accepted row count differs from the reported run')
    summary = {
        'schema_version': 1, 'source': source, 'metadata': meta,
        'waveform': meta.get('waveform'),
        'gain_source': meta.get('gsrc', meta.get('gain')),
        'skew_route': meta.get('skew_route', meta.get('skew')),
        'requested': requested, 'qualified': qualified, 'captures': int(meta['captures']),
        'rejected': int(meta.get('rejected', '0')), 'skew_abort': aborted,
        'run_completed': qualified == requested,
        'run_healthy': qualified == requested and aborted == 0 and finite_accepted == len(accepted),
        'finite_accepted_rows': finite_accepted,
        'exported_rows': len(rows), 'total_rows': total, 'dropped_rows': dropped,
        'history_complete': dropped == 0,
        'capture_failures_without_measurement': max(0, int(meta['captures']) - total)
            if source.startswith('firmware_export_v') or dropped == 0 else None,
        'actuator': {'code': int(meta['code']),
                     'deadband_ps': number(meta.get('deadband_cps'), 100),
                     'step_ps': number(meta.get('step_cps'), 100)},
        'actuator_code': int(meta['code']),
        'correction': {'equation': '(raw - offset_codes) * gain_corr',
                       'gain_corr_a': number(meta.get('corr_a_ppm'), 1e6, 1),
                       'gain_corr_b': number(meta.get('corr_b_ppm'), 1e6, 1),
                       'offset_a_codes': number(meta.get('offa_mc'), 1e3),
                       'offset_b_codes': number(meta.get('offb_mc'), 1e3)},
        'tail': {'window': len(tail), 'feedback': 'active',
                 'gain_error_ppm': metric_stats(tail, 'gain_error_ppm'),
                 'offset_error_codes': metric_stats(tail, 'offset_error_codes'),
                 'skew_error_ps': metric_stats(tail, 'skew_error_ps')},
        'precision_validation': 'training_residuals_only',
    }
    if source == 'legacy_uart_rows':
        for key, field in (('gain_source', 'gsrc'), ('skew_route', 'skew_route')):
            tags = {row[field] for row in wire if field in row}
            if tags:
                summary[key] = next(iter(tags)) if len(tags) == 1 else 'mixed'
    return rows, summary


def save_results(lines, log_path):
    rows, summary = decode_results(lines)
    path = Path(log_path)
    csv_path, json_path = path.with_suffix('.csv'), path.with_suffix('.json')
    if path.suffix.lower() in ('.csv', '.json', '.png', '.svg'):
        raise ValueError('--out must name a UART log, not a data/image file')
    with csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    if __package__:
        from .loop_result_plot import save_plots
    else:
        from loop_result_plot import save_plots
    summary['plots'] = save_plots(rows, summary, path)
    json_path.write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return csv_path, json_path, summary
