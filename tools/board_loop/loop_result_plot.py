"""Plot all exported calibration records, with accepted/rejected frames distinct."""
from __future__ import annotations

from pathlib import Path
import math


def _finite(value):
    return value is not None and math.isfinite(value)


def build_figure(rows, summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True, layout='constrained')
    indexed = bool(rows) and all(row['capture'] is not None for row in rows)
    x = [row['capture'] if indexed else i + 1 for i, row in enumerate(rows)]
    accepted = [i for i, row in enumerate(rows) if row['status'] == 'ok']
    rejected_x = [x[i] for i, row in enumerate(rows) if row['status'] != 'ok']
    blue, orange, red = '#2563eb', '#d97706', '#dc2626'
    for ax in axes.flat:
        ax.grid(alpha=.18)
        ax.spines[['top', 'right']].set_visible(False)

    for axis, key, scale, label in (
        (axes[0, 0], 'gain_error_ppm', 1e4, 'Gain residual (%)'),
        (axes[1, 0], 'offset_error_codes', 1, 'Offset B - A (ADC codes)'),
        (axes[2, 0], 'skew_error_ps', 1, 'Skew residual (ps)'),
    ):
        valid = [i for i in accepted if _finite(rows[i][key])]
        raw_x = [x[i] for i in valid]
        raw_y = [rows[i][key] / scale for i in valid]
        # Every accepted measurement is drawn, not just the final window/means.
        axis.plot(raw_x, raw_y, '.-', ms=3, lw=.5, alpha=.4,
                  color=blue, label='Accepted measurements')
        window = 20
        if len(raw_y) >= window:
            windows = [i for i in range(window-1, len(raw_y))
                       if rows[valid[i-window+1]].get('phase') == rows[valid[i]].get('phase')]
            means = [sum(raw_y[i-window+1:i+1])/window for i in windows]
            axis.plot([raw_x[i] for i in windows], means, lw=1.8, color=orange,
                      label='Mean of 20 accepted frames')
        if rejected_x:
            axis.plot(rejected_x, [.97] * len(rejected_x), '|', color=red,
                      transform=axis.get_xaxis_transform(), label='Rejected captures')
        axis.axhline(0, color='#64748b', lw=.8, ls='--')
        axis.set_ylabel(label)
        axis.legend(fontsize=7, loc='best')

    deadband = summary.get('actuator', {}).get('deadband_ps')
    if _finite(deadband) and deadband > 0:
        axes[2, 0].axhspan(-deadband, deadband, alpha=.06, color='#16a34a')

    for key, color, label in (('gain_corr_a', blue, 'Channel A'),
                              ('gain_corr_b', orange, 'Channel B')):
        values = [(row[key] - 1) * 100 if _finite(row[key]) else math.nan for row in rows]
        axes[0, 1].plot(x, values, color=color, lw=1, label=label)
    axes[0, 1].set_ylabel('Gain correction - 1 (%)')
    axes[0, 1].legend(fontsize=8)
    for key, color, label in (('offset_a_codes', blue, 'Channel A'),
                              ('offset_b_codes', orange, 'Channel B')):
        axes[1, 1].plot(x, [row[key] if _finite(row[key]) else math.nan for row in rows],
                        color=color, lw=1, label=label)
    axes[1, 1].set_ylabel('Offset correction (ADC codes)')
    axes[1, 1].legend(fontsize=8)
    axes[2, 1].step(x, [row['actuator_code'] if row['actuator_code'] is not None else math.nan
                       for row in rows], where='post', color=blue, lw=1.3)
    axes[2, 1].set_ylabel('Skew actuator code')
    for axis in axes[2, :]:
        axis.set_xlabel('Capture index' if indexed else 'Exported record index')

    phase_test = summary.get('source') == 'single_axis_test'
    if phase_test and rows:
        start = 0
        for end in range(1, len(rows) + 1):
            if end < len(rows) and rows[end].get('phase') == rows[start].get('phase'):
                continue
            name = rows[start].get('phase', '')
            color = '#16a34a' if 'hold' in name else '#d97706' if 'train' in name else '#64748b'
            for ax in axes.flat:
                ax.axvspan(x[start] - .5, x[end - 1] + .5, color=color, alpha=.055, zorder=0)
            for ax in axes[0, :]:
                ax.text((x[start] + x[end - 1])/2, 1.01, name.upper(),
                        transform=ax.get_xaxis_transform(), ha='center', fontsize=8)
            start = end
    status = '' if summary['history_complete'] and summary['run_completed'] else 'INCOMPLETE - '
    title = f"Dither {summary.get('axis') or 'calibration'} test" if phase_test else 'Dither calibration'
    fig.suptitle(f"{status}{title} | {len(rows)} records | "
                 f"{summary['qualified']}/{summary['requested']} accepted frames", fontsize=13)
    caption = ('PRE/HOLD frozen; TRAIN updates selected axes. Red ticks mark rejected captures.' if phase_test else
               'Feedback active. Red ticks mark rejected captures; residual curves use accepted measurements.')
    fig.supxlabel(caption,
                  fontsize=8)
    return fig


def save_plots(rows, summary, log_path):
    fig = build_figure(rows, summary)
    import matplotlib.pyplot as plt
    paths = {extension: str(Path(log_path).with_suffix('.' + extension))
             for extension in ('png', 'svg')}
    try:
        fig.savefig(paths['png'], dpi=180)
        fig.savefig(paths['svg'])
    finally:
        plt.close(fig)
    return paths
