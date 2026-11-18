"""Offline viewer/export checks; browser tests never open a serial port."""
import contextlib
import html
import io
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import uuid

from board_signal_output import main as download_signal
from loop_viewer_export import EMBEDDED, TEMPLATE, _datasets, write_viewer

ROOT = Path(__file__).resolve().parents[2]
SIGNAL_HEADER = 'sample,time_s,output_a_codes,output_b_codes\n'
RUN_CSV = ('capture,iteration,status,gain_error_ppm,offset_error_codes,skew_error_ps,actuator_code,phase\n'
           '1,0,ok,10,0.1,2,24,pre\n2,1,ok,5,0.05,1,24,train\n3,2,ok,2,0.01,0.5,24,hold\n')


def signal_text():
    return SIGNAL_HEADER + ''.join(
        f'{i},{i/1.3e9},{800*math.sin(2*math.pi*64*i/512)},'
        f'{800*math.sin(2*math.pi*64*i/512)+0.25}\n' for i in range(512))


def metadata(status='cancelled'):
    return {'source': 'frozen_board_signal_snapshot', 'status': status,
            'dither_cancelled': status == 'cancelled', 'samples': 512,
            'coefficients': 'frozen', 'sample_rate_hz': 1300000000}


class ViewerExportTests(unittest.TestCase):
    def test_signal_embeds_samples_and_status_without_live_file_access(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'signal.csv'
            path.write_text(signal_text(), encoding='utf-8')
            meta = metadata()
            meta['note'] = '</script><script>unsafe</script>'
            path.with_suffix('.json').write_text(json.dumps(meta), encoding='utf-8')
            page = write_viewer(path).read_text(encoding='utf-8')
            files = json.loads(EMBEDDED.search(page).group(2))['datasets'][0]['files']
            self.assertEqual(files['signal.csv'], path.read_text())
            self.assertEqual(json.loads(files['signal.json']), meta)
            self.assertNotIn('</script><script>unsafe', EMBEDDED.search(page).group(2))

    def test_folder_keeps_calibration_and_signal_and_nested_sweeps(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'run.csv').write_text(RUN_CSV)
            (root/'signal.csv').write_text(signal_text())
            (root/'signal.json').write_text(json.dumps(metadata('ambiguous')))
            sub = root/'axis'/'gain'
            sub.mkdir(parents=True)
            (sub/'summary.csv').write_text('axis,point,hold_accepted\ngain,0,100\n')
            (sub/'frames.csv').write_text('experiment,phase\ngain,hold\n')
            datasets = _datasets(root)
            self.assertEqual(len(datasets), 3)
            names = {name for ds in datasets for name in ds['files']}
            self.assertEqual(names, {'run.csv', 'signal.csv', 'signal.json', 'summary.csv', 'frames.csv'})
            self.assertTrue(write_viewer(root).exists())

    def test_csv_without_sidecar_is_still_viewable_but_cannot_claim_cancelled(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'signal.csv'
            path.write_text(signal_text())
            files = json.loads(EMBEDDED.search(write_viewer(path).read_text(encoding='utf-8')).group(2))['datasets'][0]['files']
            self.assertEqual(list(files), ['signal.csv'])

    def test_download_opens_viewer_and_retains_failed_status(self):
        for status, no_viewer in [('cancelled', False), ('ambiguous', False), ('cancelled', True)]:
            lines = [f'LOOP-SIGNAL-BEGIN version=1 samples=512 status={status} '
                     'sample_rate_hz=1300000000 coefficients=frozen',
                     *(f'LOOP-SIGNAL-DATA {i},1000,-1000' for i in range(512)),
                     f'LOOP-SIGNAL-END result=OK samples=512 status={status}']
            commands = []

            class Port:
                def __enter__(self): return self
                def __exit__(self, *args): pass
                def reset_input_buffer(self): pass
                def write(self, command): commands.append(command)
                def flush(self): pass
                def readline(self): return (lines.pop(0)+'\n').encode()

            with tempfile.TemporaryDirectory() as directory, \
                    patch.dict(sys.modules, serial=types.SimpleNamespace(Serial=lambda *a, **k: Port())), \
                    patch('board_signal_output.time.sleep'), patch('webbrowser.open') as browser, \
                    contextlib.redirect_stdout(io.StringIO()):
                path = Path(directory)/'signal.csv'
                args = ['--uart', 'FAKE', '--out', str(path)] + (['--no-viewer'] if no_viewer else [])
                self.assertEqual(download_signal(args), 0 if status == 'cancelled' else 1)
                self.assertEqual(commands, [b'adc -cal loop output\r'])
                self.assertEqual(path.with_suffix('.html').exists(), not no_viewer)
                self.assertEqual(browser.call_count, 0 if no_viewer else 1)
                self.assertEqual(json.loads(path.with_suffix('.json').read_text())['status'], status)


def browser_path():
    return os.environ.get('TIADC_BROWSER') or shutil.which('chromium') or shutil.which('google-chrome') or next(
        (str(p) for p in [Path('C:/Program Files/Google/Chrome/Application/chrome.exe'),
                         Path('C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe')] if p.is_file()), None)


@unittest.skipUnless(browser_path(), 'Chrome/Chromium/Edge not available')
class BrowserTests(unittest.TestCase):
    def test_waveform_controls_status_fft_and_legacy_run_in_real_browser(self):
        work = ROOT/'build'/'viewer-check'
        work.mkdir(parents=True, exist_ok=True)
        data = json.dumps({'csv': signal_text(), 'meta': metadata(), 'run': RUN_CSV})
        script = r'''
<script>
(() => {
  const results = [];
  function check(value, name) { if (!value) throw new Error(name); results.push(name); }
  try {
    const fixture = FIXTURE;
    check(state.datasets.some(d => d.kind === 'sweep'), 'legacy embedded sweep loads');
    check(classify('signal.csv', fixture.csv) === 'signal', 'detect signal CSV');
    const known = signalFromCSV(fixture.csv, fixture.meta);
    check(known.cancelled && known.rows.length === 512, 'known cancelled status and sample count');
    check(Math.abs(known.rows[511].t-511/1.3)<1e-8, 'time unit is ns');
    check(!signalFromCSV(fixture.csv, null).cancelled, 'CSV alone cannot claim cancellation');
    check(!signalFromCSV(fixture.csv, {...fixture.meta, status:'ambiguous', dither_cancelled:false}).cancelled,
          'failed cancellation remains failed');
    for (const bad of [{...fixture.meta, samples:513}, {...fixture.meta, sample_rate_hz:1e9},
                      {...fixture.meta, dither_cancelled:false}]) {
      let rejected = false;
      try { signalFromCSV(fixture.csv, bad); } catch (e) { rejected = true; }
      check(rejected, 'inconsistent sidecar rejected');
    }
    // Explicitly corrupt an index independently of floating-point string formatting.
    const lines = fixture.csv.split('\n'); lines[2] = lines[2].replace(/^1,/, '2,');
    let rejected = false;
    try { signalFromCSV(lines.join('\n'), fixture.meta); } catch(e) { rejected = true; }
    check(rejected, 'duplicate/missing sample rejected');
    state.datasets = [];
    addFiles([{path:'run/signal.csv', name:'signal.csv', text:fixture.csv},
              {path:'run/signal.json', name:'signal.json', text:JSON.stringify(fixture.meta)},
              {path:'run/run.csv', name:'run.csv', text:fixture.run}]);
    check(state.datasets.length === 2, 'run and waveform coexist');
    const signal = state.datasets.find(d => d.kind === 'signal');
    state.active = state.datasets.indexOf(signal); render();
    check(document.getElementById('view').textContent.includes('已抵消 dither'), 'cancelled badge shown');
    check(document.querySelectorAll('#view .chart svg').length === 3, 'waveform/difference/spectrum rendered');
    check(document.getElementById('signal-span').textContent.includes('0–127'), 'default time window');
    const select = document.getElementById('signal-window'); select.value = '64'; select.dispatchEvent(new Event('change'));
    const slider = document.getElementById('signal-start'); slider.value = '11'; slider.dispatchEvent(new Event('input'));
    check(document.getElementById('signal-span').textContent.includes('11–74'), 'range and start controls work');
    select.value = '512'; select.dispatchEvent(new Event('change'));
    check(slider.disabled && document.getElementById('signal-span').textContent.includes('0–511'), 'full-frame mode');
    const spectrum = signalSpectrum(known.rows, known.rate, 'a');
    const peak = spectrum.reduce((a,b) => a[1]>b[1] ? a : b);
    check(Math.abs(peak[0]-162.5)<1e-6, 'FFT frequency is MHz');
    check(Math.abs(peak[1]-20*Math.log10(800/8192))<0.01, 'FFT Hann normalization and dBFS');
    signal.meta = null; signal.cancelled = false; render();
    check(document.getElementById('view').textContent.includes('抵消狀態未確認') &&
          !document.querySelector('#view .status.pass'), 'missing JSON never renders a success badge');
    signal.meta = {...fixture.meta, status:'ambiguous', dither_cancelled:false}; render();
    check(document.querySelector('#view .status.fail').textContent.includes('ambiguous'), 'failure badge shown');
    state.active = state.datasets.findIndex(d => d.kind === 'run'); render();
    check(document.getElementById('view').textContent.includes('收斂過程'), 'legacy run still renders');
    check(['PRE', 'TRAIN', 'HOLD'].every(p => document.getElementById('view').textContent.includes(p)),
          'manual test phase bands coexist with final waveform');
    signal.meta = fixture.meta; signal.cancelled = true;
    state.active = state.datasets.indexOf(signal); signal.signalWindow = 128; render();
    document.body.dataset.viewerTests = 'pass';
  } catch (e) { results.push(e.stack || e.message); document.body.dataset.viewerTests = 'fail'; }
  const result = document.createElement('pre'); result.id = 'viewer-check-results';
  result.textContent = JSON.stringify(results); document.body.appendChild(result);
})();
</script>
'''.replace('FIXTURE', data)
        page = TEMPLATE.read_text(encoding='utf-8').replace('</body>', script+'</body>')
        test_page = work/'browser-test.html'
        test_page.write_text(page, encoding='utf-8')
        completed = subprocess.run([browser_path(), '--headless', '--disable-gpu', '--no-first-run',
            '--no-default-browser-check', '--disable-background-networking',
            '--host-resolver-rules=MAP * ~NOTFOUND', '--user-data-dir='+str(work/('profile-'+uuid.uuid4().hex)),
            '--window-size=1440,1600', '--screenshot='+str(work/'waveform.png'), '--dump-dom',
            test_page.resolve().as_uri()], capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=40)
        (work/'browser-dom.html').write_text(completed.stdout, encoding='utf-8')
        match = re.search(r'<pre id="viewer-check-results">(.*?)</pre>', completed.stdout, re.S)
        self.assertIsNotNone(match, completed.stderr[-2000:])
        results = json.loads(html.unescape(match.group(1)))
        (work/'browser-results.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
        self.assertTrue('data-viewer-tests="pass"' in completed.stdout, results)
        self.assertGreaterEqual(len(results), 20)


if __name__ == '__main__':
    unittest.main()
