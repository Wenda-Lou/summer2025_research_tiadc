"""Temp: minimal markdown -> HTML for the onboarding doc (headings, tables, lists, quotes, code)."""
import html
import re
import sys

src, dst = sys.argv[1], sys.argv[2]
lines = open(src, encoding='utf-8').read().split('\n')

CSS = """
@page { size: A4; margin: 17mm 15mm 17mm 15mm; }
body { font-family: "Segoe UI", Calibri, Arial, sans-serif; font-size: 10.5pt;
       line-height: 1.45; color: #17202a; }
h1 { font-size: 19pt; margin: 0 0 10px 0; }
h2 { font-size: 13.5pt; margin: 20px 0 6px 0; padding-bottom: 3px;
     border-bottom: 1px solid #d5dbe1; }
h3 { font-size: 11.5pt; margin: 14px 0 4px 0; }
p  { margin: 6px 0; }
ul { margin: 6px 0 6px 0; padding-left: 20px; }
li { margin: 3px 0; }
table { border-collapse: collapse; width: 100%; margin: 9px 0; font-size: 9.5pt; }
th, td { border: 1px solid #b9c2cc; padding: 4px 6px; text-align: left; vertical-align: top; }
th { background: #eef3f8; font-weight: 600; }
code { font-family: Consolas, "Courier New", monospace; font-size: 9.5pt;
       background: #f2f4f6; padding: 0 2px; border-radius: 2px; }
pre { background: #f6f7f9; border: 1px solid #dbe0e6; padding: 7px 9px;
      border-radius: 3px; overflow-x: auto; }
pre code { background: none; padding: 0; }
blockquote { margin: 8px 0; padding: 2px 0 2px 11px; border-left: 3px solid #c9d2dc;
             color: #2c3e50; }
strong { font-weight: 600; }
tr, li, blockquote, pre, h2, h3 { page-break-inside: avoid; }
"""


def inline(t: str) -> str:
    spans = []

    def stash(m):
        spans.append(m.group(1))
        return f'\x00{len(spans) - 1}\x00'

    t = re.sub(r'`([^`]+)`', stash, t)
    t = html.escape(t, quote=False)
    t = re.sub(r'\*\*([^*]+)\*\*', r'<strong>\1</strong>', t)
    t = re.sub(r'(?<!\*)\*([^*]+)\*(?!\*)', r'<em>\1</em>', t)
    for i, s in enumerate(spans):
        t = t.replace(f'\x00{i}\x00', f'<code>{html.escape(s, quote=False)}</code>')
    return t


out, i = [], 0
while i < len(lines):
    ln = lines[i]

    if ln.startswith('```'):                                  # code fence
        i += 1
        buf = []
        while i < len(lines) and not lines[i].startswith('```'):
            buf.append(lines[i]); i += 1
        i += 1
        out.append('<pre><code>' + html.escape('\n'.join(buf), quote=False) + '</code></pre>')
        continue

    m = re.match(r'^(#{1,3}) (.*)$', ln)                       # heading
    if m:
        lvl = len(m.group(1))
        out.append(f'<h{lvl}>{inline(m.group(2))}</h{lvl}>')
        i += 1
        continue

    if ln.startswith('|') and i + 1 < len(lines) and re.match(r'^\|[\s:-]+\|', lines[i + 1]):
        head = [c.strip() for c in ln.strip('|').split('|')]
        i += 2
        rows = []
        while i < len(lines) and lines[i].startswith('|'):
            rows.append([c.strip() for c in lines[i].strip('|').split('|')])
            i += 1
        t = ['<table><thead><tr>' + ''.join(f'<th>{inline(c)}</th>' for c in head)
             + '</tr></thead><tbody>']
        for r in rows:
            t.append('<tr>' + ''.join(f'<td>{inline(c)}</td>' for c in r) + '</tr>')
        t.append('</tbody></table>')
        out.append(''.join(t))
        continue

    if ln.startswith('- '):                                   # bullet list
        items = []
        while i < len(lines) and lines[i].startswith('- '):
            items.append(lines[i][2:]); i += 1
        out.append('<ul>' + ''.join(f'<li>{inline(x)}</li>' for x in items) + '</ul>')
        continue

    if ln.startswith('> '):                                   # blockquote
        buf = []
        while i < len(lines) and lines[i].startswith('> '):
            buf.append(lines[i][2:]); i += 1
        out.append('<blockquote>' + inline(' '.join(buf)) + '</blockquote>')
        continue

    if not ln.strip():
        i += 1
        continue

    buf = []                                                  # paragraph
    while i < len(lines) and lines[i].strip() and not re.match(r'^(#{1,3} |\| |- |```|> )', lines[i]):
        buf.append(lines[i]); i += 1
    out.append('<p>' + inline(' '.join(buf)) + '</p>')

doc = ('<!DOCTYPE html><html><head><meta charset="utf-8">'
       f'<title>{html.escape(lines[0].lstrip("# "))}</title><style>{CSS}</style></head>'
       '<body>' + '\n'.join(out) + '</body></html>')
open(dst, 'w', encoding='utf-8').write(doc)
print(f'wrote {dst} ({len(doc)} chars, {len(out)} blocks)')
