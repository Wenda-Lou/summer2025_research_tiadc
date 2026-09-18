"""Scratch check: does a rejected capture consume a sample slot?

``CalibrationLoop.run(N)`` promises N *qualified* samples.  A capture the acceptance
filter refuses is logged for forensics and retried, so it must not shorten the
learning curve; the plot must draw only the qualified samples; and a bench that
rejects everything must stop rather than spin.  All three are checked here against
the bench model, with rejections injected into ``step()`` so no hardware is needed.

    python calibration_out/_qualified_budget_test.py
"""

from __future__ import annotations

import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.axes as maxes          # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.dither import DitherConfig                    # noqa: E402
from calibration_loop.loop import CalibrationLoop, LoopOptions      # noqa: E402
from calibration_loop.simulate import BenchModel                    # noqa: E402

OUT = os.path.join(REPO, "calibration_out", "_qualified_test")


def make_loop(reject_every: int) -> CalibrationLoop:
    """A loop whose every ``reject_every``-th capture comes back rejected."""
    cfg = DitherConfig()
    cfg.validate()
    loop = CalibrationLoop(BenchModel(cfg=cfg), cfg, options=LoopOptions())
    real = loop.step
    count = {"n": 0}

    def flaky():
        count["n"] += 1
        row = real()
        if row is not None and reject_every and count["n"] % reject_every == 0:
            row["rejected"] = "injected: torn frame"
        return row

    loop.step = flaky            # type: ignore[method-assign]
    return loop


def main() -> int:
    checks: list[tuple[str, bool, str]] = []

    loop = make_loop(3)
    loop.run(5, verbose=False)
    accepted = len([r for r in loop.log if not r.get("rejected")])
    print(f"run(5) with every 3rd capture rejected: qualified={loop.qualified} "
          f"captures={loop.captures} rejected={loop.rejected}")
    checks.append(("the budget is delivered in full", loop.qualified == 5,
                   f"qualified={loop.qualified}"))
    checks.append(("rejected captures were retried, not counted",
                   loop.captures > loop.qualified and accepted == 5,
                   f"captures={loop.captures} accepted_rows={accepted}"))

    os.makedirs(OUT, exist_ok=True)
    meta = json.loads(
        loop.save(OUT, stem="t")["meta"].read_text(encoding="utf-8"))
    print(f"meta: samples_qualified={meta['samples_qualified']} "
          f"captures_attempted={meta['captures_attempted']} "
          f"captures_rejected={meta['captures_rejected']}")
    checks.append(("the artifact records the real sample count",
                   meta["samples_qualified"] == 5 and meta["captures_rejected"] >= 1,
                   f"{meta['samples_qualified']} qualified of "
                   f"{meta['captures_attempted']}"))

    lengths: list[int] = []
    original = maxes.Axes.plot

    def spy(self, *args, **kwargs):        # noqa: ANN001
        if len(args) >= 2 and hasattr(args[1], "__len__"):
            lengths.append(len(args[1]))
        return original(self, *args, **kwargs)

    maxes.Axes.plot = spy                  # type: ignore[assignment]
    try:
        path = loop.plot(OUT, stem="t")
    finally:
        maxes.Axes.plot = original         # type: ignore[assignment]
    print(f"series lengths handed to plot(): {sorted(set(lengths))} -> "
          f"{os.path.basename(str(path))}")
    checks.append(("the plot draws only qualified samples",
                   set(lengths) == {5}, f"lengths={sorted(set(lengths))}"))

    broken = make_loop(1)
    broken.run(5, verbose=False)
    cap = LoopOptions().max_consecutive_rejects
    print(f"all captures rejected: qualified={broken.qualified} "
          f"captures={broken.captures} (cap {cap})")
    checks.append(("a bench that rejects everything stops, it does not spin",
                   broken.qualified == 0 and broken.captures == cap,
                   f"captures={broken.captures}"))

    print()
    ok = True
    for name, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name} -- {detail}")
        ok = ok and passed
    print(f"\nQUALIFIED-SAMPLE BUDGET: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
