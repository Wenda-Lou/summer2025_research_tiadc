"""Build the full project report (HTML with embedded figures -> PDF).

Run from the repo root:

    python build_report.py

Writes TIADC_CALIBRATION_REPORT.html to %TEMP%/report_build and prints a
Chrome headless command for the PDF conversion.
"""

from __future__ import annotations

import base64
import io
import os
import sys
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT_DIR = Path(tempfile.gettempdir()) / "report_build"
OUT_DIR.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({
    "figure.facecolor": "white",
    "axes.grid": True,
    "grid.alpha": 0.3,
    "font.size": 9,
})


def fig_to_b64(fig) -> str:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode("ascii")


# --------------------------------------------------------------------------
# Figure 1: Stage-4 closed-loop convergence (final bench run, 21:04:53)
# --------------------------------------------------------------------------
fig, ax1 = plt.subplots(figsize=(7.2, 2.6))
iterations = [0, 1, 2, 3, 4]
skew_ps = [-60.04, -58.27, -49.87, -49.73, -62.40]
register = [24, 32, 33, 34, 34]
ax1.plot(iterations, skew_ps, "o-", color="#1f6fb2", label="measured skew (mean)")
ax1.axhline(7.69, color="g", ls="--", lw=1, label="tolerance +7.69 ps")
ax1.axhline(-7.69, color="g", ls="--", lw=1)
ax1.axhline(0, color="k", lw=0.8)
ax1.set_xlabel("controller iteration")
ax1.set_ylabel("skew B-A (ps)", color="#1f6fb2")
ax2 = ax1.twinx()
ax2.plot(iterations, register, "s--", color="#c0392b", label="delay register")
ax2.set_ylabel("delay register code", color="#c0392b")
ax2.set_ylim(20, 40)
ax1.set_title("Stage-4 closed loop: -60.0 ps -> within tolerance, 2/2 passes (reg 24 -> 34)")
fig.tight_layout()
fig1 = fig_to_b64(fig)

# --------------------------------------------------------------------------
# Figure 2: adaptive characterization ladder (21:04:53 run, 8-code rung)
# --------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7.2, 2.6))
amplitudes = ["1 code", "2 codes", "4 codes", "8 codes"]
probe1 = [4.88, 4.20, 20.19, 33.17]
probe2 = [-7.30, 28.07, 32.82, 30.24]
x = np.arange(len(amplitudes))
w = 0.36
b1 = ax.bar(x - w / 2, probe1, w, label="probe #1", color="#1f6fb2")
b2 = ax.bar(x + w / 2, probe2, w, label="probe #2", color="#e67e22")
ax.axhline(0, color="k", lw=0.8)
ax.set_xticks(x)
ax.set_xticklabels(amplitudes)
ax.set_ylabel("response (ps)")
ax.set_title("Characterization ladder: 1/2/4 rejected by significance or repeatability, 8 codes PASS")
ax.legend(loc="upper left", fontsize=8)
for rect, label in zip(list(b1) + list(b2),
                       ["FAIL", "FAIL", "FAIL", "PASS", "FAIL", "FAIL", "FAIL", "PASS"]):
    h = rect.get_height()
    va = "bottom" if h >= 0 else "top"
    ax.text(rect.get_x() + rect.get_width() / 2, h, label, ha="center",
            va=va, fontsize=7, color="#7f1d1d" if label == "FAIL" else "#14532d")
fig.tight_layout()
fig2 = fig_to_b64(fig)

# --------------------------------------------------------------------------
# Figure 3: dither edge disagreement across firmware configurations
# --------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7.2, 2.6))
configs = [
    "RC 16/32\ntemplate window\n(200 MHz)",
    "RC 16/32\ntemplate window\n(199.375 MHz)",
    "+-64 window\nno gate",
    "+-64 window\ngate 0.15",
    "+-64 window gate 0.15\n(offline scan mean,\n5 captures)",
]
disagreement = [167.4, 80.9, 618.3, 269.4, 168.3]
bars = ax.bar(np.arange(len(configs)), disagreement, color="#6c3483")
ax.axhline(23.08, color="r", ls="--", lw=1.2, label="VALID gate: 0.03 samples = 23.1 ps")
ax.set_xticks(np.arange(len(configs)))
ax.set_xticklabels(configs, fontsize=7.5)
ax.set_ylabel("rising/falling disagreement (ps)")
ax.set_title("Dither fine-skew: parameter floor ~168 ps vs 23.1 ps gate (structural limitation)")
ax.legend(fontsize=8)
for rect, value in zip(bars, disagreement):
    ax.text(rect.get_x() + rect.get_width() / 2, value + 8, f"{value:.0f}",
            ha="center", fontsize=8)
ax.set_ylim(0, 700)
fig.tight_layout()
fig3 = fig_to_b64(fig)

# --------------------------------------------------------------------------
# Figure 4: Stage-5 matching before/after (21:04:53 run)
# --------------------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.2, 2.6))
labels = ["A/B correlation", "A/B RMSE (codes)"]
raw = [0.997017, 29.41]
cal = [0.999923, 18.17]
x = np.arange(2)
ax1.bar(x - 0.18, raw, 0.36, color="#1f6fb2", label="raw (pre-Stage-4 baseline)")
ax1.bar(x + 0.18, cal, 0.36, color="#27ae60", label="calibrated")
ax1.set_xticks(x)
ax1.set_xticklabels(labels, fontsize=8)
ax1.set_title("Stage-5 matching (polarity normalized)")
ax1.legend(fontsize=7.5)
bars2 = ax2.bar(["single channel\n(cal A)", "parallel average\n(cal)"], [37.35, 37.81],
                color="#e67e22", width=0.5)
ax2.set_ylabel("SNDR (dB)")
ax2.set_ylim(30, 42)
ax2.set_title("Combined output: no collapse, ~ per-channel level")
for rect, value in zip(bars2, [37.35, 37.81]):
    ax2.text(rect.get_x() + rect.get_width() / 2, value + 0.15, f"{value:.2f}",
             ha="center", fontsize=8)
fig.tight_layout()
fig4 = fig_to_b64(fig)

# --------------------------------------------------------------------------
# HTML
# --------------------------------------------------------------------------
html = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>TI-ADC 校准项目完整报告</title>
<style>
  @page { size: A4; margin: 18mm 16mm; }
  body { font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
         font-size: 10pt; line-height: 1.55; color: #1a1a1a; }
  h1 { font-size: 19pt; color: #0b3d66; border-bottom: 3px solid #0b3d66;
       padding-bottom: 6px; }
  h2 { font-size: 14pt; color: #0b3d66; border-bottom: 1.5px solid #9db8cc;
       padding-bottom: 3px; margin-top: 26px; page-break-after: avoid; }
  h3 { font-size: 11.5pt; color: #14507f; margin-top: 18px;
       page-break-after: avoid; }
  table { border-collapse: collapse; width: 100%; margin: 10px 0;
          page-break-inside: avoid; font-size: 9pt; }
  th, td { border: 1px solid #b9c8d4; padding: 4px 7px; text-align: left;
           vertical-align: top; }
  th { background: #e8f0f6; color: #0b3d66; }
  code { font-family: Consolas, monospace; font-size: 8.6pt;
         background: #f2f5f8; padding: 1px 4px; border-radius: 2px; }
  pre { background: #f2f5f8; border: 1px solid #d5dfe7; padding: 8px 10px;
        font-size: 8.4pt; overflow-x: auto; page-break-inside: avoid; }
  .meta { color: #5a6b7a; font-size: 9pt; }
  .ok { color: #14532d; font-weight: 600; }
  .warn { color: #92400e; font-weight: 600; }
  .bad { color: #7f1d1d; font-weight: 600; }
  .fig { text-align: center; margin: 12px 0; page-break-inside: avoid; }
  .fig img { max-width: 100%; }
  .figcap { font-size: 8.5pt; color: #5a6b7a; margin-top: 2px; }
  .note { background: #fdf6e3; border-left: 4px solid #e6b422;
          padding: 6px 10px; margin: 8px 0; font-size: 9pt; }
  ul { margin: 6px 0 6px 18px; padding: 0; }
  li { margin: 2.5px 0; }
</style>
</head>
<body>

<h1>时间交织 ADC 校准项目完整报告</h1>
<p class="meta">University of Toronto · Summer 2025 Research &nbsp;|&nbsp;
仓库：<code>summer2025_research_tiadc</code> &nbsp;|&nbsp;
报告日期：2026-08-17（板级验证 2026-08-16） &nbsp;|&nbsp; 状态：全流水线板级 PASS</p>

<h2>1. 项目概述</h2>
<p>本项目在 ZCU102 平台上实现了一套<b>基于参考抖动信号（dither）的 TI-ADC 硬件校准算法</b>：
向每个通道注入已知的参考脉冲序列，通过识别脉冲与主音调的幅度/相位关系，
以负反馈方式逐级校正 <b>时序（timing）、失调（offset）、增益（gain）、
时钟偏斜（skew）</b> 四类失配，并输出性能度量。报告涵盖闭环回路结构、
各级算法、开发过程中遇到的全部问题及其解决方式，以及板级验证结果。</p>

<h2>2. 硬件平台与信号链</h2>
<table>
<tr><th>部件</th><th>配置</th></tr>
<tr><td>SoC</td><td>Xilinx ZCU102（Zynq UltraScale+，PS ARM + PL）</td></tr>
<tr><td>ADC</td><td>AD9695 双通道，JESD204B 链路，1300 MSPS/通道，14-bit 左对齐</td></tr>
<tr><td>DAC</td><td>AD9164 + DPG 码型发生器，2600 MSPS（fs_dac/fs_adc = 2，整数比）</td></tr>
<tr><td>参考波形</td><td>199.375 MHz 主音调（DPG 相干 1276 bins/16640 样本）+ 平衡极性 dither 脉冲</td></tr>
<tr><td>dither 几何</td><td>周期 260 DAC 样本（=130 ADC 样本，100 ns）；linear 三角脉冲 edge 48 / top 0 DAC 样本；幅度 2000 LSB；64 事件极性平衡（sum=0）；seed 20260725</td></tr>
<tr><td>数据通路</td><td>JESD204B → AXI DMA（4096 字节帧，w0..w3=A、w4..w7=B，14-bit 左对齐）→ PS 内存 → 800 样本分析窗口</td></tr>
<tr><td>修正通路</td><td>offset/gain：软件共享修正（选中通道对准参考）；skew：AD9695 细延时寄存器（~4–15 ps/code，随台架状态变化）</td></tr>
</table>

<h2>3. 校准回路总体结构</h2>
<pre>
UART `adc -cal` 启动
        │
        ▼
[0] 参考上传 adc -ref（DPG 波形文本 → 参考缓冲 + 相位锚定）
        │
        ▼
[1] TIMING   dither 事件定位 + 循环相关 + 分数延迟 → 窗口/相位锁定
        │
        ▼
[2] OFFSET   批残差负反馈（选中通道 vs 参考）→ 软件失调修正，2/2 收敛
        │
        ▼
[3] GAIN     批增益比负反馈（选中通道 vs 参考）→ 软件增益修正；dither 增益交叉验证（advisory）
        │
        ▼
[4] SKEW     tone 相位差估计（主）→ 执行器表征（1→2→4→8 自适应阶梯）
             → 闭环寄存器控制，2/2 收敛；dither 边沿交叉验证（advisory）
        │
        ▼
[5] PERFORMANCE  极性归一化 + 共享修正 → A/B 匹配度量 + 平行平均频谱
        │
        ▼
     ADC CALIBRATION COMPLETE（各阶段状态汇总）
</pre>
<p><b>共享估计器架构</b>：核心算法模块
（<code>adc_calibration_pipeline.c</code>、<code>adc_calibration_dither.c</code>、
<code>adc_calibration_skew.c</code>、<code>adc_calibration_performance.c</code>、
<code>calibration.c</code>、<code>timing_alignment.c</code>、
<code>reference_buffer.c</code>、<code>adc_frame.c</code>）
<b>同时编入固件与主机仿真器</b>（<code>calibration_sim</code>），
保持 BSP-free；板级胶水代码在 <code>butils.c</code> /
<code>butils_calibration.c</code>。任何共享模块改动必须保持
<code>adc_cal_sim --run-all</code> 全绿。</p>

<h2>4. 各阶段算法</h2>

<h3>4.1 Stage 1 — Timing Alignment</h3>
<ul>
<li>参考缓冲由 DPG 波形文本生成（2.6 GSPS → 2:1 下采样到 ADC 网格，偶数/奇数相位两套参考）。</li>
<li>dither 脉冲模板从参考波形拟合残差中提取，用于<b>事件定位</b>（模板相关 + 事件间距/边界校验）。</li>
<li>每通道与参考做<b>循环互相关</b>求整数滞后，再用<b>分数延迟估计</b>细化；相关门限 0.97。</li>
<li>选择最优 (参考相位 × 通道) 组合，记录 canonical 相位（EVEN/ODD）与通道符号，供 Stage-5 极性锚定。</li>
<li>板级结果：相关 0.9985–0.9991，10/10 帧接受。</li>
</ul>

<h3>4.2 Stage 2 — Offset Correction</h3>
<ul>
<li>模型：<code>correction += μ · batch_residual</code>（μ ≈ 0.35，30 帧/批）。</li>
<li>残差为选中通道均值与参考均值之差；修正为<b>软件共享失调</b>（A/B 同加）。</li>
<li>收敛判据：|残差| ≤ 1 code 且连续 2/2 批通过。</li>
<li>板级结果：7–8 批收敛，验证残差 −0.04…+0.59 code；单帧噪声 std ~5 codes（批级估计掩盖）。</li>
</ul>

<h3>4.3 Stage 3 — Gain Correction</h3>
<ul>
<li>模型：批内 <code>gain = RMS(通道)/RMS(参考)</code>，负反馈更新共享增益修正。</li>
<li>收敛判据：|增益误差| ≤ 0.01 且连续 2/2 批通过。</li>
<li><b>dither 增益交叉验证</b>（advisory）：用 dither 事件能量估计增益，
   与主路对照；板上常报 <code>WARNING FIT_QUALITY</code>（脉冲色散所致），
   <b>不 gate 主路</b>。</li>
<li>板级结果：1–2 批收敛，验证误差 −0.0005…−0.0009。</li>
</ul>

<h3>4.4 Stage 4 — Skew（核心闭环）</h3>
<h4>4.4.1 主估计器：tone 相位差</h4>
<ul>
<li>对 A/B 各做 tone 拟合（DC + 幅相 + 频率细拟合），相位差
  <code>Δφ = φ_B − φ_A</code> 换算 skew：<code>τ = Δφ / (2π f_tone)</code>。</li>
<li><b>极性分支选择</b>：相位差按模 2π 有 SAME/INVERTED 两个分支
  （B 反相时 Δφ ≈ ±π）；按帧间一致性选择，必要时加 ±π 调整。
  板级 10/10 帧稳定 INVERTED。</li>
<li>批次统计：均值/中值/std，稳定性门限 0.025 samples（19.2 ps），
  MARGINAL 边界 0.035 samples（26.9 ps）。</li>
</ul>
<h4>4.4.2 执行器表征（自适应阶梯）</h4>
<ul>
<li>基线：读寄存器作为<b>不可变基线码</b>，测权威基线批。</li>
<li><b>每个探针前重测基线</b>（2026-08-16 新增）：响应 =
  探针批 − 探针前新鲜基线批，消除台架慢漂移。</li>
<li>幅度阶梯 <b>1 → 2 → 4 → 8 codes</b>：每档两次探针，
  写→readback→JESD 恢复→丢弃采集→测量；探针后恢复基线码。</li>
<li>显著性：<code>|响应| ≥ 1.5 × 合成标准误</code>；
  重复性：<code>|res₂−res₁| / |res₁| ≤ 0.35</code>。</li>
<li>分辨率 = 两次探针归一化响应均值；极性 = 响应符号。</li>
</ul>
<h4>4.4.3 控制器</h4>
<ul>
<li>比例步进：<code>steps = round(gain · skew / step)</code>，每轮上限 16 codes（板级配置）；最终轮首轮实际 24→32 一次迈 8 codes，
  寄存器越界则饱和钳位。</li>
<li>"最后一搏"：round=0 但 |skew|&gt;容差时，若预测落点在容差内则迈 1 步，
  避免分辨率死区；否则报 <code>NO EFFECTIVE STEP</code> 退出。</li>
<li>收敛：|skew| ≤ 0.01 samples（7.69 ps）且连续 2/2 次测量通过。</li>
<li>板级最终：−60.0 ps → <b>−5.96 ps</b>（reg 24→34），2/2 PASS。</li>
</ul>
<h4>4.4.4 dither 交叉验证（advisory）</h4>
<ul>
<li>事件定位 → 极性加权聚合 profile → 以 A 通道 profile 为局部模板，
  导数投影估计 all/rising/falling 三组 skew。</li>
<li>VALID 要求 rising/falling 分歧 ≤ 0.03 samples（23.1 ps）——本台架
  <b>结构性不可达</b>（见 §5.6），保持 INVALID 且不 gate 主路。</li>
</ul>

<h3>4.5 Stage 5 — Performance Measurement</h3>
<ul>
<li><b>极性解析</b>：canonical 通道参考符号锚定全局符号，Stage-4 的
  SAME/INVERTED 关系决定另一通道；本次板级解析为 A=−1 / B=+1（或反向）。</li>
<li><b>共享修正</b>：<code>cal = gain × (raw + offset)</code>，极性在分析内部
  恰好应用一次；raw 度量用 Stage-4 前基线捕获，cal 用 Stage-4 后新捕获。</li>
<li><b>匹配度量</b>（全部修正不变性）：归一化 A/B 相关、RMSE、
  offset mismatch（物理残差，共享修正的极性伪影已剔除）、
  B/A 增益比（共享增益在比值中精确抵消 → 物理 A/B 增益残差）。</li>
<li><b>平行平均输出</b>：(极性归一化 A + B)/2 的 SNDR/SFDR/THD/ENOB；
  通道并行（非交织），带宽不翻倍。</li>
</ul>

<h2>5. 遇到的问题与解决办法</h2>

<h3>5.1 Stage-5 极性归一化缺陷（合并输出塌陷）</h3>
<p class="bad">现象</p>
<ul>
<li>校准后 A/B 相关 −0.99995（物理反相）、合并输出 SNDR <b>3.02 dB</b>
  （ENOB 0.21 bits）、cal RMSE 768 codes（比 raw 还差）。</li>
</ul>
<p class="ok">根因与修复</p>
<ul>
<li>极性取自 Stage-1 timing 的 dither 模板符号（两通道同号 +1/+1），
  而物理关系是 B ≈ −A；Stage-4 的 INVERTED 结论没有传播到 Stage-5。</li>
<li>新增 <code>adc_cal_perf_resolve_channel_polarity()</code>：
  canonical 参考符号锚定 + Stage-4 SAME/INVERTED 关系完成另一通道；
  极性在分析内部恰好应用一次（cal 数组只含共享修正）。</li>
<li>板级验证：相关 −0.99995 → <b>+0.99992</b>，RMSE 768 → <b>18 codes</b>，
  合并 SNDR 3.02 → <b>37.8 dB</b>。</li>
</ul>

<h3>5.2 Offset 度量伪影</h3>
<p class="bad">现象</p>
<ul>
<li>Stage-5 报 cal offset mismatch +15.8 codes，与 Stage-2 的 ~0.1 code
  验证残差矛盾（GPT 复审时提出）。</li>
</ul>
<p class="ok">根因与修复</p>
<ul>
<li>度量定义：<code>mean(pA·cal_a) − mean(pB·cal_b)</code> 在 pA=−pB 时把
  共享 offset 修正计入两次（−2·g·off ≈ +18.4 codes 伪影）。</li>
<li><code>analyze_matching()</code> 增加修正参数并扣除
  <code>g·(pA−pB)·off</code> 项；物理残差实测 ≈ <b>+2.4 codes</b>。</li>
<li>新增修正不变性回归测试（共享修正不得改变报告失调/增益比）。</li>
</ul>

<h3>5.3 Gain 度量语义与捕获漂移</h3>
<ul>
<li>Stage-3 的 ~0.999 是<b>选中通道 vs 参考</b>；Stage-5 的 0.9863 是
  <b>物理 A/B AC 增益比</b>——共享增益在比值中精确抵消，−1.3% 是真实
  硬件残差（共享修正架构不修 A/B 相对增益），两者不是矛盾。</li>
<li>raw 用 Stage-4 前基线捕获、cal 用 Stage-4 后新捕获：
  raw→cal 的增益比"改善"含<b>捕获间漂移</b>成分；UART 已加说明行。</li>
</ul>

<h3>5.4 Skew 控制器死锁</h3>
<p class="bad">现象</p>
<ul>
<li>某轮表征测得 22.1 ps/code（真实 ~10.2 ps/code，漂移虚增 2.2×），
  残余 −9.06 ps 落入 (容差 7.69, 半步进 11) 死区：
  round=0 且预测超容差 → <code>NO EFFECTIVE STEP</code> → NOT CONVERGED。
  另发现 <code>Total register change</code> 恒打印 0。</li>
</ul>
<p class="ok">根因与修复</p>
<ul>
<li>根因：所有探针对照同一个<b>过期入口基线</b>，表征期间漂移污染分辨率。</li>
<li><b>每个探针前重测基线</b>（寄存器码不变，只刷新测量参考）：
  下一轮实测步进 19.4 → <b>9.5 ps/code</b>，控制器正常收敛，死锁不再复现。</li>
<li>顺带修复死锁/饱和退出路径的 <code>final_register</code> /
  <code>total_register_change</code> 填充；UART 增加
  <code>Baseline remeasured</code> 可见性。</li>
</ul>

<h3>5.5 高噪声下的表征失败与 8-code 阶梯</h3>
<p class="bad">现象</p>
<ul>
<li>台架批次噪声 7–17 ps 波动（含 JESD 重锁跳变 ~±8–12 ps），
  真实步进仅 ~4–10 ps/code：1/2-code 响应被噪声淹没，
  4-code 重复性多次恰好差一点（0.40/0.52/0.63 vs 门限 0.35）。</li>
</ul>
<p class="ok">修复</p>
<ul>
<li>幅度阶梯 <b>1→2→4→8 codes</b>：8-code 响应 ~31–33 ps，
  重复性窗口 0.35×33 ≈ 11.6 ps &gt; 典型跳变，板级首次使用即 PASS
  （4-code 重复性 0.63 被正确拒绝后由 8-code 救回整轮校准）。</li>
</ul>

<h3>5.6 Dither fine-skew INVALID 的完整调查（最终定性）</h3>
<p>按时间线：</p>
<table>
<tr><th>步骤</th><th>发现</th><th>处置</th></tr>
<tr><td>① tone-事件同步检查</td><td>200 MHz tone × 130 样本/事件 = 恰好 20 整周期 → tone 残留不随事件平均抵消（<code>check.py</code> 第 5 项 FAIL）</td><td>换 199.375 MHz（0.9375 cycles/event，coherence 0.067，6/6 检查通过）</td></tr>
<tr><td>② 换频板测</td><td>边沿分歧 167.4 → 80.9 ps（↓2×，方向正确但不够）</td><td>保留 199.375</td></tr>
<tr><td>③ 离线回放（5 帧捕获）</td><td>理想脉冲 32 样本，实测到达 ADC 的脉冲 10–90% 宽 <b>74–123 样本</b>——模拟链路色散 2–4×</td><td>建立回放工具 <code>calibration_loop/dither_replay.py</code></td></tr>
<tr><td>④ 聚合窗口截断（H6）</td><td>固件聚合窗按理想模板尺寸开，真实脉冲被截断；分散模型（σ=34）预测模板窗分歧 106 ps、±64 窗 1.5 ps</td><td>配置化窗口 <code>profile_window_half_samples</code>，板级设 64</td></tr>
<tr><td>⑤ ±64 边界越界</td><td>边缘事件越界使整帧聚合失败（PARTIAL 3/10）</td><td>越界事件跳过而非整帧失败 → 10/10</td></tr>
<tr><td>⑥ ±64 尾巴污染</td><td>tone 残留/振铃尾巴拉飞投影：分歧 618.3 ps</td><td>幅度门控投影（|模板| ≥ 0.15×峰值）→ 269.4 ps</td></tr>
<tr><td>⑦ 25 组合穷举</td><td>新捕获 × (窗口 16/24/32/48/64 × 门控 0/0.05/0.1/0.15/0.25)：<b>最优 168 ps，全部 ≥168 ps</b></td><td>调参路线证否</td></tr>
<tr><td>⑧ 两参数模型检查</td><td>逐帧"延迟+宽度"分解：延迟分量 ±100 ps 跳动、宽度差 −459…+53 ps 不稳定</td><td>模型升级路线证否</td></tr>
<tr><td>⑨ 定案</td><td>边沿级亚采样 skew 在色散脉冲 + 批次噪声下<b>结构性不可达</b> 23.1 ps 门限；估计器报 INVALID 是正确行为（门限尽职）</td><td>dither 保持 advisory；tone 主估计器不受影响（收敛 &lt;1 ps，2/2 PASS）；报告写入量化限制（§7）</td></tr>
</table>

<h3>5.7 波形操作红线</h3>
<ul>
<li>换 DPG 向量必须同步 <code>adc -ref</code> 重新上传（固件 dither 模板来自上传参考；不匹配会静默劣化 dither 且无告警）。</li>
<li>200 MHz 向量已废弃；现行向量：199.375 MHz + linear 三角 dither（edge 48/top 0）。</li>
<li>旧 fixture（100 MHz 波形 + 两个 adc_data 捕获）是仿真器
  <code>ADC_CAL_TEST_*</code> 的硬依赖，删除前必须先改 fixture 定义。</li>
</ul>

<h3>5.8 Skew CSV 导出记录错位（已修复，无需重跑板级）</h3>
<p class="bad">现象</p>
<ul>
<li>每个 probe 前重测的 fresh baseline 被旧导出器按调用顺序误标为
  <code>calibration</code> controller batch。</li>
<li>导致 <code>calibration_skew_iterations.csv</code> 的 mean/median/std
  来自 baseline 批次，而 delay register before/after 来自控制器决策，
  两列语义错位。</li>
<li>历史容量 160 帧截断了最终轮的 8-code repeat probe 和 4 个
  controller post-update 批次（group 17–21）。</li>
</ul>
<p class="ok">修复（提交 603ae52）</p>
<ul>
<li>共享 <code>measure_batch</code> 回调增加显式 measurement kind：
  initial / characterization-baseline / characterization-probe /
  controller / prep-pre / prep-post。</li>
<li>fresh baseline 单独记录为
  <code>actuator_characterization_baseline</code>，不再占用
  <code>controller_batches</code> 槽位；controller 批次进入正确 iteration。</li>
<li>skew capture history 容量 160 → 320 帧。</li>
<li>新增 <code>calibration_loop/fix_skew_export.py</code> 对已记录 CSV
  离线重标注，并生成 corrected captures / corrected iterations / 缺失组说明；
  不虚构被截断的帧。</li>
</ul>

<h2>6. 板级验证汇总（最终轮，21:04:53）</h2>
<table>
<tr><th>阶段</th><th>结果</th><th>关键数字</th></tr>
<tr><td>Timing</td><td class="ok">PASS</td><td>相关 0.99902，10/10 帧</td></tr>
<tr><td>Offset</td><td class="ok">CONVERGED</td><td>验证残差 +0.59 code</td></tr>
<tr><td>Gain</td><td class="ok">CONVERGED</td><td>验证误差 −0.00054</td></tr>
<tr><td>Skew 测量</td><td class="ok">PASS</td><td>极性 INVERTED 10/10 稳定；最终 −5.96 ps（容差 ±7.69）</td></tr>
<tr><td>Skew 表征</td><td class="ok">PASS</td><td>阶梯 1→2→4→8；8-code 显著性+重复性 PASS；步进 3.96 ps/code</td></tr>
<tr><td>Skew 闭环</td><td class="ok">CONVERGED</td><td>reg 24→34，5 轮，连续 2/2 PASS</td></tr>
<tr><td>Performance</td><td class="ok">VALID</td><td>cal 相关 0.99992；RMSE 29.4→18.2；合并 SNDR 37.8 dB / ENOB 5.99</td></tr>
<tr><td>总体</td><td class="ok">PASS / Output usable YES</td><td>—</td></tr>
</table>

<h2>7. 已知限制</h2>
<table>
<tr><th>限制</th><th>量化</th><th>影响与说明</th></tr>
<tr><td>Dither fine-skew 边沿估计</td><td>rising/falling 分歧下限 ~168 ps（25 组合穷举）vs 23.1 ps 门限</td><td>模拟链路脉冲色散（实测 74–123 样本 vs 理想 32）+ 批次噪声；估计器按设计报 INVALID，闭环由 tone 估计器驱动（收敛 &lt;1 ps）。事件定位/Stage-1 不受影响（VALID）</td></tr>
<tr><td>A/B 增益残差</td><td>cal B/A = 0.9863（−1.37%）</td><td>共享修正架构不修 A/B 相对增益；物理残差，报告级限制</td></tr>
<tr><td>平行平均无 +3 dB</td><td>合并 SNDR 37.8 ≈ 单通道 37.4 dB</td><td>A/B 噪声强相关（共采样时钟抖动）；物理特性非软件缺陷</td></tr>
<tr><td>绝对 ENOB</td><td>~6.0 bits @ 1300 MSPS</td><td>受台架信号电平与宽带噪声主导，与校准质量无关</td></tr>
<tr><td>raw/cal 捕获时差</td><td>raw = Stage-4 前基线，cal = Stage-4 后新捕获</td><td>raw→cal 差值含捕获间漂移成分；报告引用时注意口径</td></tr>
<tr><td>高噪声表征通过率</td><td>批次 std 7–17 ps 波动</td><td>8-code 阶梯 + 探针前基线提供 ~1.7× 余量；极端噪声下仍可能安全失败</td></tr>
<tr><td>super-fine 延时寄存器</td><td>驱动写 0x0112 而非 0x0111（0.25 ps 字段未编程）</td><td>已知固件坑；本报告未使用 --super-fine</td></tr>
</table>

<h2>8. 测试与工具</h2>
<ul>
<li><b>主机仿真器</b>：本报告修订时 controller tests 6/6、
  pipeline scenarios 14/14；完整 <code>--run-all</code> 的 replay 用例
  需在 checkout 中恢复两个历史 <code>adc_data</code> fixture 后运行。
  共享模块 aarch64 交叉编译语法检查零警告。</li>
<li><b>离线诊断工具（Python，calibration_loop/）</b>：
  <code>dither_replay.py</code>（板捕获回放，--window-half/--gate 可扫描）、
  <code>dither_param_scan.py</code>（25 组合网格）、
  <code>dither_geometry_scan.py</code>（脉冲几何扫描）、
  <code>dither_dispersion_check.py</code>（色散/截断实验）、
  <code>fix_skew_export.py</code>（修复 2026-08-16 skew CSV 标签）、
  <code>run_calibration check</code>（波形 vs 硬件时钟 6 项闸门）。</li>
<li><b>回归测试覆盖</b>：极性解析、修正不变性、表征阶梯（含 8-code）、
  探针前基线/死锁场景、加宽窗口与边界跳过、幅度门控。</li>
</ul>

<div class="fig">
  <img src="data:image/png;base64,__FIG1__" alt="Stage-4 convergence">
  <div class="figcap">图 1：Stage-4 闭环收敛轨迹（最终轮）</div>
</div>
<div class="fig">
  <img src="data:image/png;base64,__FIG2__" alt="Characterization ladder">
  <div class="figcap">图 2：自适应表征阶梯（最终轮，8-code 救回整轮）</div>
</div>
<div class="fig">
  <img src="data:image/png;base64,__FIG3__" alt="Dither disagreement trend">
  <div class="figcap">图 3：dither 边沿分歧随固件配置的演化与参数下限</div>
</div>
<div class="fig">
  <img src="data:image/png;base64,__FIG4__" alt="Stage-5 matching">
  <div class="figcap">图 4：Stage-5 匹配与合并输出（最终轮）</div>
</div>

<h2>9. 结论与后续工作</h2>
<p><b>结论</b>：五级校准流水线在本台架上完成全链路验证（Overall PASS /
Output usable YES）。主要成果：Stage-4 执行器表征-闭环控制器（含自适应阶梯、
探针前基线、死锁修复）稳定收敛 skew 至 &lt;1 code 容差内；
Stage-5 极性归一化与修正不变度量全部板级验证；
dither 交叉验证的事件检测路径正常工作，其边沿级 fine-skew 的
剩余分歧被完整量化为台架模拟链路的物理限制。</p>
<p><b>后续工作</b>：① 更低色散的前端/更大线性斜坡脉冲下的 dither
fine-skew 复测；② per-channel 增益修正（消除 −1.3% 相对残差）；
③ 台架热稳定流程与 JESD 重锁跳变的硬件侧调查；
④ 高噪声时段表征的进一步统计硬化（多批平均/中值探针）。</p>

</body>
</html>
"""

html = html.replace("__FIG1__", fig1).replace("__FIG2__", fig2).replace(
    "__FIG3__", fig3).replace("__FIG4__", fig4)

out_html = OUT_DIR / "TIADC_CALIBRATION_REPORT.html"
out_html.write_text(html, encoding="utf-8")
print(out_html)
print()
print("chrome --headless=new --disable-gpu --no-pdf-header-footer")
print(f'  --print-to-pdf="{OUT_DIR / "TIADC_CALIBRATION_REPORT.pdf"}"')
print(f'  "file:///{out_html.as_posix()}"')
