# Amplitude Sweep Results — FPGA_simulator (2026-08-22, revised range)

在 `D:\TIADC\FPGA_simulator` 上跑完整固件 `adc -cal` 流程，验证 299.375 MHz
参考波形在不同 dither 幅度下的系统行为。模拟器运行的是真实生产固件
（`adc -cal` + `adc -cal export`），不是 Python 闭环。

## 配置

- 主 tone：299.375 MHz（相干，29.9375 cycles/event）
- DAC：2600 MS/s，ADC：1300 MS/s
- dither 周期：260 DAC samples = 130 ADC samples
- pulse：16/32/16 DAC samples（raised cosine）
- seed：20260725
- 模拟器模式：bench-like
  - `VB_BENCH_MODE=1`
  - `VB_PULSE_DISPERSION_ALPHA=0.35`
  - `VB_SKEW_DRIFT_SIGMA_PS=2.0`
  - `VB_NOISE_CODES=10`
  - `VB_REFERENCE_GAIN=0.02`

## 汇总表（新范围）

| Amplitude (LSB) | Timing Corr | Tone RMSE (codes) | Offset Corr (codes) | Gain Corr | Final Skew (ps) | Dither A/B Valid | Dither Skew Valid | Perf Valid |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 0.9995 | 10.15 | +5.23 | 0.9770 | -5.21 | 98.5% | 0% | 1 |
| 500 | 0.9994 | 10.65 | +5.23 | 0.9770 | -5.21 | 100% | 0% | 1 |
| 750 | 0.9994 | 11.40 | +5.73 | 1.0157 | -5.19 | 83.8% | 0% | 1 |
| 1000 | 0.9992 | 12.33 | +5.52 | 0.9637 | +5.57 | 81.3% | 0.7% | 1 |
| 2000 | 0.9984 | 16.92 | +5.76 | 0.9605 | -4.93 | 70% | 0% | 1 |

## 趋势

- **Tone 拟合质量随 dither 幅度增大而下降**：
  - timing correlation：0.9995 → 0.9984
  - tone RMSE：10.15 → 16.92 codes
- **Dither 事件检测在 500 LSB 达到最佳，之后下降**：
  - 250 LSB：98.5%
  - 500 LSB：100%
  - 750 LSB：83.8%
  - 1000 LSB：81.3%
  - 2000 LSB：70%
- **Dither fine-skew 基本保持 INVALID**（0–0.7%），符合当前链路 dispersion 预期。
- **Offset/Gain 校正变化不大**。
- **Final skew 都在 ±5 ps 量级**，没有明显单调趋势。

## 初步解读

在虚拟 bench-like 模型下：

- **500 LSB 是当前模型里的最佳工作点**：tone 拟合好、事件检测 100%；
- 250 LSB 已经接近检测下限（98.5%）；
- 750 LSB 以上事件检测开始下降，tone RMSE 也开始上升；
- 这支持“低幅度 impulse、低干扰”的 selling point，但最终以上板为准。

## 文件位置

- 详细 CSV 导出：
  `D:\TIADC\FPGA_simulator\amplitude_sweep_data\amp_250/...`
  `D:\TIADC\FPGA_simulator\amplitude_sweep_data\amp_500/...`
  `D:\TIADC\FPGA_simulator\amplitude_sweep_data\amp_750/...`
  `D:\TIADC\FPGA_simulator\amplitude_sweep_data\amp_1000/...`
  `D:\TIADC\FPGA_simulator\amplitude_sweep_data\amp_2000/...`
- 机器可读汇总：
  `D:\TIADC\FPGA_simulator\amplitude_sweep_results.json`
- 仓库根目录副本：
  `amplitude_sweep_results.json`
- 上板操作清单：
  `calibration_out/amplitude_sweep/BENCH_OPERATION.md`

## 下一步

- 上板优先验证 **250 / 500 / 750 LSB** 三个点；
- 若真实板 trend 与模拟器一致，则 amplitude sweep 结论可以直接作为论文
  “low-amplitude impulse advantage” 的证据；
- 若不一致，以真实板为准，并回到模拟器调整模型参数。
