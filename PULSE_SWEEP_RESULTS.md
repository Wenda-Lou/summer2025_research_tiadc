# Pulse Width / Duty-Cycle Sweep Results — FPGA_simulator (2026-08-22)

在 `D:\TIADC\FPGA_simulator` 上跑完整固件 `adc -cal` 流程，研究第二个维度：

> 在保持可检测性的前提下，impulse 能多窄 / 多稀疏？

所有波形使用：

- 主 tone：299.375 MHz（相干）
- 幅度：**500 LSB**（amplitude sweep 的最佳点）
- DAC/ADC：2600 / 1300 MSPS
- 模拟器模式：bench-like（dispersion + noise）

## 1. Pulse Width 系列（固定周期 260 DAC）

| Label | Edge (DAC) | Top (DAC) | Pulse (DAC) | Duty | Timing Corr | Tone RMSE | Final Skew (ps) | Dither A/B Valid |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| e4t4 | 4 | 4 | 12 | 4.6% | 0.9995 | 10.09 | -5.19 | 98.5% |
| e8t4 | 8 | 4 | 20 | 7.7% | 0.9995 | 10.13 | -5.21 | 96.9% |
| e8t8 | 8 | 8 | 24 | 9.2% | 0.9995 | 10.18 | -7.11 | 98.9% |
| e16t8 | 16 | 8 | 40 | 15.4% | 0.9995 | 10.26 | -7.11 | 100% |
| e16t32 | 16 | 32 | 64 | 24.6% | 0.9994 | 10.65 | -5.21 | 100% |
| e32t32 | 32 | 32 | 96 | 36.9% | 0.9994 | 10.86 | -5.19 | 100% |

### 观察

- **窄到 12 DAC（6 ADC samples）仍然可检测**（98.5%）；
- 脉冲越宽，tone RMSE 略升（10.09 → 10.86），但变化不大；
- 事件检测在 e16t8 以上达到 100%。

## 2. Duty-Cycle / Sparsity 系列

| Label | Edge/Top (DAC) | Period (DAC) | Pulse (DAC) | Duty | Timing Corr | Tone RMSE | Final Skew (ps) | Dither A/B Valid |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| p260_e16t32 | 16/32 | 260 | 64 | 24.6% | 0.9994 | 10.65 | -5.21 | 100% |
| p520_e16t32 | 16/32 | 520 | 64 | 12.3% | 0.9995 | 10.33 | -0.87 | 58.5% |
| p1040_e16t32 | 16/32 | 1040 | 64 | 6.2% | 0.9995 | 10.11 | -7.11 | 0% |
| p520_e8t8 | 8/8 | 520 | 24 | 4.6% | 0.9995 | 10.07 | -7.11 | 83.3% |
| p1040_e8t8 | 8/8 | 1040 | 24 | 2.3% | 0.9995 | 10.02 | -5.21 | 0% |

### 观察

- **周期 260 是可靠工作点**；
- 周期 520 时检测开始下降（58.5% / 83.3%）；
- 周期 1040 时检测降到 **0%**，虽然 tone 拟合很好，但 impulse 已经无法检测；
- 也就是说，**“更稀疏”有明确上限**：在这个 bench-like 模型下，事件周期不能超过 520 DAC samples。

## 3. 结论

- **Pulse 可以很窄**：12 DAC（6 ADC samples）仍保持 ~98% 检测；
- **Pulse 不能太稀疏**：duty 低于约 12%（周期 520 DAC）时检测开始不稳，低于约 6%（1040 DAC）时完全失效；
- 这为“low-cost、minimally intrusive impulse”提供了第二个证据维度：
  - 可以做到很窄 → 对主信号占用小；
  - 但不能无限稀疏 → 需要保留足够的事件能量/周期密度供检测。

## 文件位置

- 详细 CSV 导出：
  `D:\TIADC\FPGA_simulator\pulse_sweep_data\<label>/...`
- 机器可读汇总：
  `D:\TIADC\FPGA_simulator\pulse_sweep_results.json`
- 仓库根目录副本：
  `pulse_sweep_results.json`
- 波形生成脚本：
  `calibration_loop/sweep_pulse.py`
- 模拟器运行脚本：
  `D:\TIADC\FPGA_simulator\tests\pulse_sweep.py`

## 下一步

- 上板优先验证：
  - `e4t4`（最窄）
  - `e8t8`（窄 + 中周期）
  - `e16t32`（baseline）
  - `p520_e8t8`（稀疏极限）
- 如果板上趋势与模拟器一致，可以把“narrow + sparse impulse”作为论文 selling point 的量化证据。
