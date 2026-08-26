# Pulse Width / Duty-Cycle Sweep Results — Simulator + Board (2026-08-25)

在 FPGA_simulator 和真实板级台架上跑完整固件 `adc -cal` 流程，研究第二个维度：

> 在保持可检测性的前提下，impulse 能多窄 / 多稀疏？

所有波形使用：

- 主 tone：299.375 MHz（相干）
- 幅度：**500 LSB**
- DAC/ADC：2600 / 1300 MSPS

---

## 1. 模拟器结果（FPGA_simulator）

### 1.1 Pulse Width 系列（固定周期 260 DAC）

| Label | Edge (DAC) | Top (DAC) | Pulse (DAC) | Duty | Timing Corr | Tone RMSE | Final Skew (ps) | Dither A/B Valid |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| e4t4 | 4 | 4 | 12 | 4.6% | 0.9995 | 10.09 | -5.19 | 98.5% |
| e8t4 | 8 | 4 | 20 | 7.7% | 0.9995 | 10.13 | -5.21 | 96.9% |
| e8t8 | 8 | 8 | 24 | 9.2% | 0.9995 | 10.18 | -7.11 | 98.9% |
| e16t8 | 16 | 8 | 40 | 15.4% | 0.9995 | 10.26 | -7.11 | 100% |
| e16t32 | 16 | 32 | 64 | 24.6% | 0.9994 | 10.65 | -5.21 | 100% |
| e32t32 | 32 | 32 | 96 | 36.9% | 0.9994 | 10.86 | -5.19 | 100% |

### 1.2 Duty-Cycle / Sparsity 系列

| Label | Edge/Top (DAC) | Period (DAC) | Pulse (DAC) | Duty | Timing Corr | Tone RMSE | Final Skew (ps) | Dither A/B Valid |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| p260_e16t32 | 16/32 | 260 | 64 | 24.6% | 0.9994 | 10.65 | -5.21 | 100% |
| p520_e16t32 | 16/32 | 520 | 64 | 12.3% | 0.9995 | 10.33 | -0.87 | 58.5% |
| p1040_e16t32 | 16/32 | 1040 | 64 | 6.2% | 0.9995 | 10.11 | -7.11 | 0% |
| p520_e8t8 | 8/8 | 520 | 24 | 4.6% | 0.9995 | 10.07 | -7.11 | 83.3% |
| p1040_e8t8 | 8/8 | 1040 | 24 | 2.3% | 0.9995 | 10.02 | -5.21 | 0% |

---

## 2. 板级结果（真实台架）

数据目录：`test_platform/thesis_v3_500mhz_appl/adc_data/calibration_exports/pulse_width sweep/`

### 2.1 Pulse Width 系列（板上实际跑了 e4t4 / e8t8 / e16t32）

| Label | Pulse (DAC) | Duty | Timing Corr | Tone RMSE | Final Skew (ps) | Dither A/B Valid | Dither Skew Valid | Perf Valid |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| e4t4 | 12 | 4.6% | 0.99990 | 5.73 | -0.19 | 99.4% | 0% | 1 |
| e8t8 | 24 | 9.2% | 0.99993 | 4.99 | -0.24 | 98.0% | 0.4% | 1 |
| e16t32 | 64 | 24.6% | 0.99986 | 6.91 | -0.38 | 99.4% | 0% | 1 |

### 2.2 Duty / Sparsity 系列（板上实际跑了 p520_e8t8 / p520_e16t32）

| Label | Period (DAC) | Duty | Timing Corr | Tone RMSE | Final Skew (ps) | Dither A/B Valid | Dither Skew Valid | Perf Valid |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| p520_e8t8 | 520 | 4.6% | 0.99993 | 4.81 | -4.82 | 75.3% | 0% | 1 |
| p520_e16t32 | 520 | 12.3% | 0.99991 | 5.54 | **N/A** | 88.2% | 0.6% | 未完成 |

> 注：`p520_e16t32` 缺少 `calibration_performance.csv`，因此 final skew 为空。

---

## 3. 模拟器 vs 板级对比

| 配置 | Dither A/B Valid (Sim/Board) | Tone RMSE (Sim/Board) |
|---|---:|---:|
| e4t4 | 98.5% / 99.4% | 10.09 / 5.73 |
| e8t8 | 98.9% / 98.0% | 10.18 / 4.99 |
| e16t32 | 100% / 99.4% | 10.65 / 6.91 |
| p520_e8t8 | 83.3% / 75.3% | 10.07 / 4.81 |
| p520_e16t32 | 58.5% / 88.2% | 10.33 / 5.54 |

---

## 4. 结论

### 一致的趋势

- **窄脉冲（e4t4）在模拟器和板级都可检测**；
- **周期 520 时检测率下降**，板级也观察到（75–88%），和模拟器方向一致；
- **dither fine-skew 基本 INVALID**，符合历史结论。

### 不一致的地方

- 板级 tone RMSE 整体比模拟器低（模拟器噪声模型偏保守）；
- `p520_e16t32` 在模拟器检测只有 58.5%，板级 88.2%，说明模拟器对稀疏场景的检测率也偏保守。

### 板级结论

- **Pulse 可以很窄**：12 DAC（6 ADC samples）检测 99.4%；
- **Pulse 不能太稀疏**：周期 520 时检测降到 75–88%，进一步稀疏预计失效；
- 这为“low-cost、minimally intrusive impulse”提供了板上证据。

---

## 5. 文件位置

- 板级 CSV：`test_platform/thesis_v3_500mhz_appl/adc_data/calibration_exports/pulse_width sweep/`
- 模拟器 CSV：`D:\TIADC\FPGA_simulator\pulse_sweep_data/`
- 机器可读模拟器结果：`pulse_sweep_results.json`
- 机器可读板级结果：`pulse_sweep_board_results.json`
- 波形生成脚本：`calibration_loop/sweep_pulse.py`
- 模拟器运行脚本：`D:\TIADC\FPGA_simulator\tests\pulse_sweep.py`

## 6. 下一步

- 若需要，补跑 `p520_e16t32` 的 performance；
- 将“narrow + sparse impulse”作为 selling point 的量化证据写入论文材料。
