# Amplitude Sweep Results — Simulator + Board (2026-08-24)

在 FPGA_simulator 和真实板级台架上跑完整固件 `adc -cal` 流程，验证 299.375 MHz
参考波形在不同 dither 幅度下的系统行为。

- 模拟器：`D:\TIADC\FPGA_simulator`
- 板级数据：`test_platform/thesis_v3_500mhz_appl/adc_data/calibration_exports/amplitude pulse/`

## 配置

- 主 tone：299.375 MHz（相干）
- DAC：2600 MS/s，ADC：1300 MS/s
- dither 周期：260 DAC samples = 130 ADC samples
- pulse：16/32/16 DAC samples（raised cosine）
- seed：20260725
- 模拟器模式：bench-like（dispersion + noise）

---

## 1. 模拟器结果（FPGA_simulator）

| Amplitude (LSB) | Timing Corr | Tone RMSE (codes) | Offset Corr (codes) | Gain Corr | Final Skew (ps) | Dither A/B Valid | Dither Skew Valid |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 0.9995 | 10.15 | +5.23 | 0.9770 | -5.21 | 98.5% | 0% |
| 500 | 0.9994 | 10.65 | +5.23 | 0.9770 | -5.21 | 100% | 0% |
| 750 | 0.9994 | 11.40 | +5.73 | 1.0157 | -5.19 | 83.8% | 0% |
| 1000 | 0.9992 | 12.33 | +5.52 | 0.9637 | +5.57 | 81.3% | 0.7% |
| 2000 | 0.9984 | 16.92 | +5.76 | 0.9605 | -4.93 | 70% | 0% |

---

## 2. 板级结果（真实台架）

Run 映射：

```text
calibration_run_20260824_150450 = 250 LSB
calibration_run_20260824_153550 = 500 LSB
calibration_run_20260824_161914 = 750 LSB（未跑完，无 performance）
calibration_run_20260824_164839 = 1000 LSB
calibration_run_20260824_174932 = 2000 LSB
```

| Amplitude (LSB) | Timing Corr | Tone RMSE (codes) | Offset Corr (codes) | Gain Corr | Final Skew (ps) | Dither A/B Valid | Dither Skew Valid | Perf Valid |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 250 | 0.99993 | 4.83 | -9.72 | 1.0041 | -4.06 | 98.4% | 0.5% | 1 |
| 500 | 0.99989 | 6.17 | -10.82 | 1.0131 | -6.86 | 96.4% | 0% | 1 |
| 750 | 0.99978 | 8.21 | +3.92 | 1.0056 | **N/A** | 97.8% | 1.1% | 未完成 |
| 1000 | 0.99965 | 10.74 | -10.57 | 0.9857 | -7.99 | 96.9% | 0.8% | 1 |
| 2000 | 0.99881 | 18.71 | +2.75 | 1.0248 | -1.53 | 99.4% | 3.3% | 1 |

> 注：750 LSB run 只跑到 gain 阶段，缺少 `calibration_performance.csv`，因此 `final_skew_ps` 和 `Perf Valid` 为空。

---

## 3. 模拟器 vs 板级对比

| Amplitude | Timing Corr (Sim/Board) | Tone RMSE (Sim/Board) | Dither A/B Valid (Sim/Board) |
|---:|---:|---:|---:|
| 250 | 0.9995 / 0.99993 | 10.15 / 4.83 | 98.5% / 98.4% |
| 500 | 0.9994 / 0.99989 | 10.65 / 6.17 | 100% / 96.4% |
| 750 | 0.9994 / 0.99978 | 11.40 / 8.21 | 83.8% / 97.8% |
| 1000 | 0.9992 / 0.99965 | 12.33 / 10.74 | 81.3% / 96.9% |
| 2000 | 0.9984 / 0.99881 | 16.92 / 18.71 | 70% / 99.4% |

---

## 4. 结论

### 一致的趋势

- **Tone 拟合质量随 dither 幅度增大而下降**：
  - 模拟器和板级都观察到 timing correlation 下降、tone RMSE 上升；
  - 2000 LSB 时两者 tone RMSE 都明显变差（模拟 16.92，板级 18.71）。

### 不一致的地方

- **事件检测趋势不同**：
  - 模拟器预测 750 LSB 以上检测率明显下降（83.8% / 81.3% / 70%）；
  - 真实板级在 250–2000 LSB 都保持 **96–99%** 检测，没有明显下降；
  - 说明模拟器的 dispersion 模型**高估了高幅度对事件检测的影响**。

### 板级结论

- 250–2000 LSB 范围内，事件检测都可靠；
- 500 LSB 附近 tone 拟合很好（RMSE 6.17），是较优工作点；
- 2000 LSB 时 tone RMSE 明显变差，但检测仍然可靠；
- dither fine-skew 基本保持 INVALID，与历史结论一致。

---

## 5. 文件位置

- 板级 CSV：`test_platform/thesis_v3_500mhz_appl/adc_data/calibration_exports/amplitude pulse/`
- 模拟器 CSV：`D:\TIADC\FPGA_simulator\amplitude_sweep_data/`
- 机器可读模拟器结果：`amplitude_sweep_results.json`
- 上板操作清单：`calibration_out/amplitude_sweep/BENCH_OPERATION.md`

## 6. 下一步

- 补齐 750 LSB run（如果可能），获得 performance 数据；
- 上板 pulse/duty sweep；
- 根据板级结果更新模拟器模型（尤其是事件检测对幅度的敏感性）。
