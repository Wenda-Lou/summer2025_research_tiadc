# Amplitude Sweep — 固件上板操作清单

目标：在真实 ZCU102 + AD9695 + AD9164 台架上，用固件 `adc -cal` 流程验证不同 dither 幅度下的系统行为。

参考波形已生成在本目录：

```
sine_299p375MHz_2p6GSPS_impulse_dither_a250.txt
sine_299p375MHz_2p6GSPS_impulse_dither_a500.txt
sine_299p375MHz_2p6GSPS_impulse_dither_a750.txt
sine_299p375MHz_2p6GSPS_impulse_dither_a1000.txt
sine_299p375MHz_2p6GSPS_impulse_dither_a2000.txt
```

波形参数：

- 主 tone：299.375 MHz（相干，29.9375 cycles/event）
- DAC：2600 MS/s，ADC：1300 MS/s
- dither 周期：260 DAC samples = 130 ADC samples
- pulse：16/32/16 DAC samples（raised cosine）
- seed：20260725

---

## 0. 前置检查

- [ ] 台架已上电，AD9164 DPG 正常循环
- [ ] UART COM 口号已确认（下面以 `COM3` 为例）
- [ ] 主机 IP 192.168.1.100，板子 192.168.1.10
- [ ] 已备份当前固件导出目录
- [ ] 每个幅度跑完后保存 `calibration_run_*` 目录和 UART log

---

## 1. 每个幅度的标准流程

对每个幅度重复以下步骤。

### 1.1 DPG 加载波形

把对应 TXT 加载到 DPG Downloader，例如：

```text
calibration_out/amplitude_sweep/sine_299p375MHz_2p6GSPS_impulse_dither_a250.txt
```

DPG 设置：

- 格式：一行一个 signed 16-bit 整数，无 header
- `unsigned data`：**关闭**
- 确认示波器上能看到 299.375 MHz 正弦 + 周期脉冲

### 1.2 UART 上传参考

```text
adc -ref
```

确认输出包含：

```text
Reference uploaded successfully
```

### 1.3 跑完整校准

```text
adc -cal
```

等待输出出现：

```text
ADC CALIBRATION COMPLETE
```

如果某个幅度出现：

- `Dither event detection` 不是 10/10
- 或 `dither_skew_valid` 全 0
- 或 offset/gain 阶段不收敛

都属于**该幅度的有效观测结果**，不要强行重跑或改门限，记录即可。

### 1.4 导出 CSV

```text
adc -cal export
```

把收到的 CSV 保存到独立目录，建议命名：

```text
calibration_run_<date>_amp250
calibration_run_<date>_amp500
calibration_run_<date>_amp750
calibration_run_<date>_amp1000
calibration_run_<date>_amp2000
```

同时保存完整 UART log。

---

## 2. 幅度顺序建议

建议按以下顺序跑，便于观察趋势：

```text
500 LSB   →  模拟器最佳点 / 对照
250 LSB   →  低注入极限
750 LSB   →  第一次下降点
1000 LSB  →  中间点
2000 LSB  →  高注入对照
```

每个幅度之间：

- 如果 DPG 波形切换后没有脉冲，先检查 DPG 是否在 loop；
- 每次 `adc -cal` 前都重新执行 `adc -ref`，确保参考和 DPG 波形一致。

---

## 3. 每个幅度需要记录的数据

从导出 CSV 中提取：

| 指标 | 文件/字段 |
|---|---|
| 事件检测率 | `calibration_skew_captures.csv` 的 `dither_A_valid` / `dither_B_valid` |
| dither fine-skew 有效性 | `dither_skew_valid` |
| tone 收敛 skew | `calibration_performance.csv` 的 `final_skew_ps` |
| tone 主路径收敛 | `calibration_skew_iterations.csv` / UART `skew` 收敛行 |
| offset 校正 | `calibration_offset_iterations.csv` 最后一行 `correction_after` |
| gain 校正 | `calibration_gain_iterations.csv` 最后一行 `gain_correction_after` |
| SNDR/ENOB | `calibration_performance.csv` 的 `cal_*_sndr_db` / `cal_*_enob` |
| 脉冲展宽 | 可选：示波器测 10–90% 宽度 |

---

## 4. 判定标准（记录用，不强制通过）

| 项 | 期望 |
|---|---|
| tone 主路径 | skew 收敛 <1 ps，Stage 5 正常 |
| dither 事件检测 | 10/10 最好；低于此记录为低注入极限 |
| dither fine-skew | 当前链路预期 INVALID；记录 valid rate 即可 |
| offset | 记录 weak PASS / 偏差 codes |
| gain | 记录数值，不要求恒定因子 |

---

## 5. 完成后需要带回的东西

- 5 个幅度的 `calibration_run_*` CSV 目录
- 每个幅度的 UART log
- 示波器截图（至少 500 LSB 和 250 LSB）
- 每个幅度实际加载的 TXT 文件名
- 任何异常现象（DPG 不循环、UDP 超时、寄存器异常等）

---

## 6. 当前模拟器预检状态

已在 `FPGA_simulator` 用真实固件跑完新范围（250/500/750/1000/2000 LSB）：

```text
250 LSB : timing corr 0.9995, RMSE 10.15, dither A/B valid 98.5%
500 LSB : timing corr 0.9994, RMSE 10.65, dither A/B valid 100%
750 LSB : timing corr 0.9994, RMSE 11.40, dither A/B valid 83.8%
1000 LSB: timing corr 0.9992, RMSE 12.33, dither A/B valid 81.3%
2000 LSB: timing corr 0.9984, RMSE 16.92, dither A/B valid 70%
```

说明：

- 500 LSB 是模拟器最佳点；
- 750 LSB 以上事件检测开始下降；
- dither fine-skew 保持 INVALID，符合 dispersion 预期。

上板后优先对比 **250 / 500 / 750 LSB** 是否与模拟器趋势一致。
