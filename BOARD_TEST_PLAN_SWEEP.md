# Board Test Plan — Amplitude + Pulse/Duty Sweep (Updated)

基于 FPGA_simulator 结果，更新后的上板测试计划。目标是在真实台架上验证两个维度：

1. **Amplitude**：dither 能多小，同时保持可检测性？
2. **Pulse width / duty-cycle**：impulse 能多窄、多稀疏，同时保持可检测性？

---

## 1. 模拟器结论摘要

| 维度 | 结论 |
|---|---|
| Amplitude | 500 LSB 最佳；250 LSB 接近下限；750 LSB 以上检测下降 |
| Pulse width | 窄到 12 DAC（6 ADC samples）仍 ~98% 可检测 |
| Duty / sparsity | 周期 520 DAC 开始不稳；1040 DAC 完全失效 |
| Frequency-aware period | 精确 300 MHz 已由工具生成配置，模拟器验证通过 |

---

## 2. 公共测试流程

每个配置：

```text
1. DPG 加载对应 TXT
2. UART: adc -ref
3. UART: adc -cal
4. UART: adc -cal export
5. 保存 calibration_run_<date>_<label> 目录 + UART log
```

DPG 设置：

- 一行一个 signed 16-bit 整数，无 header
- `unsigned data`：关闭
- 示波器确认正弦 + 周期脉冲存在

---

## 3. 建议测试矩阵（按优先级）

### Phase A — Amplitude Sweep（先跑）

| 优先级 | Label | 波形文件 | 模拟器预期 |
|---|---|---|---|
| ★★★ | amp500 | `calibration_out/amplitude_sweep/sine_299p375MHz_2p6GSPS_impulse_dither_a500.txt` | 最佳点，检测 100% |
| ★★★ | amp250 | 同上 `_a250.txt` | 检测 ~98.5%，接近下限 |
| ★★☆ | amp750 | 同上 `_a750.txt` | 检测开始下降 ~84% |
| ☆☆☆ | amp1000 | 同上 `_a1000.txt` | 可选，检测 ~81% |
| ☆☆☆ | amp2000 | 同上 `_a2000.txt` | 可选，检测 ~70% |

### Phase B — Pulse Width Sweep（固定 500 LSB、周期 260 DAC）

| 优先级 | Label | 波形文件 | 模拟器预期 |
|---|---|---|---|
| ★★★ | e4t4 | `calibration_out/pulse_sweep/pulse_e4t4.txt` | 最窄，检测 ~98.5% |
| ★★★ | e8t8 | `calibration_out/pulse_sweep/pulse_e8t8.txt` | 窄脉冲，检测 ~98.9% |
| ★★★ | e16t32 | `calibration_out/pulse_sweep/pulse_e16t32.txt` | baseline，检测 100% |
| ☆☆☆ | e8t4 | `pulse_e8t4.txt` | 可选 |
| ☆☆☆ | e16t8 | `pulse_e16t8.txt` | 可选 |
| ☆☆☆ | e32t32 | `pulse_e32t32.txt` | 可选 |

### Phase C — Duty / Sparsity Sweep（固定 500 LSB）

| 优先级 | Label | 波形文件 | 模拟器预期 |
|---|---|---|---|
| ★★★ | p520_e8t8 | `calibration_out/pulse_sweep/pulse_p520_e8t8.txt` | 稀疏窄脉冲，检测 ~83% |
| ★★☆ | p520_e16t32 | `calibration_out/pulse_sweep/pulse_p520_e16t32.txt` | 稀疏 baseline，检测 ~58% |
| ★★☆ | p1040_e16t32 | `calibration_out/pulse_sweep/pulse_p1040_e16t32.txt` | 预期失效，检测 ~0% |
| ☆☆☆ | p1040_e8t8 | `calibration_out/pulse_sweep/pulse_p1040_e8t8.txt` | 预期失效，检测 ~0% |

### Phase D — Frequency-Aware Period 验证（可选，能力验证）

| 优先级 | Label | 波形文件 | 模拟器预期 |
|---|---|---|---|
| ★★☆ | freq300 | `calibration_out/freq_aware_300/freq_aware_300MHz.txt` | 精确 300 MHz，`adc -cal` 完整跑通，事件检测正常 |

这个 run 用于验证 frequency-aware period 在真实硬件上可用，不是当前论文主线必需项。台架时间充裕时补跑。

---

## 4. 最少必跑组合（台架时间紧时）

如果时间只够跑 7 个：

```text
amp500   （baseline / 最佳点）
amp250   （低幅度极限）
amp750   （幅度下降点）
e4t4     （最窄脉冲）
e8t8     （窄脉冲）
p520_e8t8（稀疏极限）
p520_e16t32（稀疏对照）
```

这 7 个已经能覆盖两个维度的主要 trade-off。

如果时间允许，加第 8 个：

```text
freq300   （frequency-aware 300 MHz 能力验证）
```

---

## 5. 每个配置需要记录的数据

| 指标 | 文件/字段 |
|---|---|
| 事件检测率 | `calibration_skew_captures.csv` 的 `dither_A_valid` / `dither_B_valid` |
| dither fine-skew | `dither_skew_valid` |
| tone 收敛 skew | `calibration_performance.csv` 的 `final_skew_ps` |
| tone 拟合质量 | timing captures 的 correlation / RMSE |
| offset/gain 校正 | iterations CSV 最后一行 |
| SNDR/ENOB | performance CSV |
| 示波器脉宽 | 10–90% 宽度（可选） |

---

## 6. 判定原则

- 不强制每个配置都 PASS；
- 检测率下降或失效也是有效结果，记录即可；
- 不要放宽门限；
- 模拟器只是预检，最终以上板为准。

---

## 7. 完成后需要带回

- 所有 `calibration_run_*` 目录
- 每个配置的 UART log
- 示波器截图（至少 amp500、amp250、e4t4、p520_e8t8）
- 实际加载的 TXT 文件名清单
- 与模拟器趋势不一致的地方

---

## 8. 相关文件

- Amplitude 波形：`calibration_out/amplitude_sweep/`
- Pulse 波形：`calibration_out/pulse_sweep/`
- Amplitude 详细操作：`calibration_out/amplitude_sweep/BENCH_OPERATION.md`
- Pulse 详细操作：`calibration_out/pulse_sweep/BENCH_OPERATION.md`
- Amplitude 模拟器结果：`AMPLITUDE_SWEEP_RESULTS.md`
- Pulse 模拟器结果：`PULSE_SWEEP_RESULTS.md`
- Frequency-aware period 工具：`calibration_loop/frequency_aware_period.py`
- Frequency-aware 说明：`FREQUENCY_AWARE_PERIOD.md`
- 300 MHz 波形：`calibration_out/freq_aware_300/`
