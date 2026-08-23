# Frequency-Aware Dither Period

工具：`calibration_loop/frequency_aware_period.py`

## 解决的问题

Impulse-dither 估计器要求主 tone 在每个 dither 事件之间前进**非整数个周期**，
否则 tone 会在事件平均时泄漏进 dither 统计。

固定 dither 周期时，某些频率会正好落在“整数 cycles/event”上，例如：

- 299.375 MHz + 260 DAC period：29.9375 cycles/event ✅
- 300.000 MHz + 260 DAC period：30.0000 cycles/event ❌

Frequency-aware period 会在 DPG loop length / dither period 空间里自动搜索
一个满足相干性约束的配置，从而支持任意目标频率（包括精确 300 MHz）。

## 用法

```bash
# 查找精确 300 MHz 的可用配置
python -m calibration_loop.frequency_aware_period --target-mhz 300

# 生成对应的 DPG 波形
python -m calibration_loop.frequency_aware_period \
  --target-mhz 300 --out calibration_out/freq_aware_300
```

## 300 MHz 搜索结果示例

| # | Actual MHz | N (DAC) | Period (DAC) | Events | Duty | Cycles/Event | Coherence |
|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | 300.000 | 16640 | 640 | 26 | 0.100 | 73.8462 | 0.118 |
| 2 | 300.000 | 13312 | 512 | 26 | 0.125 | 59.0769 | 0.130 |
| 3 | 300.000 | 9984 | 384 | 26 | 0.167 | 44.3077 | 0.076 |
| 4 | 300.000 | 16640 | 320 | 52 | 0.200 | 36.9231 | 0.130 |
| 5 | 300.000 | 6656 | 256 | 26 | 0.250 | 29.5385 | 0.016 |

默认选择排序：先最小频率误差，再最小 duty cycle，因此 rank 1 是
**period=640 DAC、duty=10%** 的配置。

## 已验证

- 脚本为 299.375 MHz / 200 MHz / 300 MHz 都能找到有效配置；
- 生成的 300 MHz 波形（`calibration_out/freq_aware_300/freq_aware_300MHz.txt`）
  已在 FPGA_simulator 上跑通：
  - `adc -cal` 完整完成
  - timing accepted 100%
  - performance valid
  - timing correlation ~0.9996
  - final skew ~ -4.8 ps

## 后续

上板前可再用该工具为任意目标频率生成波形，并用
`FPGA_simulator/tests/phase4_correlate.py --mode sim --reference <txt>`
做预检。
