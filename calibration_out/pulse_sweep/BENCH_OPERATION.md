# Pulse Width / Duty-Cycle Sweep — 固件上板操作清单

目标：在真实台架上验证 impulse 能多窄 / 多稀疏，同时保持可检测性。

参考波形已生成在本目录：

```
pulse_e4t4.txt
pulse_e8t4.txt
pulse_e8t8.txt
pulse_e16t8.txt
pulse_e16t32.txt
pulse_e32t32.txt
pulse_p260_e16t32.txt
pulse_p520_e16t32.txt
pulse_p1040_e16t32.txt
pulse_p520_e8t8.txt
pulse_p1040_e8t8.txt
```

公共参数：

- 主 tone：299.375 MHz
- 幅度：500 LSB
- DAC：2600 MS/s，ADC：1300 MS/s
- seed：20260725

---

## 建议上板范围（不要一次跑完 11 个）

模拟器结果显示：

- 窄脉冲（e4t4/e8t8）仍可检测；
- 周期 1040 时检测为 0%，基本不用上板；
- 周期 520 是稀疏极限候选。

建议上板优先跑：

```text
e4t4         最窄
e8t8         窄 + 常见
e16t32       baseline
p520_e8t8    稀疏极限
p520_e16t32  稀疏对照
```

时间充裕再补：

```text
e8t4
e16t8
e32t32
p1040_e16t32 （预期失败，作为失效点确认）
p1040_e8t8   （预期失败，作为失效点确认）
```

---

## 每个配置的标准流程

1. DPG 加载对应 TXT，例如：

   ```text
   calibration_out/pulse_sweep/pulse_e4t4.txt
   ```

2. UART：

   ```text
   adc -ref
   adc -cal
   adc -cal export
   ```

3. 保存：

   ```text
   calibration_run_<date>_pulse_<label>
   ```

   同时保存 UART log。

---

## 需要记录的数据

| 指标 | 文件/字段 |
|---|---|
| 事件检测率 | `calibration_skew_captures.csv` 的 `dither_A_valid` / `dither_B_valid` |
| dither fine-skew | `dither_skew_valid` |
| tone 收敛 skew | `calibration_performance.csv` 的 `final_skew_ps` |
| tone 拟合质量 | timing captures 的 correlation / RMSE |
| offset/gain | 对应 iterations CSV 最后一行 |

---

## 判定

- 事件检测率：记录即可，不强制 100%；
- 如果某个配置检测为 0%，这就是“稀疏/窄度极限”，同样是有用结果；
- 不要为了通过而放宽门限。

---

## 当前模拟器预检结论

- 最窄 `e4t4`（12 DAC）：检测 98.5%
- baseline `e16t32`（64 DAC / 260 period）：检测 100%
- `p520_e16t32`（duty 12.3%）：检测 58.5%
- `p1040_*`（duty <6.2%）：检测 0%

上板时优先确认这几个趋势是否一致。
