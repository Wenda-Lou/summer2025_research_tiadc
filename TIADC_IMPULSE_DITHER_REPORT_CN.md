# TI ADC 数字注入脉冲 Dither 校准系统——完整报告（中文版）

**日期**：2026-08-19
**版本**：v1.0（最终）
**关联文档**：`PLAN_DITHER_FIX.md`（执行史）、`DITHER_CONCLUSION.md`（结论）、`AGENTS.md`（仓库导览）

---

## 1. 项目概述

本项目为多伦多大学 2025 暑期研究项目：**时间交织（TI）ADC 校准**。目标是用
**数字生成的 impulse dither**（由 AD9164 DAC + DPG 波形发生器注入）替代已有
工作中的 ramp dither / analog summing 方案，并利用 impulse 内不同位置的
**斜率特性**，把 **gain、offset、timing skew** 三种失配分离出来。

**目标台架**：
- **ZCU102**（Zynq UltraScale+ MPSoC，ARM Cortex-A53 + PL）
- **AD9695** 双通道 ADC（JESD204B 接口，1.3 GSPS/通道）
- **AD9164** DAC（2.6 GSPS），由 DPG 生成参考正弦 + impulse dither 波形

**核心指标**：校准后通道匹配、SNDR/ENOB、dither 三分离的独立验证。

---

## 2. 系统架构与校准 Pipeline

### 2.1 五阶段自动校准流程（`adc -cal`）

| 阶段 | 内容 | 主估计器 | dither 的角色 |
|---|---|---|---|
| Stage 1 Timing | 整数/分数延迟对齐、dither 事件检测 | 互相关 + tone 拟合 | 事件检测/结构验证 |
| Stage 2 Offset | 通道 DC 失配校正 | tone DC 拟合 | **dither offset 分离**（平顶采样）|
| Stage 3 Gain | 通道增益失配校正 | tone 幅度拟合 | **dither gain 分离**（full/flat 双参考）|
| Stage 4 Skew | B 相对 A 的时序失配闭环校正 | **tone 相位差**（主）| **dither fine-skew 交叉校验**（advisory）|
| Stage 5 Performance | SNDR/SFDR/THD/ENOB、A/B 匹配 | 频谱分析 | — |

### 2.2 三层代码结构

1. **固件**（`test_platform/thesis_v3_500mhz_appl/`）：bare-metal C，
   UART 命令控制台、lwIP UDP 数据卸载、AXI DMA 抓帧、SPI 控制 AD9695；
   共享估计器模块（`adc_calibration_skew/dither/performance.c` 等）同时
   编译进固件与桌面模拟器。
2. **桌面 C 模拟器**（`calibration_sim/`）：编译生产估计器源码，跑
   单元/场景/管线/压力测试——**主要测试套件**（当前 5728+ unit /
   18 scenario / 14 pipeline 全绿）。
3. **Python 校准环与工具**（`calibration_loop/`）：`dither_replay.py`
   （离线原始帧回放）、`capture_frames.py`（批量抓帧）、波形生成器、
   `run_calibration.py check`（波形一致性检查）。

### 2.3 信号链

```
DPG 波形 (tone + impulse dither) → AD9164 DAC → 模拟注入 → AD9695 ADC (A/B)
→ DMA 抓帧 (4096 B) → 固件估计（tone 拟合 + 残差 + dither 分析）→ 校正
→ Stage 5 频谱评估 → UDP/CSV 导出
```

---

## 3. 原理

### 3.1 Tone 主估计（主校准路径）

- 每帧 800 样本固定窗口，对 A/B 分别做**最小二乘 tone 拟合**
  （DC + cos + sin，公共频率精化），得到幅度、相位、RMSE、相关；
- **相对 skew** = B/A 相位差（含全局反相分支解析，INVERTED → ±π 调整）；
- Stage 4 闭环：表征 actuator（AD9695 细延时寄存器）每码响应 →
  控制器按批次中位数步进 → 2/2 连续 PASS 收敛。

### 3.2 Impulse dither 三分离原理

| 分离项 | 原理 | 依赖的波形特性 |
|---|---|---|
| **Offset** | 极性加权事件平均 → **平顶区采样**（模板导数≈0 处）| 真平顶（top ≥ 16 ADC 样本）|
| **Gain** | 残差对模板的**最小二乘投影增益**（full）与平顶区增益（flat）| 模板/幅度域一致性 |
| **Skew** | 事件窗口极性加权聚合 → A 实测 profile 作本地模板 → B 拟合 →
上升/下降/全 profile 三种导数投影 → rising−falling 为边沿分歧 | 长线性斜坡、窗口完整性 |

- 事件检测：残差与模板互相关，按事件周期（130 ADC 样本）槽内取最强峰；
- 交叉校验（advisory）：`|dither − tone| ≤ 0.03 样本（23.1 ps @1.3 GSPS）`
  且估计器自身 PASS；**永不 gate 任何阶段**。

### 3.3 交叉帧联合聚合（joint，实验性）

- 把一批（10 帧）残差的所有事件窗口**跨帧堆叠**再估计一次，
  利用跨帧极性序列不同将窗口偏置随机化；
- 实现为共享函数 `adc_cal_skew_estimate_joint_frames()`，板级在
  Stage 4 每 iteration 调用，结果写入 CSV 与 UART。

### 3.4 Stage 5 指标

- 频谱：Blackman-Harris7 窗 + Goertzel 逐 bin → 基波/谐波/杂散功率 →
  SNDR/SFDR/THD/ENOB；raw 与 calibrated 各一套；
- 匹配：A/B 相关、RMSE、offset/gain/skew 失配（极性归一化后）；
- 平行平均输出（(A+B)/2）的 SNDR/ENOB。

---

## 4. 结果

### 4.1 各阶段板级结果（最新 run 19:03，linear 48/32 波形，窗口 ±64）

| 阶段 | 结果 |
|---|---|
| Stage 1 Timing | PASS（correlation 0.9985，dither 结构验证 PASS）|
| Stage 2 Offset | CONVERGED（+5.13 codes，verify residual 1.79，corr 0.9983）；dither estimator PASS（平顶 31 样本，events 6）|
| Stage 3 Gain | CONVERGED（gain 1.0000，verification error 0.00054）；dither gain WARNING（dither 0.235 / flat 0.249，FIT_QUALITY）|
| Stage 4 Skew | tone 收敛 **−0.87 ps**、2/2 PASS、reg 35、actuator 12.1 ps/code；dither fine-skew PARTIAL（**1/10 帧 VALID**）、joint INVALID、edge 分歧 666 ps |
| Stage 5 | SNDR 38.8 dB / ENOB 6.13 bits（A/B）；平行平均 **39.5 dB / 6.27 bits**；cal A/B correlation 0.99995、RMSE 18.2 codes；**Mean RMSE 23.15 / Mean correlation +0.998354**（修复后正常）|
| 总体 | **PASS**（skew MARGINAL 警告）|

### 4.2 Tone 主路径历史（多 run 对比）

| run | 最终 skew | batch std | 收敛 | reg |
|---|---|---|---|---|
| 13:18 | −4.99 ps | 1.81 ps | CONVERGED | 33 |
| 17:55 | **+0.166 ps** | 0.34 ps | CONVERGED | 35 |
| 19:03 | −0.87 ps | 0.62 ps | CONVERGED | 35 |

### 4.3 Dither 三分离最终状态

| 分离项 | 状态 | 关键数据 |
|---|---|---|
| 事件检测 | **VALID 10/10** | 全 run 稳定 |
| **Offset** | **估计器 PASS / 控制器收敛（精度受限）** | 平顶 31 样本；Dither offset 与 fitted-tone DC 偏差 3–4 codes、跨 run 不稳定 |
| **Gain** | **受限（定案）** | dither_gain 0.24–0.44；**flat ≈ dither（0.249 vs 0.235）**；无校准常数 |
| **Skew** | **定案 INVALID（advisory）** | per-frame edge 分歧 median 148–305 ps（最新 666 ps）；joint 全 0；1/10 帧 VALID |

### 4.4 工具链产出

- **per-frame 诊断列**（`calibration_skew_captures.csv`）：dither_reason /
  rising / falling / edge / SNDR / ENOB（全 pipeline 可追踪）；
- **SNDR/ENOB 追踪**：timing ~37 dB、offset/gain ~37 dB、skew fit ~32 dB、
  Stage 5 39 dB（各 stage 每 capture/iteration 可绘图）；
- **sim**：5728+ unit / 18 scenario / 14 pipeline 全绿；
- **离线回放**：`dither_replay.py` 支持模板/窗口/门控/去趋势/诊断参数。

---

## 5. 问题

### 5.1 核心问题：dither fine-skew 交叉校验不可用

**症状**：per-frame edge 分歧 100–1000 ps（门限 23.1 ps）；dither 估计值
帧间漂移 ±100–200 ps 而 tone 稳定 ±1 ps；joint 聚合在离线 6 帧偶然 PASS
（4.7/12.4 ps）但板级 10 帧失败（113–666 ps）。

**累计否决 14 种方法**（全部有数据）：
波形几何（RC 16/32、linear 48/0、linear 48/32、p260 双倍周期）、窗口/门控
参数扫描（25 组合）、per-edge gain、自校准定位、detrend、去 DC、两参数
延迟+宽度模型、间隙背景估计、中位数聚合、事件质量门控、事件中心精化、
gain 比例校准、flat-gain 校准。

**机制证据链（6 条）**：
1. **链路色散**：注入 32–48 样本脉冲 → ADC 域 10–90% 宽 74–123 样本；
2. **窗口/周期约束**：130 样本周期下窗口 ±64 占满 129 样本——无间隙测
   背景、事件中心跳变（35 样本）无容错；双倍周期（260）虽解除约束但更宽
   脉冲（112 样本）污染 tone 拟合 → 主路径退化（baseline std 2–9 → 23 ps）
   ——**dither 几何空间与 tone 主估计存在根本权衡**；
3. **窗口 DC/慢变偏置**：A/B 窗口均值恒 +1.8/−1.6 codes（与极性相关），
   上升/下降掩码不对称 → 边沿相反偏置（100–240 ps）；
4. **事件中心跳变**：帧内相位 44→9（spacing 95/131 伪值）；
5. **模板/定位敏感性**：同帧换模板分歧 0.72 ↔ 243 ps（17 倍）；
6. **tone 拟合污染**：脉冲占窗口比例直接放大批次噪声。

### 5.2 Offset / Gain 分离精度

- **offset**：PASS 为容差内弱通过，Dither offset 与真值（tone DC）偏差
  3–4 codes 且跨 run 方向/量级不稳定；
- **gain**：dither_gain 恒低（0.24–0.44 vs 真值 1.0），flat ≈ full →
  整体模板域↔ADC 码域幅度映射问题，且跨 run 无恒定比例 → 不可校准。

### 5.3 Stage 5 汇总异常（已修复）

joint 固件版本起 `Mean RMSE 759 / Mean correlation −0.998`（基线 19 / +0.999）。
根因：canonical 通道极性标定与实际信号相反（polarity[A]=−1 而 cal_a 与
reference 同相）→ 归一化后反相。**修复**：`analyze_frame` 极性自校正
（canonical 相关为负 → 全局翻转双通道）；板级验证恢复
`Mean RMSE 23.15 / Mean correlation +0.998354`。

### 5.4 其他问题

- **initial baseline MARGINAL**（std 19–23 ps vs 旧几何 2–9 ps）：48/32
  波形起出现，导致表征需大步进（1–2 code 失败 → 4–8 code 通过）；
- **UART 输出淹没**（每帧 20+ 行诊断）→ 已改 summary-only；
- **批量抓帧**：单帧 UART 循环可用但慢；`dma -burst` 固件命令已实现待烧录；
  端口/固件不匹配会导致重复帧（MD5 相同）；
- **sim fixture**：两个 `--run-all` 硬依赖 fixture 曾被误删（已恢复，勿再删）。

---

## 6. 建议

### 6.1 论文叙事建议

- **主张**："用 digitally generated impulse dither 分离三种 mismatch" →
  "**三分离架构完整实现 + offset 分离估计器集成运行 + gain/skew 限制分析**"；
- **正面结果**：数字生成 impulse dither 链路（波形/注入/事件检测 10/10）✓、
  tone 主校准系统（收敛 <1 ps、Stage 5 6.27 bits 平行平均）✓、
  offset 分离估计器 PASS ✓、Stage 5 指标恢复 ✓；
- **限制节**：6 条机制证据链 + 14 种方法否决的系统性分析（这是可发表的
  工程贡献：完整归因 + 权衡量化）；
- **谨慎表述**：offset 的 PASS 是容差内弱通过（偏差 3–4 codes）；
  gain/skew 的"无恒定校准常数"结论需明确（避免读者误以为可标定）。

### 6.2 技术建议（未来方向）

1. **gain 的显式幅度映射校准**：模板域↔ADC 码域存在稳定物理比例
   （约 0.24–0.44 随波形变化），可在已知注入幅度下做**一次离线标定表**，
   代替在线估计——论文可作"限制下的工程替代"；
2. **差分路线**（skew）：reg 步进前后两批同 reg 帧的 Δdither vs Δtone
   ——绝对偏置相消；需正确批量抓帧（`capture_frames.py` + `dma -burst`）；
3. **事件定位鲁棒化**：质心/邻域加权替代 argmax，或对跳变事件做
   spacing 一致性门控（敏感性 177→105 ps 已有部分证据）；
4. **更小 dither 幅度权衡扫描**：降低对 tone 拟合的污染
   （`DITHER_SCALE_LSB` 参数）；
5. **模拟链路色散补偿**：若可表征模拟链路的脉冲响应，可对模板做
   色散预补偿（与 ADC 域测量匹配）；
6. **Stage 5 汇总字段文档化**：Mean RMSE/correlation 定义为 canonical
   通道归一化 cal vs reference 指标（含极性自校正语义）。

### 6.3 工程建议

- 烧录当前固件源码（窗口 64 基线 + joint + `dma -burst` + Stage 5 自校正
  + UART summary-only）作为**最终验证版本**；
- `--run-all` 的 fixture 文件（`adc_capture_20260801_180451.csv` 等）
  是硬依赖，保护或迁移到独立目录；
- 波形/固件窗口宏必须匹配（130 周期 ↔ 窗口 64；p260 ↔ 窗口 100）；
- 抓帧验证：批量后检查 MD5 唯一性（20 帧应有 20 个不同 hash）。

---

## 附录 A：关键文件

| 文件 | 作用 |
|---|---|
| `PLAN_DITHER_FIX.md` | 修复计划 + 8 节执行史（含全部否决记录）|
| `DITHER_CONCLUSION.md` | 最终结论与论文限制节素材 |
| `adc_calibration_skew.c/h` | 共享 skew 估计器（含 joint、detrend、极化）|
| `adc_calibration_dither.c/h` | 共享 dither 事件/分析（含极化共享函数）|
| `adc_calibration_performance.c/h` | Stage 5 频谱/匹配（含极性自校正）|
| `butils.c` / `butils_calibration.c` | 板级编排、export、诊断列、joint 接线 |
| `calibration_loop/dither_replay.py` | 离线回放/诊断 |
| `calibration_loop/capture_frames.py` | 批量抓帧（burst 模式）|
| `waveform_generation/generate_dac_waveform.py` | 波形生成（含周期/形状参数）|

## 附录 B：关键数字速查

- 门限：edge 分歧 0.03 样本 = 23.1 ps @1.3 GSPS；tone/dither 分歧同；
- dither 周期：130 ADC 样本（=260 DAC）；p260 = 260 ADC 样本；
- 窗口：±64（130 周期）或 ±100（p260）；
- Stage 5：SNDR ~39 dB、ENOB ~6.3 bits（平行平均）；
- sim：5728+ unit / 18 scenario / 14 pipeline。
