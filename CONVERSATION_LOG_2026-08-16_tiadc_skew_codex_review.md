# 对话记录 / AI 分析日志：TIADC Stage-4 Timing Skew 问题与 Codex 修复评估

- 日期：2026-08-16
- 仓库：`D:/TIADC/summer2025_research_tiadc`
- 分支：`main`（origin/main）
- 关联提交：`532d6b4 add some fix on the timing skew, still need to test it`
- 记录目的：让另一台电脑上的 AI 可以像读取日志一样快速恢复本次分析上下文
- 重要提醒：`test_platform/thesis_v3_500mhz_appl/adc_data/` 中的数据已过时，不要作为当前硬件状态参考

---

## 1. 对话流程摘要

1. 用户要求检查仓库文件并总结用途；AI 只读检查后总结了 TIADC 校准研究仓库的整体结构。
2. 用户要求重点查看 `test_platform/thesis_v3_500mhz_appl`；AI 只读梳理了固件工程、命令系统、五级校准流水线和数据通路。
3. 用户说明板载 calibration loop 的 Stage 1–3 已完成，**Stage 4 timing skew 有问题**，并提供修复前固件的 UART 现场输出。
4. 用户给出 Codex 的修复说明：将 skew actuator characterization 从固定 ±1-code probe 改为依次尝试 `1 → 2 → 4` code，每个 probe 后恢复 baseline，并扩展报告字段。
5. AI 审查了提交 `532d6b4` 的 diff、板级 actuator 实现、表征流程和仿真测试，给出“大概率有效、但未在硬件上验证”的评估。
6. 用户提醒 `adc_data` 已过时，不要参考；本次分析不依赖该目录内容。
7. 用户要求保存本对话记录并提交到 main 分支。

---

## 2. 项目背景（供后续 AI 快速恢复）

- 这是多伦多大学 2025 暑期 **Time-Interleaved ADC 校准研究仓库**。
- 目标：用参考 dither + 负反馈回路校正 timing/gain/offset/skew 失配。
- 硬件：ZCU102 + AD9695（JESD204B）+ AD9164/DPG。
- 关键目录：
  - `test_platform/thesis_v3_500mhz_appl/`：Vitis 裸机固件（主程序、UART 命令台、lwIP UDP、AXI DMA、AD9695 SPI、校准流水线）。
  - `calibration_sim/`：主机端 C 仿真与测试平台（复用生产估计器模块）。
  - `calibration_loop/`：Python 板级闭环校准包。
  - `fpga/skew_actuator/`：尚未集成到主流程的 RTL skew actuator。
- 固件五级流水线：`Timing → Offset → Gain → Skew → Performance`。

---

## 3. 问题现象（Stage 4 Skew）

用户提供的 UART 是 **修复前旧固件** 的输出，关键内容如下。

### 3.1 Skew 基线测量

```text
Skew batch aggregation
Captured frames : 10/10
Valid primary estimates: 10
Rejected primary estimates: 0
Mean skew : -167.440125 ps
Median skew : -168.561848 ps
Standard deviation : 12.255914 ps
Preferred stability limit: 19.230769 ps
Marginal/high-noise boundary: 26.923077 ps
Polarity branches : SAME 0 | INVERTED 10 | changes 0
Final stability : STABLE
Invalid condition : NONE
```

### 3.2 Closed-loop 初始基线

```text
Closed-loop initial baseline: -0.236501 samples
Closed-loop initial baseline: -181.924059 ps
Initial baseline skew std: 0.020974 samples
Initial baseline skew std: 16.133638 ps
Skew error/noise ratio: 11.276072
Initial baseline classification: STABLE
Characterization allowed: YES
Characterization policy : NORMAL
Characterization gate : PASS - STABLE BASELINE
```

### 3.3 Characterization 结果（旧逻辑）

```text
First probe classification: STABLE
Repeat probe classification: STABLE
Response significance : 1.5 x combined batch standard error
Characterization uncertainty: 0.008329 samples
Characterization uncertainty: 6.407041 ps
Minimum clear response: 0.012494 samples
Minimum clear response: 9.610561 ps
Final relative skew : -0.236501 samples
Final relative skew : -181.924059 ps
Initial delay register : 24
Final delay register : 24
Total register change : 0
Actuator resolution : 0.000000 samples/step
Measured actuator step: invalid
Actuator polarity : UNKNOWN
Iterations completed : 0
Converged : NO
Saturated : NO
Skew measurement : PASS
Skew within tolerance : NO
Correction actuator : AVAILABLE
Skew correction applied : NO
Actuator characterization: FAILED
Closed-loop correction : NOT ATTEMPTED
Correction reason : actuator response not distinguishable from measurement noise
Correction status : NOT APPLICABLE
Stage status : FAIL - ACTUATOR CHARACTERIZATION
Reason : actuator response not distinguishable from measurement noise
```

### 3.4 最终流水线结果

```text
ADC CALIBRATION FAILED
Failed stage : SKEW
Timing : PASS
Offset : CONVERGED
Gain : CONVERGED
Skew measurement : PASS
Performance : NOT RUN
Overall calibration : FAILED
Output usable : NO
```

---

## 4. Codex 的修复内容（提交 `532d6b4`）

### 4.1 核心行为变化

- 旧流程：固定 `±1` code probe → 恢复 → 重复 `±1` probe；任一 probe 不显著或不可重复就失败。
- 新流程：依次尝试 amplitudes `1 → 2 → 4`；每个 amplitude 内做两次 probe，每次 probe 后恢复 characterization-entry 的 `baseline_code`。
- baseline 以实际读到的寄存器为准，不硬编码 code 24。
- 优先正向；若 `baseline + amplitude` 超出范围，则反向 probe。
- 每次已验证的 transition：`write → hardware readback/JESD recovery → logical readback → 1 次丢弃 DMA capture → 测量批次`。
- 恢复 baseline 使用无条件 baseline 写 + readback 校验 + JESD recovery + 丢弃采集；恢复失败立即停止并报告 final register unknown。
- 权威初始 median、std、classification 不被 probe 批次覆盖。

### 4.2 显著性/重复性规则（未改动）

```text
uncertainty = sqrt(std_baseline^2 / N_baseline + std_probe^2 / N_probe)
abs(response) >= 1.5 * uncertainty
```

```text
both responses finite and nonzero
both independently significant
same physical sign
abs(resolution_2 - resolution_1) / abs(resolution_1) <= 0.35
all probe and restore readbacks successful
configured polarity, when present, must match
```

归一化：

```text
resolution_1 = response_1 / signed_p
resolution_2 = response_2 / signed_p
final resolution = (resolution_1 + resolution_2) / 2
```

### 4.3 报告变化

新增/拆分 UART 字段：
- actuator readiness、read-only readiness access、correction write status；
- attempted amplitudes 与 successful amplitude；
- 各 probe 的 requested/readback codes、responses、std、uncertainty、required response、significance、normalized resolution；
- repeatability、restoration、escalation、controller-entry status；
- dither 的结构/事件检测与 edge fine-skew 交叉校验分开报告。

### 4.4 测试声明

- 仿真构建通过（MinGW GCC）。
- `5591` unit tests 通过。
- `6` controller tests 通过。
- `14` pipeline scenarios 通过。
- `git diff --check` 通过。
- Sanitizer build 因该环境缺 libasan/libubsan 未链接。
- Vitis 固件构建未执行（Codex 声称其环境缺少 `C:\AMD\2025.1` 工具链）。

---

## 5. AI 对修复是否解决实际问题的评估

### 5.1 结论

**方向正确、大概率有效，但尚未被板级运行证明；当前只能说“对症”，不能说“已解决”。**

### 5.2 为什么认为修复对症

根据 UART 数值和代码中的执行器设计参数：

| 项目 | 数值 |
|---|---|
| 初始 baseline std | 16.13 ps |
| probe std（反推） | 约 12.26 ps |
| 合成不确定度 | 6.407 ps |
| 旧逻辑 1.5x 检测门限 | 9.611 ps |
| 设计的一码执行器步进（4 raw x 2 通道 x 1.725 ps） | 约 13.8 ps |

- 1-code 的理论响应约 13.8 ps，只有门限的约 1.44 倍；在 10 帧和上述噪声下，两个 probe 都显著的概率不高，旧逻辑失败是统计上可预期的。
- 2-code 理论响应约 27.6 ps，是门限的约 2.9 倍。
- 4-code 理论响应约 55.2 ps，是门限的约 5.7 倍。
- 若每码真实步进 > 4.8 ps，2-code 大概率通过；> 2.4 ps，4-code 大概率通过。设计值 13.8 ps 有较大余量。

因此，若 actuator 有真实物理效果，新流程应能把 Stage 4 从 `ACTUATOR CHARACTERIZATION FAILED` 推进到控制器阶段。

### 5.3 为什么不能宣布已解决

1. 新固件从未在台架上运行；提供的 UART 是旧固件输出，缺少每个 probe 的实际 response 值。
2. 如果每码真实步进接近 0（例如该工作点细延时寄存器没有物理效果），`1 → 2 → 4` 放大仍会全部失败。
3. 所有 probe 仍与同一个初始 baseline 相减；幅度放大可压制批次内随机噪声，但不能消除 baseline 与后续批次之间的慢漂移/JESD reset 系统性偏移。
4. 归一化假设执行器线性；若存在死区或非线性，表征可能通过但“每码步进”不准，控制器随后可能振荡或不收敛。
5. Dither cross-check 仍然 invalid（rising -15.96 ps、falling -134.84 ps、edge disagreement 118.87 ps）；Codex 只改了报告，没有修。目前 dither 不 gate 该 stage，但这仍是需要后续调查的信号。
6. 主机仿真测试中的 actuator 响应是确定性的，不模拟真实统计噪声；测试通过只证明控制逻辑正确。

### 5.4 板卡复测判据

新固件应逐 amplitude 打印 `Response #1/#2`、`Required response`、`Significance`、`Resolution`、`Repeatability`：

- **1-code fail，但 2-code 或 4-code 两次都显著、repeatable、restore PASS** → 本次问题已解决，Stage 4 进入 controller，继续观察收敛。
- **1/2/4 全部 fail，且响应接近 0** → 不是幅度问题，执行器没有可测物理效果；需要 `adc -cal diagnose skewprep actuator|combined|jesd|ctrl|analog|digital` 隔离 SPI 写、JESD reset、物理延时各自贡献。
- **4-code 表征通过但 controller 不收敛/饱和** → 下一层问题（线性度、步进误差、控制器增益），不在本次修复范围内。

### 5.5 AI 只读验证记录

- 检查了 `532d6b4` 的完整 diff。
- 在 `/tmp` 用 MinGW 重建 `calibration_sim`：`5591` unit tests、`6` controller tests、`14` pipeline scenarios 全部通过。
- 用 `H:\2025.1` 的 `aarch64-none-elf-gcc` 对 `butils.c`（含 `butils_calibration.c`）做 `-fsyntax-only`：通过，仅有原有 pedantic 警告；`adc_calibration_skew.c` 也通过。
- 分析期间未修改任何源文件。

---

## 6. 后续建议（给下一台电脑的 AI）

1. 先读 `AGENTS.md` 和本日志。
2. 不要参考 `adc_data/` 里的旧数据。
3. 在台架重建 Vitis 固件，重跑 `adc -cal`，抓取新的 per-attempt characterization 输出。
4. 根据新输出按下述分支处理：
   - 表征通过但控制器失败 → 分析新的 controller 日志；
   - 所有 amplitude 响应约 0 → 转硬件诊断，不要继续调固件阈值；
   - 表征通过且控制器收敛 → 对比最终 performance，关闭 Stage 4 问题。
5. 若调查 actuator 物理效果，优先使用 `adc -cal diagnose skewprep` 系列命令。
6. 若修改共享估计器/流水线模块，必须保持 `calibration_sim --run-all` 通过。
