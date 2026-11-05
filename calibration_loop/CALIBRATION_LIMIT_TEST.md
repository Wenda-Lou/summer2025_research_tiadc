# Offset / gain / skew 单项校准极限测试

后续 AI/维护者先读 [交接记录](CALIBRATION_LIMIT_HANDOFF.md)，了解当前
实现、验证和未上板状态；下文是详细原理和操作步骤。

适用程序：`firmware/thesis_v3_loop_appl` 的 A53 闭环，使用
`calibration_loop_c`。需要重新构建、下载该 ELF；旧 ELF 不包含下面的
`adc -cal loop test ...` 命令。这里没有自动下载 ELF 或连接台架。

## Dither-only 验证（2026-10-28）

已有 `BASELINE valid=1` 时，关闭串口终端，自动完成 offset 扫描：

```powershell
python tools/board_loop/board_axis_test.py sweep --axis offset --uart COM5 --reuse-baseline --execute
```

默认 B 通道，偏置 −64、−16、−4、0、4、16、64 codes，每档 5 遍，
每遍 PRE/TRAIN/HOLD = 40/300/100，按每轮七档循环。
每遍自动 restore、注入、测试、导出 CSV/JSON/PNG/SVG，并立即显示 PASS/FAIL。
全部结束后 restore。输出位于新的 `calibration_out/axis_时间戳/`，
另有 console.log、frames.csv、summary.csv、meta.json；不覆盖已有目录。
`--reuse-baseline` 首先只读验证有效基准、无故障、无参考路由、配置匹配，
不重建基准或写配置；前置检查失败即停止。硬件事务故障会停止，不自动恢复写入；
普通 HOLD 不通过则记录 FAIL 并继续下一档。中途异常保留已完成数据。
只测 +16 共 5 遍可追加 `--points=16 --repeats 5`。不带 `--execute` 只打印计划。

**最新接口：板端已改为无 reference 的相对校准。** 不需要 `wave 3`、参考文件、
脉冲模板或波形参数，直接 `adc -cal loop 300` 或 `adc -cal loop test baseline 600 100`。
下面描述的 wave 2/3 和 dither_phase 是历史模板模式；当前板端输出应为
`reference_free=1`、`gain=dither_spectrum`、`skew=dither_cross_spectrum`。
需向 A/B 输入同一宽带 dither；算法匹配两路相对误差，不能识别共同绝对误差。
新路径已做仿真和构建验证，尚未上板。PC 导出命令保持不变，不传 `--wave`。

基准训练结束的 offset 系数使用末段最多 100 个尝试中的合格 dither 帧平均，
在输入域消除逐帧反馈状态的影响；控制码移动会清空平均，至少 20 帧才应用。
UART 的 `LOOP-TEST-FREEZE source=dither_training_mean` 标明平均帧数和冻结系数。
随后 HOLD 仍是全新采集、全部反馈关闭，offset ±0.5 codes 等通过阈值不变。
命令仍为 `adc -cal loop test baseline 600 100`，需要下载新 ELF。

本次先用 **dither-only** 进行相同的单项 PRE/TRAIN/HOLD 测试。新固件选择
`adc -cal loop wave 3`：DPG 加载
`waveforms/dither_only/w06adc_e04_t04_amp32767_dither_only.txt`。
Gain 使用 `dither_mag`（每事件峰峰值幅度比），skew 使用 `dither_phase`
（极性校正折叠后拟合分数采样相位），offset 保留极性解耦的平顶估计。
不使用 tone cancellation，不用 tone 幅度/相位作门限、控制或评分。

必须重新构建并下载本次固件；旧 ELF 没有满量程 wave 3。`test status` 应输出
`LOOP-TEST-ROUTES gain=dither_mag skew=dither_phase tone_free=1`；逐帧日志应有
`gsrc=dither_mag skew_route=dither_phase`。`used_gain_ppm` 是当前路线的
增益残差；旧版的 `tone_ppm=nan` 字段已从当前板端输出中移除。
本次使用 DAC 峰值满量程 wave 3，启动默认也为 3。Wave 2 保留 5190 codes 对照。
板端只允许这两种 dither-only 配置。Wave 0/1 的历史配置仍保留在表中，
但 `loop` 和 `loop test` 会拒绝使用它们校准，且在采集或写入前返回错误。
扫描脚本默认 wave 3，接受 `--wave 3` 或 `--wave 2`。

**先验证普通 dither-only 闭环：** DPG 加载上述 wave 3 TXT，正常 bring-up 后
只运行下面一条命令（本固件启动默认 wave 3）。`loop 300` 使用 dither
更新全部三项；`test baseline`
是在此之外增加独立冻结测量和基准准入检查，不是普通校准的前置条件。

```text
adc -cal loop 300
```

闭环命令会先执行 ACK 检查的 zero-step 事务，必要时初始化中性状态。
`wave` 仅用于切换已知输入配置，`status` 仅查询结果，都不是必需校准步骤。
`LOOP-START` 显示 `gain=dither_mag skew=dither_phase tone_free=1`；普通日志每
20 个合格帧输出 `LOOP-PROGRESS`，其中 `gain_ppm`、`offset_mc`、`skew_cps`
是该窗口的有符号平均残差，分别使用 ppm、0.001 ADC codes、0.01 ps。
结束时 `LOOP-RESULT` 给出最后 20 个合格帧的均值和逐帧 SD。
普通闭环也启用原始 ADC clipping 拒绝，未改其学习率或 dither 公式。
看末段有符号残差和 `skew_abort=0`，不能把完成 300 帧等同于精度已验证。
Gain/offset 是软件数字校正系数，skew 使用 ACK 检查的硬件延迟事务。

**校准并自动保存到电脑：** 下载最新 ELF、正常上电并加载匹配 DPG 波形后，
关闭占用串口的终端，在仓库根目录执行下面一条电脑指令（COM7 按实际修改）：

```powershell
python tools/board_loop/board_loop_log.py --uart COM7 --out calibration_out/dither_run.log
```

脚本用一条 `adc -cal loop 300 stream` 完成校准和全程传输，全部逐帧校准
记录（含拒帧）实时写入 CSV，不受事后 RAM 快照条数限制。`.log` 只保留
简短进度和异常，不复制数据包。校验完整性后生成 `.json`、全程曲线 `.png`
和 `.svg`，包含全部合格点、20 帧均值、拒帧标记、A/B 系数和控制码。
CSV 使用 ADC codes、ps、乘法增益系数；图和 JSON 标明反馈仍开启。
复用文件名前缀会自动加时间后缀，保留之前结果。
已经校准过、只想下载结果时，电脑指令是：

```powershell
python tools/board_loop/board_loop_log.py --uart COM7 --cmd "adc -cal loop export" --out calibration_out/dither_saved.log
```

这不会重新采集或校准，也不写 ADC/延迟寄存器。只读保存的上一次普通 loop
快照；记录在 RAM 中，重启或下一次普通 loop 会清除/覆盖。最多保留最近
4000 条逐帧记录，若覆盖会明确记录缺失数量；该限制不影响上述 PC stream
全程传输。`loop test` 的详细日志和扫描
导出仍由 `tools/board_loop/board_axis_test.py` 处理。当前新版日志/导出接口只完成离线
和实际 Vitis 构建验证，尚需用户下载后上板检查。

算法以 `d22d23ff6753b4503c54157529ad8396483cb006` 的 Python dither 分支为基准，
`dither_raw.py` 的幅度和分数相位公式保持不变。此次进一步剥离了其外围的
tone 辅助处理：解帧只用时域相关，对齐只用脉冲模板；不调用 tone 纯度、
拟合、抵消、相位或载波指标函数。Offset 由脉冲平顶的极性解耦均值更新。
通过原生调用计数和 Python 禁用函数测试验证整条校准路径的 tone 调用为零。

Wave 3 幅度为 ±32767 DAC codes，即 DAC 数字峰值的 100%；正负各 32 个脉冲，
均值为零。到达 DAC 数字轨是这个已知输入的设计值，不代表有越界截断。
`touches_dac_rails=true` 与 `clipping=false` 分别表示触轨和无数字越界。
这不保证 ADC 输入达到满量程：ADC 端取决于模拟链路，仍须检查实际采样幅度。
原始 ADC 削顶保护保持启用，不能为了运行满量程 DAC 波形而关闭它。
其他幅度需要同步固件表并作为另一个条件记录，不能加载既有的
16000/30000 文件后选择 wave 3。Dither-only 的增益结果评价脉冲幅度比，
不代表主音处的增益比；下文单频相位的限制说明适用于后续 tone 对照。
本次 Vitis 2025.1 编译、链接已验证通过；仍需下载本次 ELF 并首次上板验证。

## 首次上板操作顺序（先跑通三个单点）

下面是可直接按顺序操作的首次测试流程。先跑通每项一个测试点，再做
第 6 节的重复阶梯扫描。串口开启日志保存；每条命令完成后再发下一条，
不要整段同时粘贴。正常测试命令结束会输出 `LOOP-TEST-END result=OK`。
`step 0` 则检查 `SKEW-TXN ... result=OK`；波形选择检查 `LOOP-WAVE`。

### 第一步：新固件和固定波形

1. 在 Vitis clean/build `thesis_v3_loop_appl`，下载新 ELF，完成正常 bring-up。
2. DPG 加载 `waveforms/dither_only/w06adc_e04_t04_amp32767_dither_only.txt`，
   即平衡正负 dither，无主音；加载 TXT，不是 JSON。
3. 保持 DAC 2.6 GS/s、ADC 1.3 GS/s，以及现有接线、幅度。

Dither 为 32767 DAC codes，脉冲上升/平顶/下降各 4 个 DAC 样本。
该 TXT 实际范围为 -32767..32767，均值为零。先固定这个幅度测失配阶梯。
这些数字不是 ADC 输入电压。先确认原始 ADC 数据未削顶。

### 第二步：建立基准，记录实际控制码 N

在串口终端逐条输入：

```text
adc -cal skew step 0
adc -cal loop wave 3
adc -cal loop test help
adc -cal loop test config 200000 500 1000 1910 20
adc -cal loop test baseline 600 100
adc -cal loop test status
```

基准阶段包含 600 次三项校准尝试，再全部冻结、进行 100 次独立测量。
上面的 config 为 dither-only 显式选择 ±0.5 sample 捕获门限；19.1 ps/code
只是启动控制值，后续仍须用本次工作点的配对步长测量复核并配置。
可能需要几分钟，等待完成，不要因没有立即返回而重复发送命令。

确认 `LOOP-TEST-BASELINE valid=1`，记录：

- 基准 `code=N`：本次实验的真实起点，不能预先假设是 12、24 或 35。
- `LOOP-TEST-SNAPSHOT channel=1 ifc=...`：物理 B 银行的基准 IFC。
  输出为十进制，例如 `ifc=12` 就是 `0x0C`。

若基准无效、出现错误 ACK 或 clipping，先排查，不进入下一个测试。

基准的独立 HOLD 门限是：合格率至少 80%、有符号 offset 均值不超过
±0.5 codes、dither gain 比残差不超过 ±0.2%，skew 不超过
`max(deadband, step/2) + 3*SD/sqrt(N)`。TRAIN 的全程平均误差不能代替
冻结后 HOLD 的结果。不要使用 HOLD 更新系数或放宽门限来获取有效基准。

2026-10-28 用户首轮日志：TRAIN 600/600、offset 均值 −0.018 codes；
HOLD 100/100、offset −1.623 codes、gain +0.1768%、skew −1.88 ps。
仅 offset 超出基准门限。HOLD 每 20 帧 offset 均值依次为
−1.089、−1.176、−1.610、−2.067、−2.171 codes，冻结期间还存在偏移趋势。
这证明基准不合格，尚不能确定是末态系数噪声、模拟漂移或估计偏差造成。
需要 TRAIN 末段逐帧系数和原始采样继续区分，不能据此改换 tone 路线。

本次诊断更新新增 `LOOP-TEST-BASELINE-CHECK`，逐项打印均值、门限和
`pass=0/1`；冻结阶段每 20 个合格帧另输出 `LOOP-TEST-WINDOW`，末尾不足
20 帧也单独输出。窗口仅作观察，HOLD 的反馈始终关闭。
`LOOP-TEST-BASELINE` 的 `wave/code` 指已保存的有效基准；无效时显示 −1。
`LOOP-TEST-CURRENT` 才是当前选定的波形和执行器控制码。旧 ELF 无效基准
显示的 `wave=0 code=0` 是未保存成功的字段，不表示当前正在使用 tone。
新诊断需重新构建/下载；不会修改 dither 公式、学习率或上述门限。

### 第三步：Offset 单点

当前 `test` 默认也使用简洁 UART 输出。电脑工具自动请求完整逐帧数据；
手动注入后关闭串口终端，可运行下列指令保存本次单项测试并自动作图：

```powershell
python tools/board_loop/board_loop_log.py --uart COM7 --cmd "adc -cal loop test run offset 40 300 100" --out calibration_out/offset_p16_r1.log
```

它只执行本次单项测试，不重建或覆盖已有基准。文件包含全部 PRE/TRAIN/HOLD
数据；图中标注阶段，JSON 使用冻结 HOLD 数据判断结果。自动 sweep 每个点、
每次重复也独立保存 CSV/JSON/PNG/SVG。需要下载新版 ELF 并重建一次 RAM 基准。

```text
adc -cal loop test restore
adc -cal loop test offset B 32
adc -cal loop test run offset 40 300 100
adc -cal loop test restore
```

这里在 B 的采样值上加 32 个原生 ADC codes 的**软件偏置**。
`40 300 100` 分别表示：40 次全冻结 PRE、300 次 offset-only TRAIN、
100 次全冻结 HOLD。Gain 和 skew 不参与调整。

先看 PRE 确实有 offset 变化，再看 HOLD 是否接近零。测试成功后，
再按相同步骤测 -32、正负 4、16、64 等点。硬件 DC 滤波开关对照见 5.3 节；
不要把这组软件阶梯写成模拟输入 offset 注入。

### 第四步：Gain 单点

**仅当基准 B 的 IFC 为 `0x0C` 时**，下面的 `0x0D` 才构成量程变化：

```text
adc -cal loop test restore
adc -cal loop test ifc B 0x0D
adc -cal loop test run gain 40 300 100
adc -cal loop test restore
```

如果基准已是 `0x0D`，改测另一个合法量程，例如 `0x0C`。
不能把“写入与基准相同的 IFC”当作施加了增益失配。

检查 PRE 幅度比确实变化，随后 HOLD 的 gain 残差应接近零，即
校正后的 dither 幅度比接近 1。此时 offset 和 skew 校正保持基准状态。

### 第五步：Skew 步长与一个恢复点

使用第二步读出的 N。**下例只是假设 N=12；实际不是 12 时必须替换数字。**

先做配对步长测量：

```text
adc -cal loop test step 12 55
adc -cal loop test restore
```

它依次测 12、13、12、13、12，每个状态尝试 55 帧。
比较 `step0/step2/step4` 的 skew 均值是否接近，`step1/step3` 是否接近，
两组差值就是当前工作点的一控制码响应。Gain/offset 始终冻结。
不要直接套用 19.1 ps/code；这条串口命令提供测量摘要，自动显著性和
重复性评分可用第 6 节的 Python `step` 模式完成。

然后从 N+1 测单项恢复；仍以 N=12 为例：

```text
adc -cal loop test park 13
adc -cal loop test run skew 40 300 100
adc -cal loop test restore
```

PRE 测初始 skew，TRAIN 只移动 skew 执行器，HOLD 全冻结测残差。
这里只设置了初始状态，控制器不知道它需要“退回一码”；它必须根据
实际采样的误差决定动作。如果初始误差已在死区内，也可能不移动。

### 第六步：读取独立残差，再扩大扫描

重点看 `LOOP-TEST-SUMMARY phase=hold`，不是只看 TRAIN 最后一行。

| 字段 | 换算为实际报告单位 | 例子 |
|---|---|---|
| `offset_mc` / `offset_sd_mc` | 除以 1000，校正输出 codes | 100 = 0.100 code |
| `gain_ppm` / `gain_sd_ppm` | 除以 10000，得到 % | 1000 = 0.100% |
| `skew_cps` / `skew_sd_cps` | 除以 100，得到 ps | 500 = 5.00 ps |

这些是有符号均值及帧间 SD；SD 不是均值偏差。检查 accepted/attempts，
保留拒绝原因。单个点 HOLD 均值小也不能代替多次重复的成功率。

三个单点都跑通后，再扩大正、负失配，每个点重复至少 5 次。
Skew 可测 N±1、±2、±4、±8，绝对控制码必须在 0..48；gain 使用合法 IFC
阶梯；offset 软件阶梯以 codes 为单位。每次都先恢复同一个基准。

如果使用 Python 自动化，先关闭手动串口终端，避免抢占同一 COM 口。
自动脚本会自己建立基准和恢复，不需要同时手动发送上述命令。
发生失败 ACK、方向错误或 clipping 时停止，不反复发写入命令。

## 1. 测试要回答的问题

对每一种失配，分别报告：

1. **捕获/校准范围**：从多大的正、负初始失配，能在固定预算内收敛。
2. **残差精度**：停止更新后，新的采样还剩多少有符号误差和波动。
3. **重复性**：同一初始状态重复运行的成功率，以及收敛需要的帧数。

三项同时收敛的历史记录是参考结果，不是单项极限。软件拒绝门限、
执行器边界、固定采集预算、噪声和算法发散也不是同一种失败。

| 测试项 | 扰动方式 | 实际覆盖的范围 |
|---|---|---|
| Offset 软件阶梯 | 在解帧后的一个通道加已知 DC codes | 采样后 offset 算法；不覆盖 ADC 前端过载 |
| Offset 硬件对照 | 单通道开启/关闭内部 DC 消除滤波器 | 芯片实际 offset 状态变化；不是已知大小的 offset 注入 |
| Gain | 单通道改 `0x1910` 的 IFC | 已实现的七种硬件量程；不是任意连续增益范围 |
| Skew | ACK 检查的相对时钟延迟控制码 | 当前执行器、输入频率及上电状态下的校准能力 |

**Offset 的限制需要明确：**现有寄存器接口提供 DC 消除开关，不能
把它当成一个可以设置任意 `+32 codes` 的 offset DAC。本工具不写未知
寄存器。`test offset B 32` 明确是软件注入；`test dcfilter B off` 才是
实际芯片寄存器操作。这两种实验在报告中必须分开。

## 2. 原理、符号和单位

数字校正沿用生产闭环的定义：

```text
y_cal = (y_raw + injected_offset - offset_state) * gain_corr
```

软件 offset 加在物理解帧后、校正前，不改变原始 DMA bytes，不通过
`rotation + 4` 重新切分通道，也不对采样记录翻转符号。

- 注入量、`offset_state`：原生有符号 14-bit ADC codes。
- Offset 残差：校正输出的 `offset_B - offset_A`，单位是校正后的 codes。
  若冻结的 `gain_corr` 不等于 1，注入 32 个原生 codes 的初始残差
  不一定恰好是 32 个输出 codes。不要未经转换就写成实测 µV。
- Gain 残差：`mag_B / mag_A - 1`，乘 100 得到百分比。
  它是脉冲的相对通道增益，不是绝对 volts/code。
- Skew：正值表示 B 采样时刻更晚。Dither-only 使用两路折叠脉冲的分数
  采样相位差；通路脉冲形状差异也可能影响这个估计，仍须配对步长复核。
  不计算 tone 相位；也不能把这个结果当作独立的绝对时基标定。

修改 B 的 IFC 时，名义幅度变化为：

```text
amplitude_after / amplitude_before = VFS_B_before / VFS_B_after
```

例如 `0x0C -> 0x0D` 对应 `1.59 / 1.70 - 1 = -6.47%`。
这只是名义寄存器比例；实际初始失配以 PRE 采样为准。

Skew 不用固定的标称 ps/code 当作注入真值。先测当前工作点的一码步长：

```text
N -> N+1 -> N -> N+1 -> N
```

每个状态都冻结校正、采集新帧。工具比较两次步进的方向、显著性，以及
两次返回 N 是否恢复到基线。这里是步长/重复性测量，不是独立的绝对
时间标定；使用同一执行器注入和纠正也不能单独证明时基准确度。
当前控制模型要求增大控制码使 skew 增大。若两次都测到负方向，结果
仍会保留，但不标 PASS；先核对通道/测量路线，不自动翻转控制方向。

## 3. PRE / TRAIN / HOLD

一次 `test run AXIS PRE TRAIN HOLD` 包含三个阶段：

| 阶段 | 允许更新 | 作用 |
|---|---|---|
| PRE | 无 | 测扰动后的初始失配 |
| TRAIN | 仅指定项 | 测单项收敛 |
| HOLD | 无 | 用新采样独立测残差 |

每个测试点都从保存的数字基准系数开始，避免继承上一个测试点的
学习结果。非待测项冻结在基准值，而不是简单清零。

所有预算均为**尝试帧数**，包含拒绝帧。它与旧命令
`adc -cal loop 300`（收集 300 个合格帧）不同。新测试不会只挑合格帧
来隐藏低成功率。连续三次采集失败会提前停止当前阶段。

`loop open` 仍保留原行为：只关闭 skew 执行器，gain/offset 仍在学习。
严格独立测试应使用本文件的 `test` 命令。

## 4. 上板前准备

1. 在 Vitis 构建并下载 `thesis_v3_loop_appl`，按现有流程完成硬件 bring-up。
   不需要修改 Vitis 生成的 `src/` 或链接脚本。
2. 本次 DPG 加载满量程 wave 3；wave 2 可作低幅度对照：

   | Wave | 文件 | Tone |
   |---|---|---|
   | 3（本次默认） | `waveforms/dither_only/w06adc_e04_t04_amp32767_dither_only.txt` | 无主音 |
   | 2 | `waveforms/dither_only/w06adc_e04_t04_amp5190_dither_only.txt` | 无主音 |
   | 0（校准禁用） | `waveforms/tone_dither/w06adc_e04_t04_amp5190_tone_m1p5.txt` | 199.375 MHz |
   | 1（校准禁用） | `waveforms/tone_dither/w06adc_e04_t04_amp5190_tone_m1p5_f399p375.txt` | 399.375 MHz |

   纯 function-generator 正弦、未登记的 dither-only 波形、旧的宽脉冲向量都不能
   直接替代所选波形。固件不能可靠地自动识别所有错误向量。
3. 两通道输入同源，保持接线、幅度、采样率和温度条件一致，不让 ADC 削顶。
   先确认物理 A/B 寄存器银行与导出的通道标签对应；不要靠储存的 DC
   数值判断通道，也不要为交换通道把解帧旋转加 4。
4. 首次使用执行器：`adc -cal skew step 0`。应看到 `result=OK`。
   首次会进行 neutral 初始化；后续 `step 0` 不会自动回到 neutral。
5. 选择波形：`adc -cal loop wave 3`。关闭占用串口的其他终端再运行 Python。
6. 确定实际 COM 端口。以下 `COM7` 只是示例，不能假定每台电脑相同。

Skew 写入必须走固件事务：读回、JESD 重同步、warm-up 和新 DMA 帧检查。
本工具不直接写 `0x0110/0x0112/0x0114`。遇到失败 ACK 或反方向响应，
测试锁定停止；不通过再次写寄存器尝试恢复。

## 5. 串口命令

先查看 `adc -cal loop test help`。

### 5.1 建立和保存基准

```text
adc -cal loop test baseline 600 100
adc -cal loop test status
```

前 600 次尝试开启三项校准，后 100 次全部冻结。通过后保存：数字
校正系数、极性锚、波形编号、实际执行器码，以及 A/B 两银行的
`0x1910/0x0701/0x073B` 原值。读取或写入本地寄存器都会恢复原通道选择。

基准有效还要求 HOLD 合格率至少 80%，offset 均值在 0.5 输出 code 内、
gain 均值在 0.2% 内，skew 均值接近当前死区/半步长允许的范围。
失败时不保存为有效基准。这里的判据只是实验准备条件，不是新测得的极限。

### 5.2 Offset 软件阶梯

```text
adc -cal loop test restore
adc -cal loop test offset B 32
adc -cal loop test run offset 40 300 100
adc -cal loop test restore
```

重复 `-64,-16,-4,0,4,16,64` 等正、负注入点。范围支持
`-4096..4096` 个原生 codes。注入在 ADC 之后以浮点数相加，不模拟
ADC 饱和、模拟输入偏置或整数截断。

### 5.3 Offset 芯片开关对照

```text
adc -cal loop test restore
adc -cal loop test dcfilter B off
adc -cal loop test run offset 40 300 100
adc -cal loop test restore
```

也可用 `on`。这是改变内部 DC 消除状态，保留寄存器的其他位并核对
读回。开启滤波器时会清除 `0x073B` 的 bit 7。
使用 PRE 测实际 offset 变化并观察其是否稳定；它不是已知幅度阶梯，
滤波器在运行时还可能改变被校准对象。

### 5.4 Gain 单通道寄存器阶梯

```text
adc -cal loop test restore
adc -cal loop test ifc B 0x0D
adc -cal loop test run gain 40 300 100
adc -cal loop test restore
```

合法 IFC 码：

| Code | 名义差分满量程 |
|---|---|
| `0x00` | 2.04 Vpp |
| `0x0A` | 1.36 Vpp |
| `0x0B` | 1.47 Vpp |
| `0x0C` | 1.59 Vpp |
| `0x0D` | 1.70 Vpp |
| `0x0E` | 1.81 Vpp |
| `0x0F` | 1.93 Vpp |

仅改一个银行，保留 `0x1910` 高四位并验证读回。HOLD 评价 dither_mag 的
增益比。七个 IFC 点全部通过时，只能
说覆盖了这些硬件状态；还没找到更大的算法失配边界。

### 5.5 Skew 步长与单项范围

先用 `status` 中的当前基准码 N 进行配对测试。例如 N=12 时：

```text
adc -cal loop test step 12 55
adc -cal loop test restore
```

它测 `12,13,12,13,12`，每个状态尝试 55 帧，结束时停在 12；之后
`restore` 回到已保存基准。N 不是固定的 12、24 或 35，取决于本次上电。

再测单项恢复，例如假设基准 N=12，先从 N+4=16 开始：

```text
adc -cal loop test restore
adc -cal loop test park 16
adc -cal loop test run skew 40 300 100
adc -cal loop test restore
```

分别测试 N±1、±2、±4、±8，并在失败边界细扫。`park` 接受绝对控制码
`0..48`，通过 ACK 检查的单步事务到达，不会静默截断越界请求。
`step` 的 N 范围为 `0..47`，因为要测 N+1。

一个周期频率的 phase 残差不是全频带真实时延的独立标定。换到另一主音
复核时，尽量保持同次上电；重新建立对应波形的基准，不混用两套数字状态。

### 5.6 门限、死区、控制步长

```text
adc -cal loop test config 200000 250 1000 1910 20
```

五个参数依次为：

| 参数 | 上例意义 |
|---|---|
| Gain gate，ppm | 200000 = 允许幅度比偏离 1 达 20% |
| Skew gate，千分之一个采样周期 | 250 = ±0.25 个采样周期 |
| Deadband，0.01 ps | 1000 = ±10 ps |
| 控制用步长，0.01 ps/code | 1910 = 19.1 ps/code |
| Batch frames | 20 个合格帧决定一次 skew 移动 |

参数只影响新 `test` 流程，旧 `loop N` 保持默认行为。
手动流程的默认 skew gate 为 250；本次 dither-only 使用 500，见首次流程。
Python 扫描允许 wave 2/3，默认采用 500，可显式覆盖。
Skew gate 最大允许 0.5 个采样周期。不要为得到通过结果无限放宽门限。
允许较大失配的扫描可显式改 gain gate，例如 `300000`；报告时保留该设置。

降低 deadband 本身不保证精度提升：控制动作先按 `round(-error/step)`
量化，再限制为一次最多一控制码，小于半个控制步长可能已经不产生移动。
当前工作点步长和噪声都应单独测量。
这种停止会记录为 `quantized-zero`，与实际执行器边界分开。

TRAIN 的尝试预算还必须足够：20 帧一批时，300 次尝试最多只有 15 个
完整批次；若初始状态需要超过 15 次移动，它失败说明预算不足，不等于
执行器或算法不能收敛。

## 6. Python 自动扫描和导出

从仓库根目录执行，需要 `pyserial`。默认只显示计划，不连接串口。
先查看计划，再加 `--execute`。每次使用一个**不存在的新输出目录**，
脚本拒绝覆盖旧结果。

默认三项扫描共需较长台架时间。先用一个小测试点、一次重复确认流程，
再运行完整阶梯；例如 `--points=1 --repeats 1` 做一个 skew 点。
`--timeout` 是每条完整命令的等待上限，默认 1800 秒；增加帧数，尤其
step 的五个状态都使用大帧数时，需要相应提高这个超时。

```powershell
# Offset：注入量为软件原生 ADC codes
python tools/board_loop/board_axis_test.py sweep --axis offset --uart COM7 --wave 3 --points=-64,-16,-4,0,4,16,64 --repeats 5 --out calibration_out/limits_offset_01

# Gain：这里填的是 IFC 寄存器码，不是百分比
python tools/board_loop/board_axis_test.py sweep --axis gain --uart COM7 --wave 3 --points=12,13,11,14,10,15,0 --gain-gate-ppm 300000 --repeats 5 --out calibration_out/limits_gain_01

# Skew：相对于本次收敛基准的控制码偏移
python tools/board_loop/board_axis_test.py sweep --axis skew --uart COM7 --wave 3 --points=-8,-4,-2,-1,0,1,2,4,8 --repeats 5 --out calibration_out/limits_skew_01

# 配对步长：默认在本次收敛基准码 N 测 N/N+1；也可指定 --code N
python tools/board_loop/board_axis_test.py step --uart COM7 --wave 3 --frames 55 --out calibration_out/limits_step_01
```

确认硬件和波形就绪后，在所需命令末尾加 `--execute`。
每次脚本启动先建立基准；每个点/重复开始时 restore，注入后运行
PRE/TRAIN/HOLD，全部结束后正常 restore。脚本只设置固件假定的波形编号，
不替用户加载 DPG。换输入连接、重新上电、重载 ELF 或更换波形后，需新基准。

对每个点默认 PRE=40、TRAIN=300、HOLD=100 次尝试；可用
`--pre/--train/--hold` 修改。基准默认 600+100。一步范围越界会明确报错，
不会把 N-8 静默改成 0。

默认判据：

- Offset：0.5 个校正输出 code。
- Gain：0.001，即 0.1% 幅度比残差。
- Skew：10 ps。

用 `--tolerance` 修改，单位分别为 codes、比例、ps。
例如 gain 的 0.05% 写 `--tolerance 0.0005`。这些是声明的成功标准，
不是已经验证的硬件能力。

HOLD 至少要有 80% 合格帧，且 `abs(mean) + 1.96*SE <= tolerance`。
还检查系数是否冻结、HOLD 是否接续最终训练状态，以及是否碰到执行器边界。
`convergence_attempt` 来自 TRAIN 的稳定滑动窗口，是辅助读数；最终结果
以独立 HOLD 为准。SE 是帧间独立假设下的近似值，序列相关会使它偏小，
因此保留 SD，并用多次独立重复验证，不能仅凭很小的 SE 宣称绝对精度。

输出：

| 文件 | 内容 |
|---|---|
| `console.log` | 每条命令、完整串口输出、失败原因和逐点结果 |
| `frames.csv` | 阶段、尝试序号、三项残差、校正系数、拒绝标签 |
| `summary.csv` | 初始失配、HOLD 均值/SD/SE、成功率所需逐次结果、收敛帧数 |
| `meta.json` | 参数、单位、基准码、是否完整结束；step 模式还含步长比较 |

`pre_mean` 只使用 PRE 合格帧。`pre_observed_mean` 还保留被 gain/skew
门限拒绝帧的有限估计，用来辨认软件门限；这些帧不会参与校正。
失败事务发生时仍保留 `console.log`，CSV 保留已经完成的测试点。

也可以只记录一次手动运行并离线分析：

```powershell
python tools/board_loop/board_loop_log.py --uart COM7 --cmd "adc -cal loop test run skew 40 300 100" --out calibration_out/skew_single_01.log
python tools/board_loop/board_axis_test.py analyse --log calibration_out/skew_single_01.log --axis skew --hold 100 --tolerance 10
```

`analyse` 用于单次 run/step 的日志；不要把完整多点 sweep 日志当成一个点。

## 7. 失败的解释和恢复

| 记录 | 应如何解释 |
|---|---|
| `gain_range` / `skew_range` | 当前接收门限阻止捕获，不直接证明估计器失效 |
| `margin` / `events` / `tone` | 波形、对齐或信噪比不满足条件 |
| TRAIN 预算用尽且残差仍大 | 本预算下未收敛；先检查最多可完成的控制步数 |
| `at-range-limit` | 执行器调节范围耗尽 |
| `quantized-zero` | 校正误差量化为零码，受控制步长限制，未必到硬件边界 |
| `inside-deadband` | 软件停止条件，不自动等于噪声或精度极限 |
| `clipping` | 原始 ADC 数据触及数字轨，不能靠校正把过载帧伪装成正常 |
| `move:FAILED` / `RECOVERY_REQUIRED` | 事务失败；锁定停止，无写入重试 |
| `abort:direction` | 实测方向与控制模型不一致；锁定停止 |

正常完成可以 `test restore` 回到已保存基准。失败写入后本工具不自动
恢复；先只读排查，再重启/重载 ELF、完成硬件 bring-up 并建立新基准。
软件故障锁保持到固件重新启动；只重新执行外设配置不会清除该锁。
中断或超时也不会自动再发硬件命令，因为最后一个事务可能尚未完成。
不要在仍有未完成串口命令时重复启动脚本。

最终汇报按项给出“初始实测失配 -> HOLD 残差”，并附 SD、重复成功率、
输入频率/幅度、门限和预算。若扫描到的所有点都通过，应报告
“至少覆盖已测范围”，不要把最大已测点写成已经找到的校准极限。

## 8. 离线验证与来源

```powershell
cmake -S calibration_sim -B calibration_sim/build-axis -G Ninja -DCMAKE_C_COMPILER=C:/Strawberry/c/bin/gcc.exe
cmake --build calibration_sim/build-axis
calibration_sim/build-axis/adc_cal_sim.exe --run-loop-tests --output-dir calibration_sim/output/axis_native
python calibration_loop_c/export_golden.py --out calibration_sim/output/axis_golden.txt
calibration_sim/build-axis/adc_cal_sim.exe --loop-golden calibration_sim/output/axis_golden.txt --output-dir calibration_sim/output/axis_golden_replay
calibration_sim/build-axis/adc_cal_sim.exe --run-all --output-dir calibration_sim/output/axis_all
python tools/board_loop/test_board_axis_test.py
python checks/test_dither_only.py
```

Native 测试验证真实生产控制器的单项冻结、独立 HOLD、注入和 clipping；
SPI/UART spies 编译真实板级实验片段并验证单通道选择、读回、恢复及失败锁定。
Python 测试验证日志解析、缺帧/NaN/边界不能误报通过，以及失败后不重试。
这些不能替代上板验证，不能证明 JESD、模拟噪声或真实 SPI 时序。

寄存器说明来自 [AD9695 Rev. C datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ad9695.pdf)：
第 30 页 DC offset calibration，第 133 页 IFC 和本地控制寄存器。
现有事务/硬件流程见 `firmware/thesis_v3_loop_appl/README.md` 和
`calibration_loop/BENCH_GUIDE.md`。
