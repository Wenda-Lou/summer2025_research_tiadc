# AI 交接记录：单项 offset / gain / skew 校准极限

记录日期：2026-10-27。入口已写入仓库根目录 `AGENTS.md`。
详细原理与全部命令：[`CALIBRATION_LIMIT_TEST.md`](CALIBRATION_LIMIT_TEST.md)。

## 2026-10-28 最新要求：无 reference 的板端校准

首轮实测新模式 0/300，20 帧全部 margin=0，尚无校准更新。
检查发现 reference_free 提前返回绕过原 de-framer 的 B 通道反相处理，
负相关直接得到 margin=0，互谱也带入了额外的 pi 相位。
已把原有极性归一化放到两个估计路径之前，质量门控使用 |corr|，
同时保留原始有符号 corr 并在逐帧诊断打印 corr_ppm；不使用参考或 tone。
无效估计的 gain_ratio 改为 NaN，避免出现误导的 -1000000 ppm。
回归覆盖两种相对极性、三种脉冲宽度、闭环和独立 HOLD、独立噪声拒绝。
原日志未记录 corr，硬件是否确为反相仍需用新 ELF 重跑确认，不能宣布实测修复。

用户明确禁止任何 reference，此要求取代之前按 d22d23f 的模板 dither_phase 路径。
板端普通 loop 和 loop test 现在默认 reference_free=1，只保留 ADC 1.3 GHz 硬件采样率。
loop_waveform.h 已移除波形表；旧 wave 命令仅兼容、不选择模板，不再需要调用。
核心新分支 adc_calibration_loop_blind.h 使用 A/B 共同宽带 dither 的频谱幅度比、
互谱相位斜率、去除增益影响的 DC 截距，估计通道间相对 gain/skew/offset。
不构造 loop_ref，不用周期、幅度、脉宽、极性表、tone 拟合或 pulse template。
UART 路由为 dither_spectrum / dither_cross_spectrum，导出和绘图支持这些路由。
仅能确定相对误差；无参考不能恢复共同绝对增益、共同 DC 或绝对延时。
需要 A/B 看到同一宽带 dither，模拟传递形状可用增益和相对延时描述；
常量、单频、弱激励和不一致数据拒绝，不能承诺任意波形必过。
Python 原有校准入口与 C 历史模式仍保留模板实现，不代表无参考路径。
新板端模式尚未上板验证；此前 dither_phase 的实测通过结果不等于新模式已验证。

## 2026-10-28 baseline offset 冻结修正

后续 offset 自动扫描完成 14 次，8 PASS、6 FAIL（随后 KeyboardInterrupt），
所有失败均为 offset HOLD 判据，采集、轴隔离和冻结检查正常。
现将同样的训练末段输入域平均扩展到 `test run offset`，不改变阈值。
平均系数在最后一个合格 TRAIN 帧输出前提交，并同步该行的 post-update 状态，
确保 HOLD 与最后 TRAIN 记录一致。最后一次尝试失败/拒绝、样本不足 20 或故障时不应用。
日志 `LOOP-TEST-FREEZE ... phase=train` 标明单轴平均；HOLD 数据从不参与计算。
新 ELF 下载后基准清空，自动扫描不带 --reuse-baseline 即会先重新建立基准。
旧扫描数据保留，新策略尚需上板重测，不能把旧结果改判为通过。

用户基准 HOLD offset +0.612 codes 超过 ±0.5，训练结束原先直接冻结
mu_offset=0.35 的末帧状态。现在 baseline_train 使用最后 min(100, TRAIN/2)
个尝试中合格帧的输入域 offset 估计均值：更新前 offset + 残差 / 更新前 gain。
skew 控制码变化时清空累积并跳过移动帧，至少 20 帧才应用均值；不足则保留原状态。
打印 LOOP-TEST-FREEZE，随后使用全新 baseline_hold 验收，阈值不变。
仅基准冻结策略改变；普通 loop、单轴 TRAIN、核心 dither 估计器和学习率不变。
没有使用 HOLD 数据修正系数，也没有失败后自动重试。硬件通过率仍需上板验证。

## 2026-10-28 全部逐帧记录与自动生图（当前入口）

最新修正：用户实际使用 `loop test`。该入口现在默认也只打印阶段、20 帧
窗口、总结和异常；PC 工具对 baseline/run/step 自动追加一次性 `full`，
保存完整数据并按 PRE/TRAIN/HOLD 作图，不再持续刷逐帧 UART 日志。
`tools/board_loop/axis_result_export.py` 输出每个测试点/重复的 CSV、JSON、PNG、SVG，
HOLD 统计和判断使用独立冻结数据。普通 loop 的功能保留，校准算法未改。
验证：native 432 通过，Python 单项协议 12 项及导出/绘图 17 项通过；
实际 Vitis 2025.1 构建通过。新 ELF 下载会清掉 RAM 基准，必须重建一次。

用户明确：后续测试导出全部逐帧校准记录并自动生图；UART 日志简化；
不需要导出刚跑完的旧记录，不包含 ADC 原始采样波形。估计公式、反馈
参数、dither 路线和时钟事务保持不变。

下载新 ELF、正常上电/加载匹配满量程 dither TXT 后，关闭占用串口的
终端，在仓库根目录运行一条电脑指令（COM7 按实际修改）：

```powershell
python tools/board_loop/board_loop_log.py --uart COM7 --out calibration_out/dither_run.log
```

PC 用一条 `adc -cal loop 300 stream` 完成校准和全程传输。每条校准记录
（含拒帧）实时写入 CSV，不受板上 4000 条事后 RAM 快照限制。UART .log
只留每 20 个合格帧的进度、异常和结果，不复制数据包。完成时校验 run_id、
行数和合格数，再生成 JSON、PNG、SVG。图保留全程三项残差、A/B gain/
offset 系数和执行器码，区分每个合格点、20 帧均值和拒帧标记；标明反馈
仍开启，不代表独立冻结精度。复用输出名前缀自动加时间后缀；通信中断
保留已收到的 CSV 行。刚跑日志的临时末段导出已经移除，不继续处理旧数据。

普通 `adc -cal loop 300` 仍为简洁串口模式；`loop test` 保持详细实验日志。
可选 `loop export` 只读快照功能保留，但不是当前全程工作流的必要步骤。
新增源码：`firmware/thesis_v3_loop_appl/butils_loop_results.c`、
`tools/board_loop/loop_result_export.py`、`tools/board_loop/loop_result_plot.py`。

验证：native 424 项通过、0 失败，包含超出 RAM 容量仍逐条发送的检查；
PC 协议/绘图测试验证真实 C stream、全部行数、单位、拒帧标记和全程生图。
实际 Vitis 2025.1 构建通过；结果在
`calibration_sim/output/full_trace_plot_native/`。protocol_fixture 图为离线
测试数据，不是真实台架测量。新日志/导出接口尚未上板，本代理未连接串口。

## 2026-10-28 首轮用户台架日志：基准因 offset 被拒绝

用户进一步明确：当前重点是用一条命令校准 dither-only 波形。DPG 加载匹配的
满量程 dither TXT、正常 bring-up 后，只需 `adc -cal loop 300`；启动默认
wave 3。wave/status 是配置/查询，不是必需校准步骤；不要求先通过单项实验
的 `test baseline`。板端输出移除了无关载波元数据和 `tone_ppm`，波形选择
只允许 dither 配置，历史 tone 表项仅保留作源码/离线参考。
普通闭环启动现明确打印三条 dither 路线，波形选项也对普通入口启用 ADC
clipping 拒绝。Gain/offset 是数字系数，skew 是 ACK 检查的硬件延迟事务。

用户已构建并运行本次固件；本代理没有连接或操作台架。三项路线标签是
`gsrc=dither_mag skew_route=dither_phase`、`tone_ppm=nan`。TRAIN 600/600，
offset 均值 −0.018 codes；随后冻结 HOLD 100/100，offset −1.623±1.398 codes
（均值±逐帧 SD）、gain +1768 ppm、skew −1.88 ps，clips=0、abort=0、fault=0。
基准 ±0.5-code 门限正确拒绝了 offset；gain 和 skew 通过各自基准门限。

从用户逐帧 HOLD 转录的分析在
`calibration_out/diagnostics/baseline_hold_20261028/hold.csv` 和 `analysis.json`。
20 帧分段 offset 均值为 −1.089/−1.176/−1.610/−2.067/−2.171 codes。
冻结期间偏差增长，不能只归因于最后一帧噪声；没有 TRAIN 末段系数或原始
DMA 采样，物理原因未确定。不能把 TRAIN 平均值当成已保存基准精度，不能
反复筛选直到偶然通过，也不能用 HOLD 调参、放宽门限或改回 tone。

本次仅改诊断：基准逐项 CHECK、冻结阶段 20 个合格帧的独立 WINDOW 统计；
无效基准的 wave/code 输出 −1，并通过 CURRENT 另报当前选择和执行器码。
旧 `valid=0 wave=0 code=0` 是尚未写入的保存字段，并非实际当前波形/控制码。
估计公式、学习率、冻结方式、时钟事务和验收门限保持不变。
下一步需要检查 TRAIN 最后 20–40 行的系数演变，区分冻结末态与后续偏移。
本次诊断版仍需用户重新下载；不会宣称台架问题已经解决或基准已通过。

最终单命令版已用本机 Vitis 2025.1 A53 工具链重新构建成功，无新增警告。
输出 `firmware/thesis_v3_loop_appl/build/thesis_v3_loop_appl.elf`，SHA-256：
`98313d7a9edfc3ca0d1ca2b87f12c08bed6f2f0403d59a8fb0710ec398e2c4bc`。
C native 406 项通过、0 失败（`calibration_sim/output/dither_single_command_native/`）；
Python dither 专项 5 项、扫描协议 12 项通过。另核对 `dither_raw.py` 与指定
commit 的内容一致（仅忽略工作区 CRLF 差异）。没有修改原始 dither 公式，
没有下载 ELF、执行新的台架命令或宣称独立冻结精度通过。

## 2026-10-28 Vitis 2025.1 构建修复

用户机器是 Vitis 2025.1，应用模板来自 2025.2，缺少 `print_build_info_target`
导致配置阶段失败。通过用户可编辑的 `src/UserConfig.cmake` include 应用本地
`cmake/vitis_compat.cmake`，加载当前 BSP 后仅补齐缺失的构建信息打印函数。
没有手改生成的主 CMakeLists、导出的 BSP、平台或链接脚本。实际用本机
2025.1 A53 GCC 13.3 完成了配置、编译和链接，输出
`firmware/thesis_v3_loop_appl/build/thesis_v3_loop_appl.elf`。
后文“尚未构建”的记录已过时；仍未下载 ELF 或进行本次硬件测试。

## 2026-10-28 满量程更新（优先于下方幅度配置）

用户指出 5190 codes 不是满量程。本次改用 **DAC 数字峰值满量程 ±32767**：
`waveforms/dither_only/w06adc_e04_t04_amp32767_dither_only.txt`。
新增 `wave 3` 并设为固件和扫描脚本默认；wave 2 保留 5190 codes 对照。
两者都只用 dither 校准，tone 0/1 仍禁用。脉冲形状、周期、极性和采样率不变。
重新构建/下载 ELF 后必须选择 `adc -cal loop wave 3` 并加载匹配 TXT。

向量均值为零，正负各 32 脉冲，320 个样本合法到达 DAC 轨，无数字越界。
生成器元数据现区分 `touches_dac_rails` 和 `clipping`；ADC 原始轨检测没有放宽。
DAC 满量程不代表 ADC 输入满量程，不能把这个文件当作模拟输入满量程的证明。
历史台架只验证到 30000 codes，32767 尚未上板。满量程模型预检显式采用
0.20 ADC codes/DAC code 的链路比例以留出 ADC 余量，60/60 帧合格、无 ADC 触轨；
末段残差：gain ratio 0.999965、offset -0.0084 codes、skew -0.0138 ps。
这只是软件预检，实际链路仍须检查 clipping。生成器/纯 dither 测试 5 项、
扫描协议 12 项通过；C 原生加 9 个 golden 场景共 939 项通过、0 失败，输出见
`calibration_sim/output/fullscale_dither_verified/test_summary.txt`。

## 2026-10-28 最终约束：校准必须基于 dither

用户已再次确认：calibration 必须用 dither，不能依赖 tone。
指定基准 commit `d22d23ff6753b4503c54157529ad8396483cb006` 的 C 默认实现
使用 tone；当前采用的是该 commit 的 Python dither 分支公式。
`calibration_loop/dither_raw.py` 与该 commit 完全一致，未改其幅度比、分数
相位拟合和差分时序公式。外层处理已进一步去掉隐含的 tone 依赖。

本次有效路径：

- 解帧：时域 A/B 相关和半组 lag 检查，无 tone 纯度评分。
- 对齐：直接与已知 dither 模板相关，无预先的 tone 拟合/减除。
- Offset：脉冲窗口的极性解耦平顶统计，无 tone-fit DC。
- Gain：dither_mag，逐事件峰峰值幅度比，无 tone 回退。
- Skew：dither_phase，折叠脉冲的分数采样相位差，不用正弦相位。
- 筛选、批次、反馈和日志均跟随 dither 路线；载波指标不计算。

固件启动默认 wave 2；`loop` 和 `loop test` 在任何采集或校准写入前拒绝
非 dither 配置。扫描脚本也只允许 wave 2。历史 tone 代码仅作离线回归参考，
不接入当前板端校准入口。Wave 2 已设 `sig_cycles=0`，生成器不产生正弦分量，
JSON 中主音频率和幅度均为 0；TXT 的 SHA-256 与此前的纯 dither 样本一致。

专项验证：原生测试对每次 tone fit/purity 调用计数，dither 路径计数为 0；
Python 将所有 tone helper 设为调用即报错，闭环仍收敛。两者均验证相同采样
改变无关 tone 参数时结果不变；无 dither、只有正弦的输入不会触发反馈。
原生独立测试 375 项通过；Python 专项 4 项、扫描/协议 12 项通过。
完整 C + golden 回归 6601 项通过、0 失败，18 scenarios、14 pipelines；
tone-free 和 direction 两份离线证明均 PASS。
结果在 `calibration_sim/output/strict_dither_final/test_summary.txt`。

仍未进行本次 Vitis 构建/ELF 下载和上板测量。下面的初期验证记录保留为历史；
涉及路线选择时以本节及当前测试文档为准。

## 2026-10-28 初期适配记录（已由上节严格路径取代）

用户已明确要求用 dither-only 跑相同的单项极限测试。当前工作区已给
`calibration_loop_c` 增加 `tone_free` 路线：dither_mag 增益、分数模板
相位 dither_phase 时序、无主音抵消、无 tone 回退、按路线门限和批次 MAD
剔除。Offset 保留平顶估计；PRE/TRAIN/HOLD、单步 ACK 和故障锁规则不变。
固件 `wave 2` 对应
`waveforms/dither_only/w06adc_e04_t04_amp5190_dither_only.txt`，幅度 ±5190，
edge/top=4/4 DAC samples，采样率和种子不变。旧 wave 0/1 保留供对照。

`test status` 应显示 `gain=dither_mag skew=dither_phase tone_free=1`。
逐帧 `used_gain_ppm` 为被控制的增益残差，`mag_ppm` 为 dither 比，
`tone_ppm=nan` 表示主音不存在；脚本评分当前路线并核对路线标签一致性。
Python 扫描用 `--wave 2`，仍默认 PLAN ONLY。

新增原生测试覆盖正负已知延迟、单项更新、HOLD 冻结、诚实/反方向执行器和
批次异常值；golden 文件新增四个 dither-only 场景，比较所驱动的估计值、
系数和动作。历史 tone 诊断在 dither-only 输出中记 NaN，不解释为有效测量。

尚未进行本次固件的 Vitis 完整构建/下载或硬件测试。上板前按完整测试文档
开头的 wave 2 流程重新构建并下载 ELF。以下 2026-10-27 内容保留为历史交接，
涉及 excitation 时以本节为准。

本次离线验证（2026-10-28，Windows GCC 14.2，CMake RelWithDebInfo）：

- `adc_cal_sim --run-all`：6077 通过、0 失败，18 scenarios、14 pipelines。
- 新生成的 Python golden，8 个场景共 280 帧，C replay：489 通过、0 失败。
- `python tools/board_loop/test_board_axis_test.py`：11 项通过；wave 2 的 PLAN ONLY
  输出确认默认 skew gate 为 500，不打开串口。
- 原生 dither-only 时钟批次：诚实执行器 8 步后残差 -2.80 ps，无锁定；
  反方向执行器只走 1 步后锁定。正负已知延迟及冻结测试均通过。
- 输出：`calibration_sim/output/dither_only_all/test_summary.txt` 和
  `calibration_sim/output/dither_only_replay/test_summary.txt`；构建目录
  `calibration_sim/build-dither/`。测试产物不作为源码提交。
- 旧 `timing_alignment.c` 仍有两处 float-conversion 警告，新代码无新增警告。

## 用户当前目标（2026-10-27 历史记录）

教授要求分别测 offset、gain、skew 的校准范围和校准后残差。
用户希望通过改变单通道寄存器制造失配，skew 通过采样时钟延迟制造失配。
功能和操作说明已实现；下一步是构建新固件并进行第一次真实台架验证。

不要把历史“三项同时收敛”的记录当成新功能的单项极限结果。
不要把 ADC 绝对满量程验证与这里的相对失配校准混为一谈。

## 当前状态：已实现，但未上板

- `firmware/thesis_v3_loop_appl/butils_loop_test.c`：新 `adc -cal loop test` 命令。
  由 `butils_loop.c` include，与现有捕获/时钟事务共用生产接口。
- `calibration_loop_c/adc_calibration_loop.h/.c`：`update_axes` 选择更新项、
  显式软件 offset 注入、实验用 clipping/tone 检查；默认仍更新全部三项。
- `tools/board_loop/board_axis_test.py`：配对步长、重复阶梯扫描、日志/CSV/JSON 和 HOLD
  评分。默认 PLAN ONLY，只有 `--execute` 才打开串口。
- `tools/board_loop/board_loop_log.py`：支持等待新命令的 `LOOP-TEST-END`，ERROR 返回失败。
- 测试：`calibration_sim/sim_loop_tests.c`、`sim_board_axis_tests.c`、
  `tools/board_loop/test_board_axis_test.py`。

这些修改目前在工作区，包含尚未跟踪的新文件，**没有创建提交**。
继续工作时先查看 `git status`，不要覆盖用户后续修改或清理新文件。
没有运行 Vitis 完整构建、没有下载 ELF、没有连接 COM 口或操作台架。
桌面测试成功不能作为真实 SPI/JESD/模拟测量已经成功的证据。

## 已执行的离线验证

- Python `sim` 改动前后均完成 60/60 合格帧，0 拒绝；最后均值：
  tone ratio 0.999997、offset -0.0083 codes、skew -0.0037 ps。
- 新 C native 测试：218 项通过，0 失败。
- 新生成 Python golden 文件并完成 C replay，无差异失败。
- 最终 `--run-all --loop-golden`：6136 项通过，0 失败；18 scenarios、14 pipelines。
- Python grading/protocol 测试：10 项通过。
- `git diff --check` 通过。构建时有未改动旧模块的 float-conversion 警告；
  新模块没有新增编译警告。

可复查输出位于 `calibration_sim/output/axis_verified/test_summary.txt`。
构建目录 `calibration_sim/build-axis/`。这些是可重新生成的忽略文件，
不要当作源码提交。

## 波形和幅度：先保持现有配置

新实验仍使用已有 **tone + dither** 闭环，不是仅输入一个 function-generator
纯正弦就能运行。纯正弦缺少参考脉冲；不要为了放行它而绕过事件/对齐检查。

| 固件选择 | DPG 实际需加载的 TXT |
|---|---|
| `adc -cal loop wave 0` | `waveforms/tone_dither/w06adc_e04_t04_amp5190_tone_m1p5.txt` |
| `adc -cal loop wave 1` | `waveforms/tone_dither/w06adc_e04_t04_amp5190_tone_m1p5_f399p375.txt` |

两者 DAC rate 2.6 GS/s、ADC rate 1.3 GS/s、16640 DAC samples。
主音分别为 199.375 MHz / 399.375 MHz，-1.5 dBFS，约 27570 DAC codes 峰值；
正负平衡 raised-cosine dither 峰值 5190 DAC codes，脉冲宽度 6 ADC samples。
这些是 DAC codes，不是 ADC codes 或实测电压。

实读 TXT 得到：wave 0 范围 -32746..32746；wave 1 范围 -32717..32717，
没有达到数字轨。主音与 dither 的保守峰值和约 32760，已接近 DAC 32767。
不能继续随意增加数字 amplitude。ADC 输入仍可能因模拟路径衰减而很小。
先固定幅度测失配阶梯；幅度对残差的影响另做实验。
改向量参数后必须同步 `loop_waveform.h`，不能只改 TXT/JSON。

## 实验的关键约定

1. 基准：先三项校准，随后全部冻结，用新采样验证，再保存数字系数、
   极性、波形编号、实际执行器码和 A/B 的 IFC/DC 滤波器寄存器值。
2. 每个点：恢复同一个基准，扰动一个通道，执行
   **PRE 冻结测初始值 -> TRAIN 只更新待测项 -> HOLD 全冻结测新数据**。
3. PRE/TRAIN/HOLD 参数都是**尝试帧预算**，包含拒绝帧。
   旧 `loop N` 的 N 是合格帧数量，两者不能混用。
4. Offset：`test offset B 32` 是在解帧后、校正前加 32 个原生 ADC codes。
   不是写芯片 offset DAC，不模拟模拟前端过载。`test dcfilter B on/off`
   才改真实 DC 消除开关，但不能由开关指定任意失配大小。
5. Gain：`test ifc B 0x0D` 只改 B 银行的 `0x1910`，保留其他位并核对读回。
   用实际 PRE 幅度比作横轴；名义 VFS 比例不是高精度标定真值。
6. Skew：只使用现有 ACK 检查的事务。`park` 为绝对逻辑控制码 0..48，
   不是 ps；`step` 测 N,N+1,N,N+1,N。每次上电、频率和工作点需重新确认。
   不假定一个 code 恒等于 7.4 或 19.1 ps，也不固定零点为 12、24 或 35。
7. HOLD 使用新帧，系数不再学习；残差是有符号均值，SD 与 SE 分开报告。
   Offset 残差为校正输出 code 单位；软件注入量为原生 ADC code 单位。
8. 当前 A/B 并行采样，目标 skew=0，不能宣称已经实现真实 2 倍交织。

## 常见误判和恢复规则

- `loop open` 只停止 skew 执行器，gain/offset 仍更新；不要用它替代全冻结。
- `gain_range`、`skew_range` 是软件接收门限，不直接等于算法极限。
- `inside-deadband` 是停止条件；`quantized-zero` 是误差量化为零码；
  `at-range-limit` 才是执行器边界。本次代码已把它们分开。
- 默认 gate：gain 20%，skew 0.25 sample；deadband=10 ps，batch=20。
  300 次尝试最多 15 个完整 skew 批次；远起点失败可能只是预算不足。
- 相位换算的 ps 还包括输入通路相位。两频率比较尽量同次上电；单频测量
  不能独立证明纯采样时延或绝对时基。配对步长若为负，不静默翻转控制方向。
- 不能通过 `rotation+4` 交换通道；物理银行与通道标签先单独确认。
- clock write 会扰动 JESD；不得裸写 delay 寄存器或恢复被冻结的 raw host 写路径。
- 失败 ACK、方向反转会锁定实验；禁止写入重试或自动恢复。
  故障锁保持到固件重新启动。先只读排查，再重启/重载 ELF、bring-up、建基准。
- 中断/超时后也不自动发更多命令；等未完成命令结束或明确重建板状态。
- 正常完成的 `restore` 恢复基准寄存器/时钟码并清软件注入；下一次 `run`
  从保存的数字基准状态初始化，不依赖上一次留下的学习结果。

## 下一步上板顺序

完整测试文档开头已增加“首次上板操作顺序（先跑通三个单点）”，包含
逐条串口命令、基准 IFC/N 的读取、条件示例以及 HOLD 单位换算。
向用户解释或继续操作时优先按该流程，不把示例 N=12 当成固定硬件状态。

1. 在 Vitis clean/build `thesis_v3_loop_appl`，下载新 ELF 并正常 bring-up。
   不编辑生成的 `src/`、链接脚本或平台文件来添加功能。
2. DPG 加载 wave 0 的 TXT，确认采样率/波形，初始化执行器。
3. 先用串口查看 `adc -cal loop test help`，确认新 ELF 确实在运行。
4. `adc -cal loop test baseline 600 100`，随后
   `adc -cal loop test status`，确认 `valid=1` 并记下实际 code=N。
5. 先做一个 offset 点、一个 IFC gain 点，再在 N/N+1 测 skew 步长。
6. 小范围 skew 恢复测试通过后，再扩大正/负起始控制码扫描，每点多次重复。
7. 保存新的输出目录，比较初始失配、HOLD 残差、成功率、收敛尝试数和失败原因。
   所有点通过只能说“至少覆盖该范围”，还不能说找到真正边界。

自动化入口示例（默认只打印计划）：

```powershell
python tools/board_loop/board_axis_test.py sweep --axis skew --uart COM7 --wave 0 --points=1 --repeats 1 --out calibration_out/limits_skew_first
```

实际 COM 端口按机器确认。硬件和波形就绪后加 `--execute`；它会重新建立基准、
扫描并正常恢复。运行 Python 前关闭占用同一串口的终端。
正式大范围/重复扫描的参数及输出字段见完整测试文档。
