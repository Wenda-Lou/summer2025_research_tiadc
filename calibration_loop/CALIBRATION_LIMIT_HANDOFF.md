# AI 交接记录：单项 offset / gain / skew 校准极限

记录日期：2026-10-27。入口已写入仓库根目录 `AGENTS.md`。
详细原理与全部命令：[`CALIBRATION_LIMIT_TEST.md`](CALIBRATION_LIMIT_TEST.md)。

## 用户当前目标

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
- `tools/board_axis_test.py`：配对步长、重复阶梯扫描、日志/CSV/JSON 和 HOLD
  评分。默认 PLAN ONLY，只有 `--execute` 才打开串口。
- `tools/board_loop_log.py`：支持等待新命令的 `LOOP-TEST-END`，ERROR 返回失败。
- 测试：`calibration_sim/sim_loop_tests.c`、`sim_board_axis_tests.c`、
  `tools/test_board_axis_test.py`。

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
python tools/board_axis_test.py sweep --axis skew --uart COM7 --wave 0 --points=1 --repeats 1 --out calibration_out/limits_skew_first
```

实际 COM 端口按机器确认。硬件和波形就绪后加 `--execute`；它会重新建立基准、
扫描并正常恢复。运行 Python 前关闭占用同一串口的终端。
正式大范围/重复扫描的参数及输出字段见完整测试文档。
