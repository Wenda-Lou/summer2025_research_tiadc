# Impulse-Dither 背景校准环路

面向 ZCU102 + AD9164(DPG) + AD9695 平台的 gain / offset / timing-skew 联合校准。
目标期刊：IEEE TCAS-II。

---

## 0. 先说结论：你现在该做什么

按顺序，前三步不需要碰硬件：

1. **跑仿真**，确认算法和你的理解一致（5 分钟）
2. **生成新的 DPG 波形 TXT**，替换掉 `Vector10_DitherFixedPos.txt`
3. **把 ADC 采样率从 500 MHz 改成 614.4 MHz**（见 §3，这一步是硬约束，不改后面全白做）
4. 上板跑闭环，收数据

```bash
cd tiadc_repo
python -m calibration_loop.run_calibration sim --iterations 60
python -m calibration_loop.run_calibration gen --out waveforms
```

---

## 1. 你的平台现在是什么样（我读代码读出来的）

| 环节 | 现状 | 校准环路怎么用它 |
|---|---|---|
| DAC | AD9164 @ 2457.6 MS/s，DPG 循环播放 65536 点 TXT | **dither 注入点**：数字域直接把 dither 加到正弦上，不需要模拟加法器 |
| ADC | AD9695，双通道 A/B，JESD204B L=4 M=2 N=NP=16 K=32 | 被校准对象 |
| 数据流 | JESD → FIFO → AXI DMA(4095 B) → DDR → lwIP UDP → PC:6666 | 每次 capture 给出 8×512 B = 2047 个 16-bit 字 |
| 打包格式 | 每 8 个字 = `[A A A A B B B B]`，14 bit 左对齐（需 `>>2`） | `estimator.unpack_words` / `split_at` |
| 分路器 | 一路反相（`udp_receiver.py` 里那个 `-words[:, 4:]`） | 代码自动检测，不写死 |
| 控制通路 | UART 命令 `dma -d` / `dma -w` / `udp`；UDP 4 字节包改采样时钟延迟 | `capture.py` 直接驱动，**不用改固件** |
| 时钟延迟 | fine 1.725 ps × 0..192，super-fine 0.25 ps × 0..128 | **skew 执行器**，闭环推回硬件 |
| IFC 增益 | 0x1910，7 档 1.36~2.04 Vpp，步进约 5% | 太粗，只能做量程选择；**gain 校准必须在数字域做** |

关键点：**你已经有了一个完整的双向硬件在环平台**，capture 和 actuate 两条路都通。
缺的只是中间那个算法，也就是这次写的东西。

---

## 2. 你现在那个 dither 为什么不对

我看了 `Vector10_DitherFixedPos.txt`：65536 点，每 1024 点一个 **+300 LSB 的单点脉冲**，
没有主信号，全部同号。三个问题，每一个都是致命的：

**(a) 脉冲只有 1 个 DAC 采样宽 = 407 ps。**
AD9695 在 614.4 MS/s 下采样周期 1.63 ns。也就是说 ADC **有很大概率完全采不到这个脉冲**，
采到了也是落在 DAC 重建滤波器和 ADC 前端（约 2 GHz 带宽）平滑之后的未知波形上。
你无法写出它的解析表达式，就无法从它身上提取任何参数。
→ 脉冲必须有 **若干个 ADC 采样周期宽的平顶**。

**(b) 极性全是 +1。**
这是最根本的问题。dither 校准的全部数学基础是：dither 与输入信号**不相关**，
所以用极性序列去相关时，输入信号被平均掉、dither 被保留。
极性恒定 = dither 和信号完全混在一起，永远分不开。
→ 必须是 **±1 随机极性**（Su TCAS-I 2022、Wang TCAS-I 2025 都是这么做的）。

**(c) 没有主信号。**
只有 dither 的话你测的是 DAC 直流特性，不是 ADC 在真实工作条件下的失配。

**(d) 顺带一提**：全正极性还给波形加了 +0.293 LSB 的直流。而你的校准环路要估 offset，
文档里自己写了 "Any intentional DC offset should be avoided"。
新生成器用 **严格平衡** 的极性序列（正负各一半），所以 `Σp[k] = 0` 是恒等式而不是期望值 —— 
这让 offset 估计在**有限样本下**也严格无偏。

---

## 3. ⚠️ 最重要的一件事：ADC 采样率必须改成 614.4 MHz

`fs_dac = 2457.6 MHz`，`fs_adc = 500 MHz` 时比值是

```
2457.6 / 500 = 4.9152 = 3072/625
```

不是整数。后果：一个 DPG 循环 = 65536 DAC 点 = **13333.33 个 ADC 采样**。
循环不落在 ADC 采样边界上，于是**每一个 dither 脉冲落在不同的亚采样相位上**，
平均出来的脉冲复本被抹平 → gain 有偏差，skew 根本估不出来。

改成 **fs_adc = fs_dac / 4 = 614.4 MS/s**：

```
65536 DAC 点 = 16384 ADC 采样   （整数 ✓）
1024 DAC 点   =   256 ADC 采样   （整数 ✓）
Nyquist = 307.2 MHz，测试音 230.36 MHz = 0.75×Nyquist（skew 敏感区）
```

`fs_dac / 2 = 1228.8 MS/s` 也可以（AD9695 上限 1300 MS/s），比值 2。
你的 ADC 时钟本来就来自 DAC 板的参考（报告里写的 J62 / ADF4355），
所以这是**改时钟配置，不是加硬件**。

代码里 `DitherConfig.validate()` 会强制检查这一点，比值非整数会直接报错。

---

## 4. 算法：为什么 impulse 能同时校 gain 和 offset（论文的核心论点）

### 4.1 信号模型

DAC 输出

```
x(t) = s(t) + Σ_k p[k] · A_d · pulse(t − t_k)
```

`s(t)` 主音，`pulse()` 平顶 + 升余弦边沿的脉冲，`p[k] = ±1` 严格平衡随机极性。

通道 i 的 ADC 输出

```
y_i[n] = g_i · x(t_n + Δt_i) + o_i + noise
```

### 4.2 两个正交统计量

把主音拟合掉得到 `r[n]`，然后对所有可见 dither 事件，
分别做**带极性加权**和**不带极性加权**的平均：

```
V[m] = mean_k  p[k] · r[n_k + m]  =  G · d(m + Δt)     ← offset 和主音都被消掉
U[m] = mean_k         r[n_k + m]  =  o                 ← dither 被严格消掉（Σp[k]=0）
```

**这就是"同时"的含义**：同一批数据、同一次采集、同一个 dither，
加极性权重给 gain 和 skew，不加权重给 offset。

### 4.3 从 V 里同时拿到 gain 和 skew

一阶展开 `d(m + Δt) ≈ d(m) + Δt·d'(m)`：

```
Ĝ  = ⟨V, d⟩ / ⟨d, d⟩
Δt̂ = ⟨V − Ĝ·d, d'⟩ / (Ĝ · ⟨d', d'⟩)
```

因为脉冲对称，`⟨d, d'⟩ ≈ 0`，所以这两个估计**几乎正交**，互不污染。

**这才是 impulse 相对 ramp 的真正优势，也是你论文该主打的点：**

| | ramp dither (Su, TCAS-I 2022) | impulse dither (本工作) |
|---|---|---|
| 波形导数 | `d' = 常数` | 平顶 `d'=0`，边沿 `d'` 最大 |
| 幅度信息与时间信息 | **不可分**（幅度误差和时间误差长得一样） | **按脉冲内位置天然分离** |
| 一个 dither 能校 | skew | gain + offset + skew |
| 模拟求和电路 | 需要定制 S/H | 不需要，DAC 数字域相加 |

最后一行是你 slides 里 "Challenges" 那页自己提的开放问题 ——
**用 DPG 在 DAC 数字域相加，就是那个问题的答案，而且这让方法能在商用 ADC 上验证。**

### 4.4 信号干扰对消（对应主参考论文的核心贡献）

Wang et al. TCAS-I 2025 的关键贡献是"复用 sub-ADC 多转换几位，把信号干扰减掉，
收敛从 4.4×10⁶ 加速到 1.0×10⁶ 周期"（Eq. 20–22, Fig. 4/23）。

在你的平台上对应物是：**主音频率是精确已知的（DAC 和 ADC 同源锁定），
所以可以先最小二乘拟合并减掉主音，再去平均 dither 窗口。**
这不改变估计的期望值，只降低方差 —— 和论文里干扰对消的作用完全一致。

代码里是 `LoopOptions.cancel_signal`。**关掉它跑一遍、开着跑一遍，
两条收敛曲线画在一起，就是你论文里对应 Fig. 4 的那张图。**

```bash
python -m calibration_loop.run_calibration sim --iterations 200 --stem with_cancel
python -m calibration_loop.run_calibration sim --iterations 200 --stem no_cancel --no-cancellation
```

---

## 5. 代码结构

```
calibration_loop/
  dither.py      DPG 波形生成 + ADC 速率解析模板（脉冲及其导数）
  estimator.py   解帧 / 对齐 / 联合估计 / block-LMS 状态
  metrics.py     SNDR / SFDR / ENOB、TIADC 失配杂散、A−B 残差
  capture.py     硬件 I/O：UART 触发 + UDP 收帧 + 时钟延迟执行器
  simulate.py    整条链路的仿真模型（含真值，用来验算法）
  loop.py        闭环编排 + CSV 日志 + 学习曲线绘图
  run_calibration.py   命令行入口
```

### 命令

```bash
# 生成 DPG 波形（默认参数已按你的 2457.6 MS/s / 65536 点配好）
python -m calibration_loop.run_calibration gen --out waveforms

# 仿真验证（无需硬件）
python -m calibration_loop.run_calibration sim --iterations 60

# 上板闭环
python -m calibration_loop.run_calibration bench --uart COM3 --iterations 300
```

`gen` 输出两个文件：
- `impulse_dither.txt` — 一行一个有符号整数，无表头，**直接拖进 DPG Downloader**，
  格式和你现在的 Vector10 完全一样
- `impulse_dither.json` — 元数据（极性序列、脉冲几何、种子…）。
  **分析代码读的是同一份配置**，所以 DPG 里播的和代码里假设的严格一致

默认参数（可用命令行覆盖）：

```
fs_dac 2457.6 MS/s   65536 点   adc_ratio 4 → fs_adc 614.4 MS/s
主音 230.3625 MHz (6143 周期/循环，相干)   −6 dBFS
dither 2000 LSB，周期 256 DAC 点 (= 64 ADC 采样)，256 个脉冲/循环
脉冲几何 16/32/16 DAC 点 = 4/8/4 ADC 采样（升余弦边沿 + 平顶）
极性严格平衡，seed 20260725
峰值 18421 / 32767 → 不削顶
```

### 仿真里已经验证的结果

从 gain 失配 2.1%、offset 失配 37 LSB、skew 失配 3.6 ps 出发，40 次迭代后：

| | 校准前 | 校准后 |
|---|---|---|
| gain ratio | 1.0210 | 1.00011（≈110 ppm 残差）|
| offset 失配 | 37 LSB | 0.12 LSB |
| skew 失配 | 3.6 ps | 0.7 ps |
| image 杂散 | −39.7 dBc | −65.9 dBc |
| offset 杂散 | −42.6 dBc | −70.4 dBc |
| SNDR / SFDR | 36.5 / 39.7 dB | 42.2 / 51.0 dB |

---

## 6. 两个模式：先做 parallel，再做真 TIADC

**这里有个硬件限制你需要知道。** 真正的 2× 交织要求 B 通道时钟比 A 晚半个采样周期：

```
fs_adc = 614.4 MS/s  →  Ts/2 = 813.8 ps
AD9695 片上延迟上限 = 192×1.725 + 128×0.25 = 363 ps
```

**片上延迟不够，做不出 Ts/2。** 要真交织必须外部加时钟路径延迟（同轴线约 5 ns/m，
813.8 ps ≈ 16 cm 电缆差；或者用可编程延迟线 / 时钟芯片的相位调节）。

所以分两步：

**第一步（今天就能做）—— parallel 模式。**
两通道同时采同一个信号，测它们之间的 gain / offset / skew 失配并闭环校掉。
片上 0.25 ps 步进的延迟正好用来把残余 skew 推到零。
指标看 `cal_difference_dbc`（A−B 残余载波），三种失配都体现在这一个数上。

```bash
python -m calibration_loop.run_calibration bench --uart COM3 --iterations 300
```

这一步本身就是一篇 TCAS-II 需要的核心实验数据：
*在商用 ADC 上，impulse dither 把双通道失配测到了 ppm / 亚 ps 量级并闭环校正。*

**第二步 —— 加上 Ts/2 后的真交织模式。**

```bash
python -m calibration_loop.run_calibration bench --uart COM3 \
    --interleaved --skew-target-ps 813.802 --iterations 300
```

这时才有 fs/2 和 fs/2−fin 两根失配杂散，那是审稿人最想看的图。

---

## 7. 固件里发现的两个问题

**(1) super-fine 延迟写错了寄存器 —— 会直接影响 skew 执行器**

`test_platform/thesis_v3_500mhz_appl/ad9695_api.c:149`

```c
void ad9695_adc_super_fine_delay(uint8_t super_fine_delay)
{
    ...
    ad9695_write_register(&spi_inst, AD9695_CLK_FINE_DELAY_REG, super_fine_delay);
    //                               ^^^^ 0x0112，应该是 AD9695_CLK_SUPER_FINE_DELAY_REG (0x0111)
}
```

头文件里 `AD9695_CLK_SUPER_FINE_DELAY_REG 0x0111` 是有定义的，只是没被用到。
现在的效果是：super-fine 字段永远没被写，而且 fine 字段被覆盖掉。
改成 `0x0111` 之后，0.25 ps 步进才能用（`bench --super-fine`）。
在改之前，代码默认 `allow_super_fine=False`，只用 1.725 ps 步进。

**(2) lwIP 只在 UART 收到一行之后才被服务**

`main.c` 的主循环是

```c
uart_get_line(uart_line);   // 阻塞
handle_cmd(uart_line);
udp_update();               // xemacif_input() 在这里
```

`uart_get_line()` 阻塞，所以你从 PC 发的 UDP 时钟配置包会**一直躺在队列里**，
直到你在串口再敲一行才被处理。`capture.py` 里已经绕过去了（发完 UDP 补一个空行），
但如果以后要提速，建议把 `udp_update()` 挪到一个非阻塞的轮询里。

**（可选，提速用）** 现在一次 capture 要走 `dma -d` → `dma -w` → `udp` 三条串口命令，
UART 往返是主要瓶颈。在 `butils.c` 的 `cmd_table` 里加一条 `cal` 命令，
内部直接调已有的 `adc_capture_frame()` + `udp_send_mem()`（`adc_ifc_sweep()` 就是这么做的），
可以把单次迭代时间压掉一个量级。不加也能跑。

---

## 8. 建议的实验清单（直接对应论文的图）

| # | 实验 | 命令 / 做法 | 对应论文图 |
|---|---|---|---|
| 1 | 收敛曲线 | `sim`/`bench`，画 `gain_ratio` / `offset` / `skew` vs iteration | 学习曲线（对应 Wang Fig. 23）|
| 2 | 干扰对消加速比 | `--no-cancellation` 对照组 | 对应 Wang Fig. 4(a) |
| 3 | dither 幅度扫描 | `--dither-scale 500/1000/2000/4000`，画收敛周期数 vs A | 收敛周期 vs dither 幅度 |
| 4 | 脉冲几何扫描 | `--dither-edge` / `--dither-top` 扫，看 skew 分辨率 | **只有你有，impulse 特有** |
| 5 | 输入频率扫描 | 改 `--sig-cycles`，测校准前后 SFDR/SNDR | 动态性能 vs fin |
| 6 | 温度 / 时间漂移 | 长时间跑，看环路跟踪能力 | 背景校准的核心卖点 |
| 7 | 与 ramp dither 对比 | 同平台换成 ramp 波形，只能校 skew | 方法对比表（对应 Wang Table I）|

实验 4 和 7 是**别人做不了、只有 impulse 才有**的数据，
TCAS-II 的贡献点应该压在这两个上。

---

## 9. 论文定位建议（诚实版）

写 TCAS-II 的时候，claim 要精确，审稿人会挑：

**可以理直气壮说的：**
- impulse dither 用**同一个波形、同一批数据**同时提取 gain、offset、skew，
  三者按脉冲内位置正交分离 —— ramp dither 做不到
- dither 在 **DAC 数字域**注入，不需要定制模拟求和 S/H，
  因此**首次在商用 ADC 上完成了 dither 背景校准的硬件验证**
- 严格平衡极性序列使 offset 估计在有限样本下无偏
- 主音对消把收敛周期数降低了 N 倍（用实验 2 的数据填 N）

**不要过度宣称的：**
- 不要说"offset 只有 impulse 才能校"。相干正弦下，记录均值本身就能给 offset。
  诚实的说法是：**gain 和 skew 由 impulse 独有地同时给出，offset 是同一个统计量的免费副产品。**
- 这是**数据辅助**（dither 是自己产生的、已知的），但对**输入信号是盲的** ——
  后者才是背景校准的定义，说清楚就没问题
- 校准运行在 host 上，不是片上数字电路。写成
  "算法用定点友好的 block-LMS 表述，无矩阵求逆，可直接移植到 PL/PS" 比较稳妥

---

## 10. 依赖

```bash
pip install numpy matplotlib pyserial
```

`pyserial` 只有 `bench` 模式需要；`gen` 和 `sim` 只要 numpy（画图要 matplotlib）。
