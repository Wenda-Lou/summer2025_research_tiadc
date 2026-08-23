# TI ADC 数字注入 Dither 校准——研究进展

**核心要点：** 原始 impulse-dither 估计器受实测 DAC→ADC 色散限制；码序列——尤其是配合专用估计器的 Golay——在当前工作点下的仿真中展现出有希望的 fine-skew 表现。

## 1. 板级发现

- tone 主校准路径稳定且准确：skew 收敛到 **<1 ps**，Stage 5 平行平均 **39.5 dB / 6.27 bits**。
- dither 事件检测可靠（10/10），但：
  - offset 分离较弱（3–4 codes，跨 run 不稳定）；
  - gain 没有恒定校准常数（0.24–0.44）；
  - 当前 fine-skew 估计器在板级**并不可靠**（edge 分歧 100–1000 ps vs 23.1 ps）。
- 关键限制是链路色散：注入 32–48 样本脉冲到达 ADC 时展宽为 **74–123 样本**；在当前 DAC→ADC 信号链中，足够干净的平顶无法被保留。

## 2. 仿真发现

测试了四种 dither 结构（无平顶三角、平顶线性、PRBS、Golay），分别在 ideal 和模拟板（色散 + 噪声）模式下。

- 使用现有生产估计器时，脉冲类 dither 事件检测 100% 有效，但模拟板色散下 fine-skew 掉到 **约 1%**；PRBS/Golay 不被识别。
- 单独实现了相关/最小二乘估计器：
  - 当前工作点下，Golay 的 offset 误差约 **0.3 codes**、gain 误差约 **2%**；PRBS 的 B 通道 offset 仍有偏置（约 6–7 codes）；
  - 多帧平均下 fine-skew：**Golay 平均误差约 0.35 ps**（20 帧），PRBS 约 2.1 ps；
  - 在更高采样率/频率（如 2.7 GSPS / 1200 MHz）下，tone 拟合仍然准确，但码序列估计器会退化（Golay skew 约 4 ps，gain 约 3.6%）。

## 3. 结论

- 在当前 DAC→ADC 信号链中，足够干净的平顶无法被保留；仅去掉平顶并不能解决 fine-skew，因为核心瓶颈是**色散/带宽**。
- 在目前测试的结构中，**码序列是 dither fine-skew 估计最有希望的方向**；**Golay 目前整体仿真表现最好**。

## 4. 当前想法

Golay 目前看起来更有希望：在当前工作点下，多帧平均 skew 误差约 **0.35 ps**，而 PRBS 约 **2.1 ps**。
