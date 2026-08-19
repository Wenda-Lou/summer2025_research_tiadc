# Dither Fine-Skew 调查最终结论（2026-08-19）

## 1. 最终状态（板级，19:03 run 为最新，Stage 5 修复已验证）

| 项 | 状态 | 数值 |
|---|---|---|
| tone 主路径 | **PASS** | skew −0.87 ps、2/2、reg 35、Stage 5 38.8 dB / 6.13 bits（平行 39.5 dB / 6.27）|
| **Stage 5 汇总（极性自校正修复）** | **恢复** | Mean RMSE **23.15**（异常 759 → 修复）、Mean correlation **+0.998354**（−0.998 → 修复）|
| dither 事件检测 | VALID 10/10 | 全 run 稳定 |
| dither fine-skew | PARTIAL（1/10 帧 VALID）| 首次出现 per-frame VALID（calibration iter4 cap7：dither 1.79 ps vs tone −0.85 ps，edge 14.9 ps）；9/10 仍 edge 分歧 |
| **offset 分离** | **估计器 PASS / 控制器收敛（精度受限）** | 平顶 31 样本；Dither offset 与 fitted-tone DC 偏差跨 run 不稳定（3–4 codes），PASS 为容差内弱通过 |
| gain 分离 | 受限（定案）| **flat ≈ dither（0.249 vs 0.235）**——平顶不例外，整体模板域↔ADC 码域幅度映射 + FIT_QUALITY，无校准常数 |
| skew 分离 | **定案 INVALID**（advisory）| per-frame edge 分歧 median ~230 ps、joint 全 0、14 种方法否决 |

## 2. 累计否决的方法（14 种，全部有数据）

波形几何（H5：RC 16/32、linear 48/0、linear 48/32、p260 双倍周期）、
窗口/门控参数扫描（25 组合）、per-edge gain、自校准定位、detrend（及
去 DC）、两参数延迟+宽度模型、间隙背景估计、中位数聚合、事件质量门控、
事件中心精化、gain 比例校准。flat gain 已实测：**`flat` ≈ `dither`
（0.249 vs 0.235）**，平顶不例外 → 整体幅度映射限制，无校准常数。

## 3. 机制证据链（论文限制节素材）

1. **链路色散**：注入 32–48 样本脉冲 → ADC 域 10–90% 宽 74–123 样本；
2. **窗口/周期约束**：130 样本事件周期下窗口 ±64 占满 129 样本，无间隙
   测背景、事件中心跳变（35 样本）无容错；双倍周期（260）虽解除约束但
   更宽脉冲（112 样本）污染 tone 拟合 → 主路径退化（baseline std
   2–9 → 23 ps）——**dither 几何空间与 tone 主估计存在根本权衡**；
3. **窗口 DC/慢变偏置**：A/B 窗口均值恒 +1.8/−1.6 codes（与极性相关），
   上升/下降掩码不对称 → 边沿相反偏置（100–240 ps 分歧）；detrend 可
   消部分但窗口近满周期时线性拟合吸收脉冲肩部（部分帧恶化）；
4. **事件中心跳变**：帧内相位 44→9（spacing 95/131 伪值），检测峰不稳；
5. **模板/定位敏感性**：同帧换模板分歧 0.72 ↔ 243 ps（17 倍）；
6. **tone 拟合污染**：dither 脉冲占窗口比例（64/800 基线、112/800 p260）
   直接影响批次噪声（baseline std 2–23 ps）。

## 4. 论文叙事建议

- **主张**："用 digitally generated impulse dither 分离 gain/offset/
  timing skew"→ 改为"三分离架构完整实现 + **offset 分离板级验证** +
  gain/skew 限制分析"；
- **正面结果**：数字生成 impulse dither 链路（波形、注入、事件检测
  10/10）✓；offset 分离估计器运行且 PASS、offset 控制器收敛（CONVERGED、
  verify PASS）✓；tone 主校准系统（收敛 <1 ps、Stage 5 6.2 bits）✓；
- **限制节**：上表 6 条机制 + 14 种方法否决的证据；
- **谨慎表述**：offset 的 PASS 是容差内弱通过（Dither offset 与
  fitted-tone DC 偏差 3–4 codes 且跨 run 不稳定，batch 值接近校正后
  残余）——**不写"dither 独立精确估计 offset"**，写"offset 分离估计器
  集成运行、与 tone 主估计共同收敛"。

## 5. 遗留项（全部关闭，不阻塞论文）

1. `dither_flat_gain`：**已实测关闭**（flat ≈ dither 0.249/0.235，UART 直接获得）；
2. **Stage 5 极性自校正：已修复并板级验证**（19:03 run：Mean RMSE 23.15、Mean correlation +0.998354；sim 用例 + `--run-all` 全绿）；
3. **差分路线：已关闭**（需调试抓帧，非必要，skew 定案不依赖）；
4. 抓帧工具链：`capture_frames.py`（burst 默认）保留备用。

## 6. 结论

**dither fine-skew 交叉校验在现链路上无可行修复（14 种方法否决，
板级与离线一致），正式定案：dither 保持 advisory、fine-skew 如实报
INVALID、机制证据链写入报告限制节。tone 主估计器与 offset 分离分支
是论文的正面结果。**
