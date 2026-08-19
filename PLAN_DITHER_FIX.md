# 修复 Dither Fine-Skew INVALID 的计划

- 日期：2026-08-16
- 状态：待执行（Phase 0 需要台架配合抓原始帧）
- 关联：Stage-4 skew cross-check、Stage-3 gain dither warning
- 约束：不动 Stage-4 tone 主路径；dither 保持 advisory，永不 gate 任何 stage

---

## 6. 执行记录（2026-08-19 追加）

### 6.1 新波形上板结果（H5 否决）

`calibration_run_20260819_131800`（199.375 MHz + linear 三角 edge 48/top 0，
skew 帧拟合 tone = 199.3750176 MHz，确认新波形已上传）：

- Dither fine-skew / cross-check：**仍全 INVALID**（10/10 帧 `dither_skew_valid=0`）；
- UART 汇总打印 `Dither rejection reason : POLARITY`，但 export CSV 显示这是
  **最后一帧的误导性原因**：120 行 capture 中 119 行 `dither_A_valid=1,
  dither_B_valid=1, dither_skew_valid=0`（估计器跑通但交叉校验不通过，老
  边沿分歧家族），只有最后 1 行（calibration 第 10 帧）是 0,0,0（POLARITY）；
- tone 主路径不受影响：收敛 −4.99 ps、reg 33、2/2 PASS、表征 7.07 ps/code；
- gain 阶段 dither 依旧 `FIT_QUALITY` WARNING、dither_gain ≈ 0.40。

**结论：H5（波形几何）正式否决**。波形文件极性序列已验证平衡
（32+/32−，两个几何都是），排除生成器问题。

### 6.2 POLARITY 机制复核（修正 2026-08-16 的初判）

极化公式 `polarized = capture_sign * template_sign * aligned` 中
`template_sign` 因子**不是冗余**：它抵消模板自身的交替，使精确匹配的捕获
得到交替极性模板（mean_polarity ≈ 0），全局反相得到交替反相模板；只有捕获
事件全部同号时才产生单极性模板 → 平衡检查（`1 - mean_polarity^2 < 0.05`）
正确拒绝（POLARITY）。因此：

- POLARITY 是估计器对"该帧捕获事件同号"的正确拒绝，不是代码缺陷；
- 曾考虑的"去掉 template_sign"一行修复是**错误方向**（会让所有干净帧
  报 POLARITY），已用 sim 契约测试锁定正确行为；
- 该 POLARITY 帧的根因（对齐 lag 偏置 / 帧内 dither 真实同号 / tone 残差
  偏置）需要原始 DMA 帧或 per-frame 诊断才能区分——08-19 未抓原始帧。

### 6.3 本轮代码改动（2026-08-19，主机侧完成，待板级验证）

1. **共享极化函数**：`adc_calibration_dither.c/h` 新增
   `adc_cal_dither_polarize_template()`（BSP-free，行为与板级内联块完全
   一致），`butils_calibration.c` 改为调用；sim 新增
   `unit_dither_polarize_contract`：精确匹配→交替、全局反相→交替反相、
   同号捕获→POLARITY 正确拒绝、端到端精确匹配→PASS。
2. **per-frame dither 诊断列**：`calibration_skew_captures.csv` 追加
   `dither_reason, dither_rising_skew_ps, dither_falling_skew_ps,
   dither_edge_disagreement_ps`——下一轮台架可直接看到每帧的真实拒绝原因
   与逐边沿分歧，不再被汇总行的"最后一帧原因"误导。
3. **全 pipeline SNDR/ENOB 追踪**：
   - timing/offset/gain captures 追加 `sndr_db, enob_bits`（对校正后的
     分析窗口做全频谱 `adc_cal_perf_analyze_record`）；
   - skew captures 追加 `tone_A/B_sndr_db, tone_A/B_enob_bits`（tone fit
     幅度/rmse 换算，共享助手 `adc_cal_perf_sndr_enob_from_tone_fit`）；
   - offset/gain/skew iterations CSV 追加对应均值列；
   - sim 新增 `unit_perf_sndr_fit_helper`。
4. 恢复 `--run-all` 硬依赖 fixture：`adc_capture_20260801_180451.csv` /
   `adc_capture_20260805_100121.csv` 曾被 `aae7959` 删除，已从历史
   （`8ee609e` / `fc6b4fb`）恢复，unit 5695/0 全绿。

### 6.4 剩余行动

- 下次台架：linear 48/0 或 RC 16/32 重跑 `adc -cal`，**同时抓原始 DMA 帧**
  （4096 字节 CSV），用 `dither_replay.py` 对比换几何前后脉宽与逐边沿分歧；
  新 export 的 per-frame 诊断列可现场确认 POLARITY 帧与边沿分歧分布；
- 若分歧仍 >23.1 ps：实施 Phase 1 剩余项（per-edge gain 优先）；
- 若确认是单帧 POLARITY 偶发：对照 6.2 的三个候选根因，倾向对齐 lag 检查。

---

## 7. 机制定案与主机侧修复（2026-08-19 追加）

### 7.1 离线诊断结论（3 帧原始捕获 144256/144314/144328，199.375 linear 48/0）

| 帧 | detrend 后边沿分歧 | dither rel | tone skew | \|dither−tone\| |
|---|---|---|---|---|
| 144256 | 7.2 ps ✓ | +29.6 ps | +9.7 ps | 19.9 ps |
| 144314 | 47.0 ps | −77.7 ps | +4.3 ps | 82.1 ps |
| 144328 | 110.4 ps（RC 模板 7.7 ✓）| +213.7 ps | +12.1 ps | 201.6 ps |

（replay 的模板几何/相位对结果影响巨大：同一帧换模板分歧 13.7↔242.8 ps；
固件模板为逐帧极化的 capture_polarized_template → 帧间估计跳变。）

1. **模板/定位敏感性（已确认）**：事件定位受"模板窄、真实脉冲宽 2 倍、
   A/B 宽度不等"影响，B 的模板相关峰恒定偏置 +7 样本（形状伪影，非抖动）；
   定位质量帧间不同（spacing std 1.7~67 样本）且与分歧相关。
2. **窗口内 DC/慢变偏置（已确认，已修复）**：A 窗口均值恒 +1.8 codes、
   B 恒 −1.6 codes（与极性相关的 tone 残差偏置）；上升/下降掩码不对称 →
   两条边相反偏置 → 100–240 ps 分歧。合成验证（0.9375 cycles/event 正弦
   偏置）：legacy 分歧 0.168 样本 → per-event 去趋势后 0.030 样本。
3. **per-edge gain：数据否决（2026-08-19）**：g_rise/g_fall 与 g_all 仅差
   1–3%（B/A 幅度一致，边沿差异是宽度差不是幅度差），144314 分歧
   47.0→42.8 ps 几乎不变。Phase 1 候选 #1 不实施。
4. **机制 B（未解决，需更多数据）**：tone skew 帧间稳定（+4~+12 ps），
   但 dither full-profile 估计帧间漂移 −78~+214 ps；即使边沿分歧进门限的
   帧（144256），dither 与 tone 仍差 ~20 ps。3 帧样本不足以定案根因
   （定位/残差曲率/形状逐帧变化）。

### 7.2 主机侧修复（已完成，待台架验证）

1. **per-event 窗口去趋势**（H2 残差净化）：
   - `adc_cal_skew_config_t.profile_window_detrend`（0=旧行为，1=去趋势），
     `adc_calibration_skew.c` 聚合前对每个事件窗口做线性去趋势（DC+斜率），
     `butils.c` 板级宏 `CAL_SKEW_DITHER_PROFILE_WINDOW_DETREND=1` 启用；
   - 注意：去趋势仅配合加宽窗口（±64）使用——模板尺寸窗口内脉冲本身是
     斜坡，去趋势会把信号去掉（实测 ~0.25× 衰减，sim 契约测试已注明）；
   - sim 新增契约测试：0.9375 cycles/event 正弦偏置合成帧，legacy 分歧
     >0.03 → detrend 后 <0.03 且 skew ≈ 真值；干净帧不回归；
   - `--run-all`：5712 unit / 18 scenario / 14 pipeline 全绿。
2. **replay 正式功能**：`dither_replay.py` 新增 `--detrend-windows`、
   `--template-shape/edge/top`、`--diagnose`（事件网格/质心/间距/A-B delta）、
   `--selfcalibrate`（两阶段定位，实验结论：定位非主因）、profile-level
   B-A 延迟。复现：detrend 后 144256=12.95 ps、144328=7.70 ps 进门限，
   144314=54.6 ps（形状差残留，见 7.1.3/7.1.4）。

### 7.3 台架验证项（下次上板）

1. 烧录新固件（含 detrend=1 板级路径）重跑 `adc -cal`，看 per-frame
   诊断列：dither_reason 分布与边沿分歧是否显著下降（预期 EDGE_DISAGREEMENT
   帧减少、分歧从 100–240 ps 降到 <30 ps 量级）；
2. 抓帧时**同时记录寄存器状态与 tone skew**（机制 B 归因需要）；
3. 若机制 B（dither 值帧间漂移）持续：考虑计划验收 #5 的模型路线——
   报告边沿不对称度诊断字段、VALID 判定依据文档化，或接受 advisory
   INVALID 定案（不放松门限）。

### 7.4 交叉帧联合聚合（2026-08-19 追加，已验证有效）

**发现**：单帧分歧/漂移的本质是"每帧窗口内偏置与噪声的随机实现"——
跨帧极性序列不同，窗口偏置在聚合时随机化抵消。6 帧 42 事件离线联合：
边沿分歧 70–350 ps → **4.7 ps**；dither rel 帧间漂移 −145~+209 ps →
联合 **+21.7 ps**；|dither − tone| = **12.4 ps < 23.1 ps 门限 → VALID**。

**detrend 板级开关：已关闭（2026-08-19 15:55 run 后定案）**。
detrend=1 固件跑 `adc -cal`（`calibration_run_20260819_155534`）：skew 6
个 iteration 仍全部 `dither_valid_frames=0 / inv=10`——per-frame 交叉
校验未改善；且离线 replay 已证明 detrend 在部分帧恶化（151221 47→90、
151252 44→133 ps；窗口 129 样本接近事件周期 130，线性拟合吸收脉冲肩
部）。`CAL_SKEW_DITHER_PROFILE_WINDOW_DETREND=0`。sim 的 detrend 契约
测试保留（验证配置行为本身），joint 估计 config 固定 detrend=0 不受影响。
该轮其余正常：tone 收敛 +0.25 ps / reg 35 / CONVERGED；SNDR/ENOB 追踪
工作正常（timing 36.6–37.7 dB、skew fit 32.8–33.1 dB）；run 在 Stage 5
前中断（缺 skew_captures/performance 导出）。

**实施（主机侧完成，待台架验证）**：
1. 共享模块（`adc_calibration_skew.c/h`）：
   - 提取 `finish_profile_estimate()`（单帧/joint 共用的模板派生、gain
     投影、四路边沿估计、状态分类）；
   - 新增 `adc_cal_skew_estimate_joint_frames()`：每帧独立事件检测 →
     所有事件窗口极性加权堆叠（固定 ±N 窗口，要求
     `profile_window_half_samples > 0`）→ finish；事件总数作为
     `accepted_events` 输出；
2. board（`butils_calibration.c`）：`calibration_estimate_skew_frame`
   新增可选输出（残差 ×2 + aligned 模板）；batch 帧循环收集 accepted 帧
   残差（10 × 800 doubles ×2 静态缓冲）；controller 迭代记录前调用
   `calibration_run_skew_joint_estimate()`（window 64 / gate 0 /
   detrend 0，与离线验证参数一致）；UART 新增
   `Dither joint (N-frame)` 行；`calibration_skew_iterations.csv` 新增
   `dither_joint_valid / skew_ps / edge_disagreement_ps /
   tone_disagreement_ps` 四列；
3. sim：`unit_skew_joint_frames`——4 帧不同相位正弦偏置（0.9375
   cycles/event）：单帧分歧 >0.03（对照）→ joint <0.03 且 skew≈0.10
   真值、PASS；干净帧不回归；参数守卫；
4. `--run-all`：**5728 unit / 18 scenario / 14 pipeline 全绿**；
5. 注意：joint 路径独立于 detrend（raw 窗口聚合，与离线验证一致）；
   若台架显示 detrend=1 在 per-frame 路径有害，可单独将
   `CAL_SKEW_DITHER_PROFILE_WINDOW_DETREND` 改回 0，不影响 joint。

**台架验证**：烧录后跑 `adc -cal`，看 UART `Dither joint (N-frame)` 行
与 CSV 新列：预期 VALID（分歧 <23.1 ps 且 |joint − tone| <23.1 ps），
每 iteration 一行可追踪。

### 7.5 双倍周期几何（方法 4，2026-08-19 实施，待台架验证）

**动机**：所有离线失败都撞在"130 样本事件周期"的约束上——窗口 ±64
（129 样本）几乎占满周期：无间隙测背景、事件中心跳变（35 样本）无容错、
斜坡长度受限。双倍周期（260 ADC 样本间距）打破该约束：窗口 ±100
（201 样本，间隙 59 样本）、脉冲 112 样本完整容纳、斜坡 48 ADC 样本
（比 linear 48/0 长一倍）、真平顶 16 ADC 样本（offset 分支条件）。

**改动**：
1. `generate_dac_waveform.py`：周期推导/样本数参数化
   （`derive_dither_periods`/`derive_sample_counts`），新增
   `--dither-event-period-ns`（默认 100，200 = 双倍）；命名加 `_p260`
   后缀区分；生成
   `sine_199p375MHz_2p6GSPS_impulse_dither_p260.txt`（linear 96/32，
   check 6/6：260 ADC 样本间距、脉冲 112、coherence 0.067、极性 32+/32−）；
2. `adc_calibration_skew.h`：`ADC_CAL_SKEW_MAX_PROFILE_SAMPLES` 192→256；
3. `butils.c`：`CAL_SKEW_DITHER_PROFILE_WINDOW_HALF` 64→100
   （**注意：窗口宏必须与 DPG 上实际加载的波形匹配**——旧 130 周期波形
   需改回 64 重新编译）；
4. `dither_replay.py`：`DITHER_PERIOD_ADC` 260、`PROFILE_HALF` 100、
   `--period-adc` 参数（旧帧分析传 130）；
5. sim：`--run-all` 全绿（5728+ unit / 18 / 14）。

**台架流程**：加载 `..._p260.txt` → `adc -ref` → `adc -cal`（joint 固件 +
窗口 100 编译）→ 跑完 `capture_frames.py --frames 20`。预期一次验证：
joint 稳定性、offset 分支（真平顶）、gain 分支（长斜坡+平顶双参考）。

### 7.6 方法 4 台架结果（2026-08-19 17:14 run）——否决，回到 130 周期基线

`calibration_run_20260819_171440`（p260 波形 + 窗口 100 + joint 固件）：
- per-frame：dither_reason = edge estimates disagree 77 / **POLARITY 23** /
  outside linear 14 / 其他 6；edge 分歧 median 260 ps（min 1.4 / max 1043，
  与旧几何 148–305 ps 同量级——**无改善**）；
- joint（7 iteration）：估计器产出数值但 edge 113–397 ps 全超门限；
  CSV 曾显示 `dither_joint_valid=1` 是**语义 bug**（`result.valid` 在
  WARNING 时也是 1）——已修为 `status==PASS` 且 |joint−tone| ≤ 0.03 才算
  VALID；
- **tone 主路径退化（新问题）**：initial baseline std 23.3 ps（以往
  1.8–9.7）、MARGINAL、correction NOT CONVERGED（0/2）、actuator
  分辨率 25.9 ps/code（以往 ~7）——原因：脉冲 112 样本占 tone 拟合窗口
  比例更大（112/800 vs 64/800）→ dither 污染 tone 拟合 → 批次噪声放大；
- Stage 5 异常（Mean RMSE 738 codes、correlation −0.9985）待查，疑似
  通道极性/汇总字段问题，与 dither 方案无关。

**结论：双倍周期方向否决**——扩展 dither 几何空间以伤害 tone 主估计器
为代价，得不偿失。**已执行**：`CAL_SKEW_DITHER_PROFILE_WINDOW_HALF` 回 64、
`ADC_CAL_SKEW_MAX_PROFILE_SAMPLES` 回 192、replay 默认回 130/64，sim
`--run-all` 全绿；波形回到 130 周期基线（linear 48/0 或 RC 16/32，tone
主路径健康）。生成脚本 `--dither-event-period-ns` 与 `_p260` 波形保留为
工具资产。

**累计否决清单（2026-08-19）**：波形几何 H5、窗口/门控扫描、per-edge
gain、自校准定位、detrend、中心精化、两参数模型、间隙背景、中位数
聚合、事件质量门控、双倍周期、gain 比例校准。**dither fine-skew 交叉校验
在现有链路上无可行修复**——正式走计划验收 #5 定案路线：dither 保持
advisory、fine-skew 如实报 INVALID、边沿不对称度与证据链写入报告限制节。

### 7.7 板级结论：offset 分离分支 PASS（2026-08-19 17:55 run，linear 48/32）

Stage 2 "Dither-Aware Offset Diagnostic"（linear 48/32 平顶生效）：
- Flat-top samples 31、Dither offset −0.39 codes、Tone fit RMSE 9.6、
  **New estimator status: PASS**、汇总 "Dither estimator status: PASS"；
- 每 batch 运行（events 6），offset 控制器照常收敛（−10.58 codes，
  CONVERGED），dither offset 独立估计残余（−0.53~+0.58 codes 跳动）。

**论文三分离最终状态**：
- **offset：板级 PASS ✓**（平顶采样，linear 48/32 首次提供真平顶；
  RC 波形离线亦可行 −0.4~−2.1 codes）；
- gain：数值错误（dither_gain 0.23–0.36 vs 真值 1.0，无恒定比例、
  与 FIT_QUALITY 相关，校准路线否决）。2026-08-19 18:24 run CSV 细化
  归因：reason=NONE 帧 dither_gain 高度集中（0.315–0.446，mean 0.360，
  n=34），FIT_QUALITY 帧 0.034–0.097（n=26）——**双峰分布**：模板匹配
  好时是"恒定 ~0.36× 低估"（模板域 vs ADC 码域的链路幅度映射；48/0
  波形时为 0.39–0.44，跨波形 ±10% 变化 → 无精确校准常数），匹配差时
  严重低估（0.06×）。dither_flat_gain（平顶增益）未导出，是最后一个
  未确认点（若 flat 区不受色散影响可能更接近 1.0），可给 gain_captures
  CSV 加列验证；
- skew：定案 INVALID（advisory，13 种方法否决）。

论文叙事："用 digitally generated impulse dither 分离三种 mismatch" →
三分离架构完整实现，**offset 分离板级验证成功**，gain/skew 受链路色散/
窗口偏置限制（限制节：色散 74–123 样本、窗口/周期约束、窗口 DC 偏置、
事件中心跳变、tone 污染权衡），skew 由 tone 主估计 + dither 事件级
验证（10/10 VALID，Stage 1 依赖）承担。

### 7.8 Stage 5 极性自校正（2026-08-19，共享模块修复）

**异常**：joint 固件版本起 UART 汇总 "Mean RMSE 759 / Mean correlation
−0.998"（基线 19 / +0.999）；定位为 canonical 通道（Channel A）的
极性标定与实际信号相反（polarity[A]=−1 而 cal_a 与 reference 同相）→
归一化后 canonical 相关反相、reference RMSE 放大 ~40×（CSV A_correlation
+0.998 与 UART Mean −0.998 是同一对数据的正负号）。

**修复**（`adc_calibration_performance.c` `analyze_frame`）：canonical
通道归一化 cal 与 reference 的相关为负时，**全局翻转两通道极性**（保持
A/B 相对关系；光谱 SNDR/ENOB 与 matching 为符号不变项）。sim 新增
`unit_performance_estimator_direct` 用例：错误标定 (−1,+1) → 自校正后
correlation>0.9、rmse<5；正确标定 (1,−1) 不误翻；两次 sndr 相等。
`--run-all` 全绿。板级验证：下次烧录后 Stage 5 汇总应显示 Mean RMSE
~19、Mean correlation ~+0.999。

---

## 1. 目标与验收标准

1. 板级 `Dither fine-skew estimate: VALID`、`Dither cross-check: VALID`
2. `|rising − falling| ≤ 0.03 samples`（= 23.1 ps @ 1.3 GSPS，现有门限
   `ADC_CAL_SKEW_MAX_EDGE_DISAGREEMENT_SAMPLES`，**不放松**）
3. `|dither − tone|` 在可接受范围内（建议 ≤ 2× 组合标准差，约 < 10 ps）
4. Stage-4 tone 收敛行为与修复前一致（对照组：最终 skew +1.12 ps、
   寄存器 35、2/2 连续 PASS）
5. 若最终判定为板级物理脉冲形状问题（H1），则改为：估计器正确报告
   边沿不对称度（新 diagnostic 字段）并把 VALID/INVALID 判定依据文档化——
   不允许用放宽门限的方式"通过"

非目标：不调低边沿门限、不让 Stage-4 依赖 dither、不重调 tone 控制器、
不修 `ad9695_adc_super_fine_delay()` 的 0x0112/0x0111 已知问题（与本问题无关）。

---

## 2. 现状与证据

### 2.1 板级事实（16:43:32 跑，新固件，修复后）

| 项 | 值 |
|---|---|
| Dither event detection | VALID，10/10 events（16:22 跑同） |
| Dither rising-edge | −91.71 ps |
| Dither falling-edge | +75.69 ps |
| Edge disagreement | 167.40 ps = 0.2176 samples（门限 0.03 → 超 7.3×） |
| Full-profile dither skew | +19.32 ps |
| Tone skew | +1.59 ps（收敛，std 0.21 ps） |
| Tone/dither disagreement | 17.83 ps |
| Gain 阶段 dither | WARNING `FIT_QUALITY`，dither_gain ≈ 0.44，events 6 |

16:43:32 跑的 skew 迭代 CSV 中 dither 全为 nan / `dither_skew_valid=0`。

### 2.2 代码事实

- `adc_cal_skew_estimate_from_residuals()`（`adc_calibration_skew.c`）：
  - 事件定位用 `adc_cal_dither_analyze()` + DAC 文本派生的 `dither_template`
  - 聚合 A/B/模板 profile 后，**用 A 的采集 profile 作为形状模板**，B 经
    投影增益（可负 → 已处理反相）拟合到 A 模板
  - 导数投影分别估计 all / rising（+1）/ falling（−1）三组 skew
  - `rising = rise_B − rise_A`；`falling = fall_B − fall_A`；
    `disagreement = |rising − falling|`；超门限 → `EDGE_DISAGREEMENT`
    warning → 上层将 dither 判 INVALID（"edge estimates disagree"）
- 模板来源：`calibration_fit_tone_refined()` 从上传参考波形拟合
  `reference_tone + dither_template`，再按 A 残差对齐/极化
- 门限：`ADC_CAL_SKEW_MAX_EDGE_DISAGREEMENT_SAMPLES = 0.03`
- 主机确定性测试（刚体对称脉冲，含反相 B、正负 skew）全过 → 纯符号/
  插值 bug 概率低；板级 167 ps 分歧指向形状/残差问题

### 2.3 数值解读

rising/falling 符号相反且幅度接近，等价于：B 的上升沿比 A 晚 ~92 ps、
B 的下降沿比 A 早 ~76 ps —— 在"刚体平移"假设下 B 脉冲比 A **窄 ~168 ps**
（≈ 0.22 samples）。刚体模型无法吸收边宽差，所以 full-profile 拟合取了
中间值 19.3 ps，两条边各剩 ±76~92 ps 残差。tone 说 1.6 ps。

### 2.4 新发现：tone 与 dither 事件网格同步（2026-08-16 `check` 实测）

`python -m calibration_loop.run_calibration check --waveform-json
sine_200MHz_2p6GSPS_impulse_dither_001.txt.json --adc-rate 1.3e9`
6 项检查中第 5 项 FAIL：

- 200.0 MHz tone × 130 样本/事件 ÷ 1.3 GHz = **恰好 20 整周期/事件**，
  coherence 1.000（要求 < 0.3）
- 板上 199.875 MHz 拟合下是 19.9875 周期/事件——窗口内 6 个事件相位
  几乎不扩散，结构性问题仍在
- 后果：tone 残留（拟合不完美部分）在事件聚合后保持同一相位，非对称
  偏置两条边沿；这正是 `check.py` 文档里 2026-08-10 事故的同款机制
- `check` 建议的合规 tone：**199.375 MHz（bin 1276，0.9375 cycles/event，
  coherence 0.067）** 或 200.625 MHz（bin 1284）

行动：在 Phase 0 之前先做一个低成本实验——用现有生成脚本生成
199.375 MHz 向量（dither 参数完全不变），重新 `adc -ref` 上传，
重跑 `adc -cal`。若 dither 转 VALID 则根因即为此；若仍 INVALID，
至少排除了最高概率的可控因素。

---

## 3. 根因假设（按先验概率排序）

- **H1 物理脉冲形状非刚体（最可能）**：A/B 两路模拟前端/注入路径对
  dither 脉冲的色散不同，B 脉冲边宽与 A 差 ~168 ps 且跨帧稳定。
  此时 167 ps 是真实信号，需要升级模型或把 VALID 判定改为基于模型诊断。
- **H2 残差污染**：tone 拟合残留（当前 tone 199.875 MHz；A/B 窗口间
  tone 相位/幅度漂移）在两条边上产生方向相反的偏差。
- **H3 模板/对齐伪影**：DAC 文本模板与板上真实脉冲形状不一致；事件对齐
  为整采样 + anchor 延迟，亚采样误差在聚合 profile 两条边留下相反斜率偏差。
- **H4 代码缺陷（符号/掩码/极性）**：现有刚体测试测不到，需板形数据回放。
- **H5 脉冲几何（VB 项目经验，2026-08-16 引入）**：另一份基于本仓库的
  模拟器项目用同一生产估计链定位并解决了 dither INVALID——阶跃/短边缘
  脉冲经采样插值后不携带亚采样位移；把 dither 换成线性斜坡三角脉冲
  （ramp32/w64）后 rising/falling/tone 三路收敛到真值 0.3 ps 以内。
  经验曲线：ramp4 → 254 ps（2.1× 偏置）、ramp16 → 125 ps、ramp32 → 119.4 ps
  ✓——斜坡越长，估计越准且两边沿越一致。
- **H6 profile 窗口截断（本仓库实测，2026-08-16）**：离线回放 5 帧
  199.375 捕获（`calibration_loop/dither_replay.py`）测得真实到达 ADC 的
  脉冲 10–90% 宽度 **74–123 样本**，而理想注入脉冲只有 32 样本——模拟链路
  展宽 2–4 倍。固件聚合窗口按理想模板尺寸开，真实脉冲被截断，两条边
  被不同地偏置。注入更宽的脉冲可同时改善 H5 与 H6（模板变宽 → 窗口
  变宽 → 截断比例下降）。

---

## 4. 阶段计划

### Phase 0：取证（先做零成本实验）

**优先实验（已备好物料，2026-08-16）**：换宽线性斜坡三角脉冲——
`waveform_generation/generated_samples/
sine_199p375MHz_2p6GSPS_impulse_dither_001.txt`（199.375 MHz、
linear 三角、edge 48 DAC / top 0、其余参数不变，`check` 6/6 通过）。
生成器已支持 `--dither-shape linear --dither-edge 48 --dither-top 0`。
下次台架：加载该向量 → `adc -ref` → `adc -cal`，验收：
- `Dither fine-skew estimate: VALID`、`Dither cross-check: VALID`
- `|rising − falling| ≤ 0.03 samples`、`|dither − tone| < 23 ps`
- Stage-4 tone 收敛不变（对照 +1.1 ps / reg 35 / 2/2）

**离线诊断工具（已就绪，无需台架）**：
- `calibration_loop/dither_replay.py`：板捕获 CSV → 重建 → tone 拟合 →
  残差 → 事件 → 聚合 profile → 三种 skew 估计 + 脉冲宽度诊断。
  已支持 `--window-half` / `--gate` 参数，用于离线扫描（窗口 × 门控分数）
  网格；已复现板级失败特征（旧捕获 + 64/0.15 → 227 ps，板级 269 ps
  同量级）。
- `calibration_loop/dither_geometry_scan.py`：合成模型扫描脉冲几何
  （VB 项目的秒级加速器移植到本仓库的采样率上）。理想模型下当前几何
  RC16/32 分歧仅 2.3 ps → 板级分歧来自模拟链路畸变/截断（H6），
  而非理想几何本身。
- `calibration_loop/dither_dispersion_check.py`：实测展宽尺度（sigma 34）
  下的分散/截断实验——模板窗 105.9 ps vs ±64 窗 1.5 ps，H6 据此实现。

**板级进展记录（2026-08-16 逐轮）**：
- 事件检测 VALID、聚合失败（±64 越界）→ 边界跳过修复 → 10/10 ✓
- ±64 无门控：分歧 618.3 ps（tone 残留/振铃尾巴污染投影）
- ±64 + 门控 0.15：**269.4 ps**（↓2.3×，方向正确但距 23.1 ps 门限 11.7×）
- 离线参数扫描（5 帧 21:06 捕获 × 25 组合：window 16/24/32/48/64 ×
  gate 0/0.05/0.10/0.15/0.25）：**最优组合 (64, 0.15) 分歧 168 ps，
  全部组合 ≥168 ps**；逐帧"延迟+宽度"分解：延迟分量 ±100 ps 跳动、
  宽度差 −459…+53 ps 不稳定 → 两参数模型也不可达 23.1 ps 交叉校验门限

**定案（2026-08-16）**：调参路线与两参数模型路线均被数据否决。
Dither fine-skew 分歧是台架模拟链路脉冲色散（注入 32–48 样本 →
实测 10–90% 宽 74–123 样本）加上批次噪声导致的**结构性限制**，
估计器正确地报 INVALID 而非输出错误值（门限按设计工作）。
采用**量化限制路线**：dither 保持 advisory，fine-skew 保持 INVALID
（真实状态），报告写入完整证据链——事件检测 VALID（Stage-1 依赖它
正常工作）、tone 主估计器不受影响（收敛 <1 ps，2/2 PASS）、
25 组合穷举扫描证明分歧下限 ~168 ps 不可再调。

剩余行动：把本节证据链整理进报告 limitation 小节；可选给固件加
"脉冲宽度诊断"打印（UART 直接出 10–90% 宽度，报告可引板级数字）。

**剩余取证**（若换几何后仍 INVALID）：
1. 抓新几何下的原始 DMA 帧，用 `dither_replay.py` 对比换几何前后的
   脉冲宽度与逐边沿分歧；
2. 若分歧持续，再实施 Phase 1 的固件侧修复（per-edge gain、
   聚合窗口加宽等）。

### Phase 1：按假设定向修复（一次一个，做完回板验证）

- **H5（已完成，波形侧）**：linear 三角脉冲已生成并通过 6/6 检查
  （`--dither-shape linear --dither-edge 48 --dither-top 0`），
  待板级验证；无需固件改动。
- **H6（已实现，待板级验证）**：`adc_calibration_skew.c` 的聚合 profile
  窗口支持配置化加宽——新增 `adc_cal_skew_config_t.profile_window_half_samples`
  （0 = 沿用模板尺寸的旧行为；N>0 = 对称加宽到 ±N）。板级 dither 交叉校验
  路径设 64（`CAL_SKEW_DITHER_PROFILE_WINDOW_HALF`，小于 130 样本事件
  周期的一半，无重叠）；`ADC_CAL_SKEW_MAX_PROFILE_SAMPLES` 128→192。
  离线模型验证（sigma 34 展宽）：RC16/32 模板窗分歧 105.9 ps → ±64 窗
  **1.5 ps**；linear48/0 模板窗 54.2 ps → ±64 窗 **2.6 ps**。
  sim 新增窗口加宽冒烟测试，5659/0 全绿，aarch64 语法检查干净。
  板级验证：烧录新固件后沿用已上传的 linear 48/0 参考（或 RC 16/32
  亦可——加宽窗口后两种几何都收敛）重跑 `adc -cal`。
- **H3**：
  - 聚合前对每个事件做亚采样对齐（质心/过零点），消除事件间整采样抖动；
  - 或改用"自校准模板"：用 A 残差聚合 profile 做事件定位模板，
    替代 DAC 文本模板（模板与数据自洽）。
- **H2**：残差净化——聚合前对残差做 tone 频率陷波或相位校正；
  A/B 使用同一拟合参数与窗口；观察 disagreement 是否下降。
- **H1**：模型升级——B 对 A 的拟合从"单一延迟"升级为
  "延迟 + 边宽（rising/falling 各自延迟）"参数模型：
  - 若边宽差稳定，分解出物理 skew 并新增 diagnostic 字段报告边沿
    不对称度（不静默吞掉）；
  - dither 的 VALID 判定改为"模型残差可解释"，而不是强制边沿一致。
- **H4**：若 Phase 0 证明是符号/掩码 bug，直接修共享模块 + 回归测试。

### Phase 2：主机测试加固（与修复同步）

**已完成（2026-08-16）**：`calibration_sim/sim_tests.c`
`unit_skew_estimator_direct` 新增：

1. 对称非刚体（±0.40 边沿平移）契约测试：符号正确、rising ≡ −falling、
   门限触发、full-profile 落在中点；
2. 板级同款不对称用例（rise −0.12 / fall +0.10 samples）：rising < 0 <
   falling、disagreement > 0.03、full-profile 介于两边沿估计之间、
   不对称度越大 disagreement 越大（单调）；
3. tone/事件同步回归：交替极性脉冲下，半整数 cycles/event 的 tone 偏置
   估计、整数 cycles/event 的 tone 抵消（编码 `check.py` 第 5 项的机制）。

**测试抓出的模型缺陷（Phase 1 候选修复 #1）**：边沿估计被系统性衰减
~0.31×——`estimate_profile_skew` 共用一个在全 profile 上投影的 `gain_b`，
非刚体脉冲时两条边通过该增益互相耦合（对称 ±0.40 注入只恢复 ±0.125）。
板级 167 ps 分歧因此**低估**了真实边沿不对称度约 3×。候选修法：per-edge
gain 归一化（rising/falling 各自投影增益），或升级为延迟+边宽参数模型。

**待办**：

1. 板帧回放 fixture：Phase 0 抓的板帧进 `adc_data/`，sim 输出必须复现
   板级 rising/falling（先复现、再修复）；
2. 保持现有 5644 unit / 18 scenario / 14 pipeline 全绿；共享模块保持
   BSP-free、警告干净、确定性。

**注意**：`ADC_CAL_TEST_*` 三个 fixture 文件（100 MHz 波形 + 两个
adc_data 捕获）是 `--run-all` 的硬依赖，已被 git restore 恢复；若要
移除，必须先改 `calibration_sim` 的 fixture 路径定义并换用新 fixture。

### Phase 3：板级验证

1. 重跑 `adc -cal`，验收标准见第 1 节；
2. 对照 Stage-4：tone skew 收敛轨迹、寄存器、2/2 passes 与修复前一致；
3. 若 VALID 但 `|dither − tone| > 10 ps`：报告诊断字段（边宽差、tone
   泄漏指标），决定是否继续下一假设，不强行通过；
4. 把本计划执行结果补进 `CONVERSATION_LOG_*.md`。

---

## 5. 风险与护栏

- `adc_calibration_skew.c` / `adc_calibration_dither.c` 是共享模块
  （固件 + `calibration_sim` 双编译）：任何改动必须 `adc_cal_sim
  --run-all` 全绿 + aarch64 交叉编译干净；
- 不动：Stage-4 表征（1→2→4）、基线恢复、controller 增益/容差、
  tone 相位分支选择、`0.03 samples` 门限；
- gain 阶段 dither `FIT_QUALITY`（0.44 vs ~0.99）纳入 Phase 0 诊断，
  但单独评估——可能与模板形状同根因，也可能独立；
- 若 Phase 0 结论是 H1（纯物理形状差异）：不强行改代码让 dither
  "变绿"，改为报告边沿不对称度并把该限制写入文档。
