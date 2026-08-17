# 修复 Dither Fine-Skew INVALID 的计划

- 日期：2026-08-16
- 状态：待执行（Phase 0 需要台架配合抓原始帧）
- 关联：Stage-4 skew cross-check、Stage-3 gain dither warning
- 约束：不动 Stage-4 tone 主路径；dither 保持 advisory，永不 gate 任何 stage

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
