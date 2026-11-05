# TIADC 校正專案術語表

對應 `summer2025_research_tiadc` 倉庫中 `firmware/thesis_v3_loop_appl/README.md` 與板上環路 log（`adc -cal loop`）裡常見的術語。

---

## 一、skew 與 code

| 術語 | 意思 |
|---|---|
| **skew（時間偏差）** | A、B 兩個 ADC 通道本該在同一瞬間採樣，但硬體讓它們差了一點時間，單位 ps（皮秒，10⁻¹² 秒）。正值 = B 比 A 晚採樣。兩路一起用時，skew 會產生誤差 spur，所以要調到接近 0。 |
| **code（控制碼）** | AD9695 內部可調的採樣時脈延遲（fine delay）的**檔位編號**，整數 0..48，中位（neutral）是 24。code ±1 = B 相對 A 的採樣時間移動一小步。log 裡 `code=23 → 12` 表示環路把檔位一檔一檔從 23 調到 12。 |
| **ps/code** | 每移動 1 個 code，skew 變多少 ps。實測不固定：199 MHz 時約 16.5、399 MHz 時約 7.1、9 月的工作點約 19 ps/code。 |
| **actuator（執行器）** | 能實際改變 skew 的東西，這裡就是 fine delay 加上它的 code。 |
| **skew transaction（skew 事務）** | 移動 code 的唯一安全途徑：寫暫存器 → 回讀 → reset JESD → 預熱採集 → 確認拿到新幀。成功時輸出 `result=OK`。 |
| **ACK（確認）** | 事務回報「成功了」。環路只在 ACK 是 OK 時才相信 code 真的變了。 |
| **deadband（死區）** | 預設 10 ps。skew 誤差小於它就不再移動，因為一次移動 1 code 就有 7–19 ps，誤差已經很小時再動反而會越過零點。 |
| **direction check / latch（方向檢查 / 鎖死）** | 每次移動後，下一個 batch 的誤差應該朝預測的方向變。方向反了就把 skew 控制鎖死關掉（`skew_abort=1`），gain/offset 環路繼續跑。 |

## 二、gain 與 offset

| 術語 | 意思 |
|---|---|
| **gain（增益）** | 兩個通道對同一個信號放大的倍數不一樣。`tone B/A` = B 通道測到的主音幅度 ÷ A 通道的幅度，理想值 1。 |
| **offset（偏移）** | 輸入為 0 時通道輸出卻不是 0 的固定差值（DC 偏移），單位 codes。`B−A` 是兩通道之差，理想值 0。 |
| **ADC code / codes（ADC 輸出碼）** | ⚠️ 和上面的「控制碼 code」**是兩回事**。這裡指 ADC 輸出的數字值，14 位元有號數，約 −8192..+8191。offset、dither 幅度都以它為單位，1 code ≈ 97 µV。 |
| **LSB（最低有效位）** | 數位量的最小單位。有時指 DAC 的單位（例如 dither 幅度 5190 LSB），有時指 ADC 的單位（≈ codes），要看上下文。 |
| **correction（修正）** | 環路學到的補償值 `gain_corr_a/b`、`offset_a/b`。套用方式：`(raw − offset) · gain_corr`。 |
| **residual（殘餘）** | 套用修正之後，估計器還看得到的剩餘誤差。「收斂」就是殘餘趨近 0（或 1）。 |

## 三、信號與激勵

| 術語 | 意思 |
|---|---|
| **tone（主音）** | DAC 送進 ADC 的正弦參考信號，例如 199.375 MHz、399.375 MHz。gain 和 skew 主要從它的幅度與相位估出來。 |
| **dither / impulse dither（擾動脈衝）** | 疊加在 tone 上、按 ± 極性序列排列的小脈衝串。用來對齊幀，也可以獨立測 gain/offset/skew（tone-free 路線）。 |
| **DPG / DAC vector（波形向量）** | DPG 是驅動 AD9164 DAC 的波形產生器，vector 是要播放的波形檔（`waveforms/tone_dither/*.txt`）。`--wave K` 必須告訴韌體 DPG 正在播哪一個。 |
| **sig_cycles** | 一段波形裡 tone 的週期數，決定 tone 頻率。1276 → 199.375 MHz，2556 → 399.375 MHz。 |
| **polarity（極性）** | dither 脈衝的正負號。這塊板子脈衝整體是反的，所以 log 裡 `polarity=-1` 是正常的。 |

## 四、資料與硬體

| 術語 | 意思 |
|---|---|
| **frame / capture（幀 / 採集）** | DMA 一次抓回來的一段原始資料，4095 bytes，A/B 兩通道交錯存放。 |
| **DMA** | 直接記憶體存取：硬體自動把 ADC 資料搬進記憶體 `RxBufferPtr`，不經 CPU 逐個複製。 |
| **JESD204B / JESD link** | ADC 和 FPGA 之間的高速串列介面。每次改採樣時脈延遲都要 reset 這條 link，約幾毫秒重新同步，所以事務裡有「預熱採集」這一步。 |
| **stale buffer / generation（舊緩衝 / 幀代數）** | 每抓到一幀新資料 generation 加 1。計數沒變 = 拿到的是舊資料，要拒收。 |
| **de-framing / rotation / n0（拆幀 / 旋轉 / 起點）** | 把原始字拆成 A、B 兩通道（`rotation` 決定從哪裡切），再找出 dither 脈衝在這一幀裡的起點 `n0`。 |
| **align margin（對齊裕度）** | 脈衝對齊的可信程度。`w06` 波形正常約 17，低於 6 就拒收。明顯偏低通常代表 DPG 放錯波形。 |
| **A53 / PS / PL** | A53 是 Zynq 晶片裡的 ARM CPU，韌體和新環路跑在它上面。PS = 處理器部分，PL = FPGA 邏輯部分。 |
| **bring-up** | 開機後初始化整套板子（時脈、JESD、ADC）的過程。每次重新載入韌體就是一次新的 bring-up，工作點可能不同。 |
| **ELF** | 編譯出來、載入板子執行的韌體檔。 |

## 五、環路相關

| 術語 | 意思 |
|---|---|
| **loop / closed loop / open loop（環路 / 閉環 / 開環）** | 閉環（`adc -cal loop N`）：測量後真的去移動 skew。開環（`loop open N`）：只測量，不寫執行器。 |
| **iteration（迭代）** | 處理一幀 = 一次迭代，log 裡的 `it=`。 |
| **qualified / rejected（合格 / 拒收）** | 通過過濾器的幀才算樣本。`loop 300` = 湊滿 300 個合格樣本。 |
| **batch（批）** | 每 20 個合格幀取一次平均，做一次 skew 決策，最多移動 1 code。 |
| **LMS / mu（最小均方 / 步長）** | gain/offset 每幀朝誤差反方向修正一小步，修正量 = `mu` × 誤差。`mu = 0.35`，約 10 幀收斂。 |
| **converge（收斂）** | 誤差進入目標範圍並保持住不再變化。 |
| **spur / dBc（雜散 / 相對載波分貝）** | 失配會在頻譜上產生不該有的尖峰（spur）。dBc 是它相對主音的強度，越負越好，例如 −48 dBc。 |
| **ppm（百萬分之一）** | 表示很小的比例。`tone_ppm=1000` = tone B/A 為 1.001。 |
| **scaled integers（縮放整數）** | `xil_printf` 不能印小數，所以 log 把小數放大成整數：`skew_cps` 單位 0.01 ps，`doff_mc` 單位 0.001 codes。例如 `skew_cps=-706` = −7.06 ps。 |

## 六、動態性能指標（SNDR 等）

| 術語 | 意思 |
|---|---|
| **SNDR（信號對雜訊加失真比，Signal-to-Noise-and-Distortion Ratio）** | 衡量 ADC 輸出「乾不乾淨」最常用的一個數字：`SNDR = 10·log10(主音功率 ÷ (雜訊功率 + 失真功率))`，單位 dB，越大越好。做法是對一段採集做 FFT，主音那幾個 bin 算「信號」，**其餘所有東西**（隨機雜訊、諧波、spur、失配產生的鏡像）都算進分母。也叫 SINAD。 |
| **SNDR 的上限** | 理想 N 位元 ADC 只有量化雜訊時：`SNDR = 6.02·N + 1.76 dB`，14 位元 ≈ 86 dB。真實的 AD9695 在滿刻度輸入時約 64–66 dB（datasheet），也就是實際只有 ~10.5 位元有效。 |
| **SNDR 和信號大小有關** | SNDR 是「相對主音」的比值。雜訊底大致固定，所以主音每小 1 dB，SNDR 就少約 1 dB。這個 bench 的主音只用到 ADC 範圍的約 24%（約 −12 dBFS，Δ≈4000 codes），光這點就把可達的 SNDR 從 ~65 dB 壓到 ~53 dB；再加上 DAC 的雜訊/spur、dither 扣除後的殘渣、只有 ~1020 點的短記錄，實測每通道 SNDR 約 37–41 dB（例如 `run6` 約 37.7 dB、模擬 40.6 dB）。 |
| **專案裡 SNDR 怎麼算** | `calibration_loop/metrics.py` 的 `analyse()`：先從記錄裡**扣掉估計出的 dither**（`loop.py` 的 `_measure()`），再加 Blackman-Harris 窗做 FFT，去掉 DC 附近的 bin，主音 ±8 bin 算信號，其餘算雜訊+失真。韌體 pipeline 在 `adc_calibration_performance.c`。log 欄位：`raw_a_sndr_db`（未校正）、`cal_a_sndr_db`（校正後）。 |
| **為什麼校正後單通道 SNDR 幾乎不變** | 目前 A、B 是**並聯**同時採樣，不是交錯。gain/offset 修正只是把單一通道整體放大或平移，信號和雜訊一起變，比值不變（模擬裡 `SNDR raw → cal 40.56 → 40.56 dB`）。失配的效果要看 **A−B 差值 spur**（`raw_difference_dbc` / `cal_difference_dbc`）或交錯輸出的**鏡像 spur**。 |
| **真正交錯時失配怎麼吃掉 SNDR** | 兩路交錯成一路輸出時，失配會產生 spur：offset 失配 → 固定在 fs/2 的 spur；gain 和 skew 失配 → 在 `fs/2 − f_in` 的鏡像。skew 造成的鏡像約為 `20·log10(π·f_in·Δt)` dBc：199 MHz、Δt = 175 ps → 約 −19 dBc；校正到 7 ps → 約 −47 dBc。這些 spur 都算在 SNDR 的分母裡，所以**校正 skew/gain/offset 就是在把交錯 ADC 的 SNDR 拉回來**。 |
| **SNR（信號雜訊比）** | 和 SNDR 一樣，但分母**不含**諧波（前幾次諧波另外算）。SNR ≥ SNDR；兩者差很多代表失真（諧波）是主要問題。 |
| **THD（總諧波失真）** | 2～5 次諧波的總功率相對主音，dBc，越負越好。量的是非線性。 |
| **SFDR（無雜散動態範圍）** | 主音和**最大的那一根** spur 之間差幾 dB。SNDR 看全部加總，SFDR 只看最糟的一根。 |
| **ENOB（有效位元數）** | 把 SNDR 換回「等效位元」：`ENOB = (SNDR − 1.76) / 6.02`。SNDR 37.7 dB → ENOB ≈ 6.0 位元。log 欄位 `*_enob`。 |
| **dBFS vs dB / dBc** | dBFS 是相對 **ADC 滿刻度**（0 dBFS = 用滿 ±8192 codes）；dB / dBc 是相對**主音**。datasheet 常寫 SNR 多少 dBFS（換算到滿刻度），專案 log 的 SNDR 是相對實際主音的 dB，所以主音越小，兩者差越多。 |
