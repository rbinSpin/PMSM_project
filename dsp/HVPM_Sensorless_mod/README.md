# DSP F28335

Replace the corresponding file in the example project, ControlSuite/HVPM_Sensorless_2833x

---

此專案資料夾針對馬達驅動器，從 ControlSuite/HVPM_Sensorless_28335x 修改而來。

原始範例中包含 level 1 到 level 6，本專案移除了其中的 level 4 ~ 6，並後續新增了一個更新的 level 4。

此更新的 level 4 是從 HVPM_Sensored 範例中的 level 4 移植大部分內容而來。

兩者主要的差異在於中斷服務程序（ISR）的觸發條件：
- HVPM_Sensored 範例中的 level 4 使用 ADC 模組觸發 ISR；
- 而此更新的 level 4 則改為使用 EPWM 模組的計時功能作為 ISR 觸發來源。

---

## 使用方法

1. 請先安裝並設定好 TI 的 Code Composer Studio (CCS) 與 ControlSuite。
2. 複製本資料夾內容，取代 ControlSuite 中對應的 HVPM_Sensorless_28335x 範例檔案。
3. 使用 CCS 開啟專案檔並編譯。
4. 確認硬體連線（F28335、馬達、驅動器、電源等）正確無誤。
5. 將程式燒錄至 F28335，執行程式。
6. 可透過 CCS 的圖形化介面或 Watch 視窗觀察變數與控制流程。
7. 若需進一步調整參數或控制邏輯，請修改對應的 `Level4` 程式碼。

