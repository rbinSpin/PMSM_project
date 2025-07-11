# Simulink 回授控制模型

本資料夾提供用於馬達控制模擬的 Simulink 模型。 請依照以下步驟操作以完成模擬流程：

---

## 使用方式

1. **初始化參數**

   - 在 MATLAB Command Window 中執行：
     ```matlab
     pm_param_pu
     ```
   - 此腳本會將 `param` 結構變數載入至 workspace 中。
   - `param` 結構中包含各種模擬中會用到的馬達與控制參數。

2. **選擇模型模擬**

   - 本資料夾內提供兩個 Simulink 模型：

     - `mdl_pm_foc_allpu.slx`：

       - 為自行設計之馬達模型，具備完整可調整性與結構開放性。

     - `mdl_pm_foc_allpu_alternative.slx`：

       - 使用 MATLAB/Simulink Toolbox 中提供的馬達模型元件。

   - 依需求開啟任一 `.slx` 模型進行模擬。

