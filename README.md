# PMSM_project
A PMSM Algorithm Verification System using Xilinx FPGA

---

本專案的檔案結構大致分為以下五個部分：

1. **dsp/**：
   - 存放 F28335 馬達驅動器控制相關的原始碼。

2. **hls/**：
   - 存放用於 FPGA 演算法加速的 High-Level Synthesis (HLS) 原始碼與測試平台（testbench）。

3. **jupyter/**：
   - 存放於 Xilinx FPGA 開發板（如 Zynq 系列）上運行的應用層程式。
   - 可使用 Jupyter Notebook 進行互動式操作。

4. **matlab/**：
   - 存放用於馬達控制模擬、分析與參數調整的 MATLAB 程式碼。

5. **simulink/**：
   - 存放整體系統的 Simulink 模型，用於系統建模與模擬驗證。

---

若您為首次使用者，請依序參閱各資料夾內的 README 或說明文件以快速了解模組功能與使用方式。

