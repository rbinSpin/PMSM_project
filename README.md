# PMSM_project
A PMSM Algorithm Verification System using Xilinx FPGA

---

本專案的檔案結構大致分為以下五個部分：

1. **dsp/**：
   - 存放 F28335 馬達驅動器控制相關的原始碼。

2. **hls/**：
   - 存放用於 FPGA 演算法加速的 High-Level Synthesis (HLS) 原始碼與測試平台（testbench）。

3. **zynqApp/**：
   - 存放於 Xilinx FPGA 開發板（如 Zynq 系列）上運行的應用層程式。

4. **matlab/**：
   - 存放用於馬達控制模擬、分析與參數調整的 MATLAB 程式碼。
   - 與 Simulink Model