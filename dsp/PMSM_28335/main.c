#include "PeripheralHeaderIncludes.h"
#include "HVPM_Sensored-Settings.h"
#include "IQmathLib.h"
#include "HVPM_Sensored.h"
#include <math.h>



#ifdef FLASH
#pragma CODE_SECTION(MainISR,"ramfuncs");
#pragma CODE_SECTION(OffsetISR,"ramfuncs");
#endif


// Prototype statements for functions found within this file.
interrupt void MainISR(void);
interrupt void OffsetISR(void);
interrupt void spiTxFifoIsr(void);
interrupt void spiRxFifoIsr(void);
void DeviceInit();
void MemCopy();
void InitFlash();
void HVDMC_Protection(void);
void spi_fifo_init(void);


// State Machine function prototypes
//------------------------------------
// Alpha states
void A0(void);  //state A0
void B0(void);  //state B0
void C0(void);  //state C0

// A branch states
void A1(void);  //state A1
void A2(void);  //state A2
void A3(void);  //state A3

// B branch states
void B1(void);  //state B1
void B2(void);  //state B2
void B3(void);  //state B3

// C branch states
void C1(void);  //state C1
void C2(void);  //state C2
void C3(void);  //state C3

// Variable declarations
void (*Alpha_State_Ptr)(void);  // Base States pointer
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch
void (*C_Task_Ptr)(void);       // State pointer C branch

// Used for running BackGround in flash, and ISR in RAM
extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;


int16   VTimer0[4];         // Virtual Timers slaved off CPU Timer 0 (A events)
int16   VTimer1[4];         // Virtual Timers slaved off CPU Timer 1 (B events)
int16   VTimer2[4];         // Virtual Timers slaved off CPU Timer 2 (C events)
int16   SerialCommsTimer;

// Global variables used in this system

Uint16 OffsetFlag=0;
_iq offsetA=0;
_iq offsetB=0;
_iq offsetC=0;
_iq offsetD=0;
_iq offsetE=0;
_iq offsetF=0;
_iq offsetG=0;
_iq K1=_IQ(0.998);      //Offset filter coefficient K1: 0.05/(T+0.05);
_iq K2=_IQ(0.001999);   //Offset filter coefficient K2: T/(T+0.05);
extern _iq IQsinTable[];
extern _iq IQcosTable[];


_iq VdTesting = _IQ(0.0);           // Vd reference (pu)
_iq VqTesting = _IQ(0.3);          // Vq reference (pu)
_iq IdRef = _IQ(0.0);               // Id reference (pu)
_iq IqRef = _IQ(0.05);               // Iq reference (pu)


#if (BUILDLEVEL<LEVEL3)             // Speed reference (pu)
_iq  SpeedRef = _IQ(0.3);          // For Open Loop tests
#elif (BUILDLEVEL == LEVEL6)
_iq  SpeedRef = _IQ(0.0);
#else
_iq  SpeedRef = _IQ(0.3);           // For Closed Loop tests
#endif



float32 T = 0.001/ISR_FREQUENCY;    // Sampling period (sec), see parameter.h



//////////////////////////////////////////////////////////////////////////////////////////
// PK 2023.07 Test Update
float TestData = 0;
float TestData_Sin = 0;
float LOG_rdata[314];               // Record data from PYNQ-Z2
int LOG_rid = 0;
//////////////////////////////////////////////////////////////////////////////////////////
float PI_timer = 0.0002;
float TIMER = 0;
Uint32 rx_test = 0;

Uint32 IsrTicker = 0;
Uint16 BackTicker = 0;
Uint32 LV6_Ticker = 0;
volatile Uint16 lsw=0;
Uint16 TripFlagDMC=0;               //Trip status
Uint16 Init_IFlag=0;
Uint32 SigTicker = 0;

// check SPI if ready for receive data
char spi_flag = 0;


// my timer parameter
Uint32 elapsed_time_cycles = 0;
Uint32 elapsed_time_cycles1 = 0;
float elapsed_time = 0;


// Default ADC initialization
int ChSel[16]   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int TrigSel[16] = {5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};
int ACQPS[16]   = {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};

int16 DlogCh1 = 0;
int16 DlogCh2 = 0;
int16 DlogCh3 = 0;
int16 DlogCh4 = 0;


volatile Uint16 EnableFlag = FALSE;


Uint16 SpeedLoopPrescaler = 1;      // Speed loop prescaler
Uint16 SpeedLoopCount = 1;           // Speed loop counter


// Instance a few transform objects
CLARKE clarke1 = CLARKE_DEFAULTS;
CLARKE clarke3V = CLARKE_DEFAULTS;
PARK park1 = PARK_DEFAULTS;
PARK park3V = PARK_DEFAULTS;
IPARK ipark1 = IPARK_DEFAULTS;

// Instance PI regulators to regulate the d and q  axis currents, and speed
PI_CONTROLLER pi_spd = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_id  = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_iq  = PI_CONTROLLER_DEFAULTS;

// Instance a PWM driver instance
PWMGEN pwm1 = PWMGEN_DEFAULTS;

// Instance a PWM DAC driver instance
PWMDAC pwmdac1 = PWMDAC_DEFAULTS;

// Instance a Space Vector PWM modulator. This modulator generates a, b and c
// phases based on the d and q stationary reference frame inputs
SVGEN svgen1 = SVGEN_DEFAULTS;

// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL rc1 = RMPCNTL_DEFAULTS;

//  Instance a ramp generator to simulate an Angle
RAMPGEN rg1 = RAMPGEN_DEFAULTS;

// Instance a speed calculator based on QEP
SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;

// Instance a QEP interface driver
QEP qep1 = QEP_DEFAULTS;

// Create an instance of DATALOG Module
DLOG_4CH dlog = DLOG_4CH_DEFAULTS;

// Create an instance of SPI Module
RECEIVED received1 = SPI_DEFAULTS;
PK_RECEIVED pk_received = SPI_DEFAULTS;

// Create an instance of LQR Module
LQR lqr1 = LQR_DEFAULTS;

//// Create an instance of SDRE Module
//SDRE sdre1 = SDRE_DEFAULTS;

void main(void)
{
    DeviceInit();            // Device Life support & GPIO
    spi_fifo_init();         // Initialize the SPI FIFO

// Only used if running from FLASH
// Note that the variable FLASH is defined by the compiler


#ifdef FLASH
// Copy time critical code and Flash setup code to RAM
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the linker files.
    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
    InitFlash();    // Call the flash wrapper init function
#endif //(FLASH)

   // Waiting for enable flag set
   while (EnableFlag==FALSE)
    {
      BackTicker++;
    }

// Timing sync for background loops
// Timer period definitions found in device specific PeripheralHeaderIncludes.h
    CpuTimer0Regs.PRD.all =  mSec10;     // A tasks
    CpuTimer1Regs.PRD.all =  mSec100;
#if (BUILDLEVEL!=LEVEL4 && BUILDLEVEL!=LEVEL6 && BUILDLEVEL!=LEVEL7)
    CpuTimer1Regs.PRD.all =  mSec5;     // B tasks
#endif
    CpuTimer2Regs.PRD.all =  mSec50;    // C tasks

// Tasks State-machine init
    Alpha_State_Ptr = &A0;
    A_Task_Ptr = &A1;
#if (BUILDLEVEL!=LEVEL4 && BUILDLEVEL!=LEVEL6 && BUILDLEVEL!=LEVEL7)
    B_Task_Ptr = &B1;
#endif
    C_Task_Ptr = &C1;

// Initialize PWM module
    pwm1.PeriodMax = SYSTEM_FREQUENCY*1000000*T/2;  // Prescaler X1 (T1), ISR period = T x 1
    pwm1.HalfPerMax=pwm1.PeriodMax/2;
    pwm1.Deadband  = 2.0*SYSTEM_FREQUENCY;          // 120 counts -> 2.0 usec for TBCLK = SYSCLK/1
    PWM_INIT_MACRO(1,2,3,pwm1)

// Initialize PWMDAC module
    pwmdac1.PeriodMax=500;          // @60Mhz, 1500->20kHz, 1000-> 30kHz, 500->60kHz
    pwmdac1.HalfPerMax=pwmdac1.PeriodMax/2;
    PWMDAC_INIT_MACRO(6,pwmdac1)    // PWM 6A,6B
    PWMDAC_INIT_MACRO(7,pwmdac1)    // PWM 7A,7B


// Initialize DATALOG module
    dlog.iptr1 = &DlogCh1;
    dlog.iptr2 = &DlogCh2;
    dlog.iptr3 = &DlogCh3;
    dlog.iptr4 = &DlogCh4;
    dlog.trig_value = 0x0064;
    dlog.size = 0x00c8;
    dlog.prescalar = 10;
    dlog.init(&dlog);


// Initialize ADC for DMC Kit Rev 1.1
    ChSel[0]=1;     // Dummy meas. avoid 1st sample issue Rev0 Picollo*/
    ChSel[1]=1;     // ChSelect: ADC A1-> Phase A Current
    ChSel[2]=9;     // ChSelect: ADC B1-> Phase B Current
    ChSel[3]=3;     // ChSelect: ADC A3-> Phase C Current
    ChSel[4]=15;    // ChSelect: ADC B7-> Phase A Voltage
    ChSel[5]=14;    // ChSelect: ADC B6-> Phase B Voltage
    ChSel[6]=12;    // ChSelect: ADC B4-> Phase C Voltage
    ChSel[7]=7;     // ChSelect: ADC A7-> DC Bus  Voltage

// Initialize ADC module
    ADC_MACRO_INIT(ChSel,TrigSel,ACQPS)


// Initialize QEP module
    qep1.LineEncoder = 2048;  //2500
    qep1.MechScaler = _IQ30(0.25/qep1.LineEncoder);
    qep1.PolePairs = POLES/2;
    qep1.CalibratedAngle = 1000;
    QEP_INIT_MACRO(1,qep1);

// Initialize the Speed module for QEP based speed calculation
    speed1.K1 = _IQ21(1/(BASE_FREQ*T));
    speed1.K2 = _IQ(1/(1+T*2*PI*5));  // Low-pass cut-off frequency
    speed1.K3 = _IQ(1)-speed1.K2;
    speed1.BaseRpm = 200*(BASE_FREQ/POLES);
    speed1.ZERORpm = _IQmpy(speed1.BaseRpm,_IQdiv(speed1.SpeedScaler,0xFFFF));

// Initialize the RAMPGEN module
    rg1.StepAngleMax = _IQ(BASE_FREQ*T);

// Initialize the PI module for Id
    pi_spd.Kp=_IQ(1.0);
    pi_spd.Ki=_IQ(T*SpeedLoopPrescaler/0.2);
    pi_spd.Umax =_IQ(0.9);
    pi_spd.Umin =_IQ(-0.9);

// Initialize the PI module for Iq
    pi_id.Kp=_IQ(1.0);
    pi_id.Ki=_IQ(T/0.04);
    pi_id.Umax =_IQ(0.3);
    pi_id.Umin =_IQ(-0.3);

// Initialize the PI module for speed
    pi_iq.Kp=_IQ(1.0);
    pi_iq.Ki=_IQ(T/0.04);
    pi_iq.Umax =_IQ(0.8);
    pi_iq.Umin =_IQ(-0.8);


//  Note that the vectorial sum of d-q PI outputs should be less than 1.0 which refers to maximum duty cycle for SVGEN.
//  Another duty cycle limiting factor is current sense through shunt resistors which depends on hardware/software implementation.
//  Depending on the application requirements 3,2 or a single shunt resistor can be used for current waveform reconstruction.
//  The higher number of shunt resistors allow the higher duty cycle operation and better dc bus utilization.
//  The users should adjust the PI saturation levels carefully during open loop tests (i.e pi_id.Umax, pi_iq.Umax and Umins) as in project manuals.
//  Violation of this procedure yields distorted current waveforms and unstable closed loop operations which may damage the inverter.

//Call HVDMC Protection function
    HVDMC_Protection();

// Reassign ISRs.

    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &OffsetISR;
    PieVectTable.SPIRXINTA = &spiRxFifoIsr;
    PieVectTable.SPITXINTA = &spiTxFifoIsr;
    EDIS;

// Enable PIE group 3 interrupt 1 for EPWM1_INT
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

// Enable CNT_zero interrupt using EPWM1 Time-base
    EPwm1Regs.ETSEL.bit.INTEN = 1;   // Enable EPWM1INT generation
    EPwm1Regs.ETSEL.bit.INTSEL = 1;  // Enable interrupt CNT_zero event
    EPwm1Regs.ETPS.bit.INTPRD = 1;   // Generate interrupt on the 1st event
    EPwm1Regs.ETCLR.bit.INT = 1;     // Enable more interrupts

// interrupt for SPI communication
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER6.bit.INTx1=1;     // Enable PIE Group 6, INT 1
    PieCtrlRegs.PIEIER6.bit.INTx2=1;     // Enable PIE Group 6, INT 2
    IER |= 0x20;                            // Enable CPU INT6

// Enable CPU INT3 for EPWM1_INT:
    IER |= M_INT3;      //     IER = IER | M_INT3;
// Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

// IDLE loop. Just sit and loop forever:
    for(;;)  //infinite loop
    {
        // State machine entry & exit point
        //===========================================================
        (*Alpha_State_Ptr)();   // jump to an Alpha state (A0,B0,...)
        //===========================================================

    }
} //END MAIN CODE



//=================================================================================
//  STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGd wROUND TASKS
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
void A0(void)
{
    // loop rate synchronizer for A-tasks
    if(CpuTimer0Regs.TCR.bit.TIF == 1)
    {
        CpuTimer0Regs.TCR.bit.TIF = 1;  // clear flag

        //-----------------------------------------------------------
        (*A_Task_Ptr)();        // jump to an A Task (A1,A2,A3,...)
        //-----------------------------------------------------------

        VTimer0[0]++;           // virtual timer 0, instance 0 (spare)
        SerialCommsTimer++;
    }

    #if (BUILDLEVEL!=LEVEL4 && BUILDLEVEL!=LEVEL6 && BUILDLEVEL!=LEVEL7)
        Alpha_State_Ptr = &B0;    // Comment out to allow only A tasks
    #else
        Alpha_State_Ptr = &C0;      // Comment out to allow only A tasks
    #endif
}

#if (BUILDLEVEL!=LEVEL4 && BUILDLEVEL!=LEVEL6 && BUILDLEVEL!=LEVEL7)
void B0(void)
{
    // loop rate synchronizer for B-tasks
    if(CpuTimer1Regs.TCR.bit.TIF == 1)
    {
        CpuTimer1Regs.TCR.bit.TIF = 1;              // clear flag

        //-----------------------------------------------------------
        (*B_Task_Ptr)();        // jump to a B Task (B1,B2,B3,...)
        //-----------------------------------------------------------
        VTimer1[0]++;           // virtual timer 1, instance 0 (spare)
    }

    Alpha_State_Ptr = &C0;      // Allow C state tasks
}
#endif

void C0(void)
{
    // loop rate synchronizer for C-tasks
    if(CpuTimer2Regs.TCR.bit.TIF == 1)
    {
        CpuTimer2Regs.TCR.bit.TIF = 1;              // clear flag

        //-----------------------------------------------------------
        (*C_Task_Ptr)();        // jump to a C Task (C1,C2,C3,...)
        //-----------------------------------------------------------
        VTimer2[0]++;           //virtual timer 2, instance 0 (spare)
    }

    Alpha_State_Ptr = &A0;  // Back to State A0
}


//=================================================================================
//  A - TASKS (executed in every 1 msec)
//=================================================================================
//--------------------------------------------------------
void A1(void) // SPARE (not used)
//--------------------------------------------------------
{
    if(EPwm1Regs.TZFLG.bit.OST==0x1)
    TripFlagDMC=1;      // Trip on DMC (halt and IPM fault trip )

    //-------------------
    //the next time CpuTimer0 'counter' reaches Period value go to A2
    A_Task_Ptr = &A2;
    //-------------------
}

//-----------------------------------------------------------------
void A2(void) // SPARE (not used)
//-----------------------------------------------------------------
{

    //-------------------
    //the next time CpuTimer0 'counter' reaches Period value go to A3
    A_Task_Ptr = &A3;
    //-------------------
}

//-----------------------------------------
void A3(void) // SPARE (not used)
//-----------------------------------------
{

    //-----------------
    //the next time CpuTimer0 'counter' reaches Period value go to A1
    A_Task_Ptr = &A1;
    //-----------------
}


#if (BUILDLEVEL!=LEVEL4 && BUILDLEVEL!=LEVEL6 && BUILDLEVEL!=LEVEL7)
//=================================================================================
//  B - TASKS (executed in every 5 msec)
//=================================================================================

//----------------------------------- USER ----------------------------------------

//----------------------------------------
void B1(void) // Toggle GPIO-00
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B2
    B_Task_Ptr = &B2;
    //-----------------
}

//----------------------------------------
void B2(void) //  SPARE
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B3
    B_Task_Ptr = &B3;
    //-----------------
}

//----------------------------------------
void B3(void) //  SPARE
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    B_Task_Ptr = &B1;
    //-----------------
}
#endif

//=================================================================================
//  C - TASKS (executed in every 50 msec)
//=================================================================================

//--------------------------------- USER ------------------------------------------

//----------------------------------------
void C1(void)   // Toggle GPIO-34
//----------------------------------------
{

    if(EPwm1Regs.TZFLG.bit.OST==0x1)            // TripZ for PWMs is low (fault trip)
      { TripFlagDMC=1;
      GpioDataRegs.GPBTOGGLE.bit.GPIO42 = 1;
      }

    if(GpioDataRegs.GPADAT.bit.GPIO15 == 1)     // Over Current Prot. for Integrated Power Module is high (fault trip)
      { TripFlagDMC=1;
      GpioDataRegs.GPBTOGGLE.bit.GPIO44 = 1;
      }

    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;     // Turn on/off LD3 on the controlCARD
    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C2
    C_Task_Ptr = &C2;
    //-----------------

}

//----------------------------------------
void C2(void) //  SPARE
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C3
    C_Task_Ptr = &C3;
    //-----------------
}


//-----------------------------------------
void C3(void) //  SPARE
//-----------------------------------------
{

    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C1
    C_Task_Ptr = &C1;
    //-----------------
}

// MainISR
interrupt void MainISR(void)
{
//    GpioDataRegs.GPBTOGGLE.bit.GPIO48 = 1;
// Verifying the ISR
    IsrTicker++;

// =============================== LEVEL 1 ======================================
//    Checks target independent modules, duty cycle waveforms and PWM update
//    Keep the motors disconnected at this level
// ==============================================================================

#if (BUILDLEVEL==LEVEL1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    rc1.TargetValue = SpeedRef;
    RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1)

//    for (delay = 0; delay < 100000; delay++){};

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
//  There are two option for trigonometric functions:
//  IQ sin/cos look-up table provides 512 discrete sin and cos points in Q30 format
//  IQ sin/cos PU functions interpolate the data in the lookup table yielding higher resolution.
// ------------------------------------------------------------------------------
    ipark1.Ds = VdTesting;
    ipark1.Qs = VqTesting;

//    ipark1.Sine  =_IQ30toIQ(IQsinTable[_IQtoIQ9(rg1.Out)]);
//    ipark1.Cosine=_IQ30toIQ(IQcosTable[_IQtoIQ9(rg1.Out)]);

    ipark1.Sine=_IQsinPU(rg1.Out);
    ipark1.Cosine=_IQcosPU(rg1.Out);
    IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen1.Ualpha = ipark1.Alpha;
    svgen1.Ubeta = ipark1.Beta;
    SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = svgen1.Ta;
    pwm1.MfuncC2 = svgen1.Tb;
    pwm1.MfuncC3 = svgen1.Tc;
    PWM_MACRO(1,2,3,pwm1)                           // Calculate the new PWM compare values

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    pwmdac1.MfuncC1 = svgen1.Ta;
    pwmdac1.MfuncC2 = svgen1.Tb;
    PWMDAC_MACRO(6,pwmdac1)                         // PWMDAC 6A, 6B

    pwmdac1.MfuncC1 = svgen1.Tc;
    pwmdac1.MfuncC2 = svgen1.Tb-svgen1.Tc;
    PWMDAC_MACRO(7,pwmdac1)

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(svgen1.Ta);
    DlogCh2 = _IQtoQ15(svgen1.Tb);
    DlogCh3 = _IQtoQ15(svgen1.Tc);
    DlogCh4 = _IQtoQ15(svgen1.Tb-svgen1.Tc);

#endif // (BUILDLEVEL==LEVEL1)


// =============================== LEVEL 2 ======================================
//    Level 2 verifies the analog-to-digital conversion, offset compensation,
//    clarke/park transformations (CLARKE/PARK), phase voltage calculations
// ==============================================================================

#if (BUILDLEVEL==LEVEL2)
    if(rx_test != 0){
//    if(1){
// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
//    rc1.TargetValue = SpeedRef;
    rc1.TargetValue = _IQ(0.35);
    RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//  Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
    #ifdef DSP2833x_DEVICE_H
    clarke1.As=((AdcMirror.ADCRESULT1)*0.00024414-offsetA)*2*0.909; // Phase A curr.
    clarke1.Bs=((AdcMirror.ADCRESULT2)*0.00024414-offsetB)*2*0.909; // Phase B curr.
    #endif                                                         // ((ADCmeas(q12)/2^12)-offset)*2*(3.0/3.3)

    #ifdef DSP2803x_DEVICE_H
    clarke1.As = _IQmpy2(_IQ12toIQ(AdcResult.ADCRESULT1)-offsetA); // Phase A curr.
    clarke1.Bs = _IQmpy2(_IQ12toIQ(AdcResult.ADCRESULT2)-offsetB); // Phase B curr.
    #endif                                                         // (ADCmeas(q12->q24)-offset)*2

    CLARKE_MACRO(clarke1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;
    park1.Angle = rg1.Out;

    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);
    PARK_MACRO(park1)

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------

    ipark1.Ds = -0.00368896173;
    ipark1.Qs = 0.170242667;

    ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
    IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen1.Ualpha = ipark1.Alpha;
    svgen1.Ubeta  = ipark1.Beta;
    SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = svgen1.Ta;
    pwm1.MfuncC2 = svgen1.Tb;
    pwm1.MfuncC3 = svgen1.Tc;
    PWM_MACRO(1,2,3,pwm1)                           // Calculate the new PWM compare values

// ------------------------------------------------------------------------------
//  Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    pwmdac1.MfuncC1 = clarke1.As;
    pwmdac1.MfuncC2 = clarke1.Bs;
    PWMDAC_MACRO(6,pwmdac1)                         // PWMDAC 6A, 6B

    pwmdac1.MfuncC1 = svgen1.Tc;
    pwmdac1.MfuncC2 = svgen1.Tb-svgen1.Tc;
    PWMDAC_MACRO(7,pwmdac1)

// ------------------------------------------------------------------------------
//  Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(svgen1.Ta);
    DlogCh2 = _IQtoQ15(svgen1.Tb);
    DlogCh3 = _IQtoQ15(svgen1.Tc);
    DlogCh4 = _IQtoQ15(rg1.Out);
    }

#endif // (BUILDLEVEL==LEVEL2)




// =============================== LEVEL 3 ======================================
//  Level 3 verifies the dq-axis current regulation performed by PID and speed
//  measurement modules.
// ==============================================================================
//    lsw=0: lock the rotor of the motor,
//    lsw=1: close the current loop,

#if (BUILDLEVEL==LEVEL3)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    if(lsw==0)rc1.TargetValue = 0;
    else rc1.TargetValue = SpeedRef;
    RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//  Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
    clarke1.As=((AdcMirror.ADCRESULT1)*0.00024414-offsetA)*2*0.909; // Phase A curr.
    clarke1.Bs=((AdcMirror.ADCRESULT2)*0.00024414-offsetB)*2*0.909; // Phase B curr.
    CLARKE_MACRO(clarke1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;
    if(lsw==0) park1.Angle = 0;
    else if(lsw==1) park1.Angle = rg1.Out;

    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);

    PARK_MACRO(park1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PID IQ controller macro
// ------------------------------------------------------------------------------
    if (lsw==0) pi_iq.Ref = 0;
    else pi_iq.Ref = IqRef;
    pi_iq.Fbk = park1.Qs;
    PI_MACRO(pi_iq)

// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PID ID controller macro
// ------------------------------------------------------------------------------
    if (lsw==0) pi_id.Ref = _IQ(0.05);  // Lock the rotor
    else pi_id.Ref = IdRef;
    pi_id.Fbk = park1.Ds;
    PI_MACRO(pi_id)

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ipark1.Ds = pi_id.Out;
    ipark1.Qs = pi_iq.Out ;
    ipark1.Sine   = park1.Sine;
    ipark1.Cosine = park1.Cosine;
    IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//    Detect calibration angle and call the QEP module
// ------------------------------------------------------------------------------
    if (lsw==0) {EQep1Regs.QPOSCNT=0; EQep1Regs.QCLR.bit.IEL = 1;} // Reset position cnt.

    if ((EQep1Regs.QFLG.bit.IEL==1) && Init_IFlag==0)              // Check the index occurrence
       {qep1.CalibratedAngle= EQep1Regs.QPOSILAT; Init_IFlag++;}   // Keep the latched pos. at the first index

    if (lsw!=0) QEP_MACRO(1,qep1);

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
    speed1.ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta);
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen1.Ualpha = ipark1.Alpha;
    svgen1.Ubeta  = ipark1.Beta;
    SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = svgen1.Ta;
    pwm1.MfuncC2 = svgen1.Tb;
    pwm1.MfuncC3 = svgen1.Tc;
    PWM_MACRO(1,2,3,pwm1)                           // Calculate the new PWM compare values

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    pwmdac1.MfuncC1 = clarke1.As;
    pwmdac1.MfuncC2 = clarke1.Bs;
    PWMDAC_MACRO(6,pwmdac1)                         // PWMDAC 6A, 6B

    pwmdac1.MfuncC1 = rg1.Out;
    pwmdac1.MfuncC2 = speed1.ElecTheta;
    PWMDAC_MACRO(7,pwmdac1)                         // PWMDAC 7A, 7B

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(qep1.ElecTheta);
    DlogCh2 = _IQtoQ15(rg1.Out);
    DlogCh3 = _IQtoQ15(clarke1.As);
    DlogCh4 = _IQtoQ15(clarke1.Bs);

#endif // (BUILDLEVEL==LEVEL3)


    // =============================== LEVEL 4 ======================================
    //    Level 4 verifies the speed regulator performed by PI module.
    //    The system speed loop is closed by using the measured speed as a feedback.
    // ==============================================================================
    //    lsw=0: lock the rotor of the motor,
    //    lsw=1: close the current loop,
    //    lsw=2: close the speed loop (sensored FOC).

#if (BUILDLEVEL==LEVEL4)
//    if(1){
    if(rx_test != 0){
// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    if(lsw==0)rc1.TargetValue = 0;
    else rc1.TargetValue = SpeedRef;
    RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//  Connect inputs of the CLARKE module and call the clarke transformation macro
//                            1/4096 = 0.00024414  3.0/3.3 = 0.909 (0~3.3 -> 0~3.0)
// ------------------------------------------------------------------------------
    clarke1.As=((AdcMirror.ADCRESULT1)*0.00024414-offsetA)*2*0.909; // Phase A curr.
    clarke1.Bs=((AdcMirror.ADCRESULT2)*0.00024414-offsetB)*2*0.909; // Phase B curr.
    CLARKE_MACRO(clarke1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;

    if(lsw==0) park1.Angle = 0;
    else if(lsw==1) park1.Angle = rg1.Out;
    else park1.Angle = qep1.ElecTheta;

    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);

    PARK_MACRO(park1)

// ------------------------------------------------------------------------------
//    Connect inputs of the PI module and call the PID speed controller macro
// ------------------------------------------------------------------------------
   if (SpeedLoopCount==SpeedLoopPrescaler)
     {
      pi_spd.Ref = rc1.SetpointValue;
      pi_spd.Fbk = speed1.Speed;
      PI_MACRO(pi_spd);
      SpeedLoopCount=1;
     }
    else SpeedLoopCount++;

    if(lsw!=2)  {pi_spd.ui=0; pi_spd.i1=0;}

// ------------------------------------------------------------------------------
//    Connect inputs of the PI module and call the PID IQ controller macro
// ------------------------------------------------------------------------------
    if(lsw==0) pi_iq.Ref = 0;
    else if(lsw==1) pi_iq.Ref = IqRef;
    else pi_iq.Ref =  pi_spd.Out;
    pi_iq.Fbk = park1.Qs;
    PI_MACRO(pi_iq)

// ------------------------------------------------------------------------------
//    Connect inputs of the PI module and call the PID ID controller macro
// ------------------------------------------------------------------------------
    if(lsw==0) pi_id.Ref = _IQ(0.05);
    else pi_id.Ref = IdRef;
    pi_id.Fbk = park1.Ds;
    PI_MACRO(pi_id)

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------

    ipark1.Ds = pi_id.Out;
    ipark1.Qs = pi_iq.Out;

    ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
    IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//    Detect calibration angle (optional) and call the QEP module
// ------------------------------------------------------------------------------
    if (lsw==0) {EQep1Regs.QPOSCNT=0; EQep1Regs.QCLR.bit.IEL = 1;} // Reset position cnt.

    if ((EQep1Regs.QFLG.bit.IEL==1) && Init_IFlag==0)              // Check the first index occurrence
       {qep1.CalibratedAngle= EQep1Regs.QPOSILAT; Init_IFlag++;}   // Keep the latched position

    if (lsw!=0) QEP_MACRO(1,qep1);

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen1.Ualpha = ipark1.Alpha;
    svgen1.Ubeta  = ipark1.Beta;
    SVGENDQ_MACRO(svgen1)
// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = svgen1.Ta;
    pwm1.MfuncC2 = svgen1.Tb;
    pwm1.MfuncC3 = svgen1.Tc;
    PWM_MACRO(1,2,3,pwm1)                           // Calculate the new PWM compare values

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    pwmdac1.MfuncC1 = clarke1.As;
    pwmdac1.MfuncC2 = clarke1.Bs;
    PWMDAC_MACRO(6,pwmdac1)                         // PWMDAC 6A, 6B

    pwmdac1.MfuncC1 = rg1.Out;
    pwmdac1.MfuncC2 = speed1.ElecTheta ;
    PWMDAC_MACRO(7,pwmdac1)

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(clarke1.As);
    DlogCh2 = _IQtoQ15(clarke1.Bs);
    DlogCh3 = _IQtoQ15(speed1.Speed);
    DlogCh4 = _IQtoQ15(svgen1.Ta);
    }

#endif // (BUILDLEVEL==LEVEL4)


    // =============================== LEVEL 5 ======================================
    //    Level 5 verifies the speed regulator performed by SDRE module and communicate with SPI.
    //    The system is still in open-looped mode; just testing out SDRE-SPI modules.
    // ==============================================================================

#if (BUILDLEVEL==LEVEL5)
    if(rx_test != 0){

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    if(lsw==0) rc1.TargetValue = 0;
    else rc1.TargetValue = SpeedRef;
    RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    #ifdef DSP2833x_DEVICE_H
        clarke1.As=((AdcMirror.ADCRESULT1)*0.00024414-offsetA)*2*0.909; // Phase A curr.
        clarke1.Bs=((AdcMirror.ADCRESULT2)*0.00024414-offsetB)*2*0.909; // Phase B curr.
    #endif                                                         // ((ADCmeas(q12)/2^12)-offset)*2*(3.0/3.3)

    CLARKE_MACRO(clarke1)
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;

    if(lsw==0) park1.Angle = 0;
    else if(lsw==1) park1.Angle = rg1.Out;
    else park1.Angle = qep1.ElecTheta;

    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);
    PARK_MACRO(park1);

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
//  There are two option for trigonometric functions:
//  IQ sin/cos look-up table provides 512 discrete sin and cos points in Q30 format
//  IQ sin/cos PU functions interpolate the data in the lookup table yielding higher resolution.
// ------------------------------------------------------------------------------

    if (lsw == 2) {        // received1.Vd, received1.Vq as input voltage
        SPI_MACRO(received1);
        ipark1.Ds = _IQ(received1.Vd);
        ipark1.Qs = _IQ(received1.Vq);
    }
    else {
        ipark1.Ds = VdTesting;
        ipark1.Qs = VqTesting;
    }

    ipark1.Sine = park1.Sine;
    ipark1.Cosine = park1.Cosine;
    IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//    Detect calibration angle and call the QEP module
// ------------------------------------------------------------------------------
    if (lsw==0) {EQep1Regs.QPOSCNT=0; EQep1Regs.QCLR.bit.IEL = 1;} // Reset position cnt.

    if ((EQep1Regs.QFLG.bit.IEL==1) && Init_IFlag==0)              // Check the index occurrence
       {qep1.CalibratedAngle= EQep1Regs.QPOSILAT; Init_IFlag++;}   // Keep the latched pos. at the first index

    if (lsw!=0) QEP_MACRO(1,qep1);

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1)


// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen1.Ualpha = ipark1.Alpha;
    svgen1.Ubeta = ipark1.Beta;
    SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = svgen1.Ta;
    pwm1.MfuncC2 = svgen1.Tb;
    pwm1.MfuncC3 = svgen1.Tc;
    PWM_MACRO(1,2,3,pwm1)                           // Calculate the new PWM compare values

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    pwmdac1.MfuncC1 = svgen1.Ta;
    pwmdac1.MfuncC2 = svgen1.Tb;
    PWMDAC_MACRO(6,pwmdac1)                         // PWMDAC 6A, 6B

    pwmdac1.MfuncC1 = svgen1.Tc;
    pwmdac1.MfuncC2 = svgen1.Tb-svgen1.Tc;
    PWMDAC_MACRO(7,pwmdac1)

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(svgen1.Ta);
    DlogCh2 = _IQtoQ15(svgen1.Tb);
    DlogCh3 = _IQtoQ15(svgen1.Tc);
    DlogCh4 = _IQtoQ15(svgen1.Tb-svgen1.Tc);
    }

#endif // (BUILDLEVEL==LEVEL5)


    // =============================== LEVEL 6 ======================================
    //    Level 6 verifies the SDRE method with both observer and controller has no
    //    time-dependent term in their SDCs, namely A matrices. In LEVEL 6 will not
    //    need to connect FPGA and the U matrices is constant, therefore, LEVEL 6
    //    tests out if the computation time is the main problem for motor vibrating.
    // ==============================================================================

#if (BUILDLEVEL==LEVEL6)
    if(rx_test != 0){
//    if(1){
    LV6_Ticker++;
// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    if(lsw==0) rc1.TargetValue = 0;
    else rc1.TargetValue = SpeedRef;
    RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    #ifdef DSP2833x_DEVICE_H
        clarke1.As=((AdcMirror.ADCRESULT1)*0.00024414-offsetA)*2*0.909; // Phase U current.
        clarke1.Bs=((AdcMirror.ADCRESULT2)*0.00024414-offsetB)*2*0.909; // Phase V current.

        lqr1.DCbus=((AdcMirror.ADCRESULT7)*0.00024414)*0.909; //  DC bus voltage.
    #endif                                                              // ((ADCmeas(q12)/2^12)-offset)*2*(3.0/3.3)

    CLARKE_MACRO(clarke1)
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;

//// ------------------------------------------------------------------------------
////  Detect calibration angle and call the QEP module
//// ------------------------------------------------------------------------------

    if(lsw==0) park1.Angle = 0;
    else if(lsw==1) park1.Angle = rg1.Out;
    else park1.Angle = qep1.ElecTheta;

    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);
    PARK_MACRO(park1);

    if (SpeedLoopCount==SpeedLoopPrescaler)
    {
        lqr1.wd = (float)(rc1.SetpointValue*2000*R2r);
        lqr1.delta_t = T/10;

        lqr1.w = (float)(speed1.SpeedRpm_fr*R2r);
        lqr1.iqs = (float)(park1.Qs*BASE_CURRENT);  // convert pu to real current value
        lqr1.ids = (float)(park1.Ds*BASE_CURRENT);  // convert pu to real current value

        CALVDQ_MACRO(lqr1);
        SpeedLoopCount=1;
    }
    else SpeedLoopCount++;


// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
//  There are two option for trigonometric functions:
//  IQ sin/cos look-up table provides 512 discrete sin and cos points in Q30 format
//  IQ sin/cos PU functions interpolate the data in the lookup table yielding higher resolution.
// ------------------------------------------------------------------------------

    if (lsw != 2)
    {
        ipark1.Ds = VdTesting;
        ipark1.Qs = VqTesting;
    }
    else
    {
        ipark1.Ds = lqr1.Vd_out;
        ipark1.Qs = lqr1.Vq_out;

    }

    ipark1.Sine = park1.Sine;
    ipark1.Cosine = park1.Cosine;
    IPARK_MACRO(ipark1)


// ------------------------------------------------------------------------------
//    Detect calibration angle and call the QEP module (origin)
// ------------------------------------------------------------------------------
    if (lsw==0) {EQep1Regs.QPOSCNT=0; EQep1Regs.QCLR.bit.IEL = 1;} // Reset position cnt.

    if ((EQep1Regs.QFLG.bit.IEL==1) && Init_IFlag==0)              // Check the index occurrence
       {qep1.CalibratedAngle= EQep1Regs.QPOSILAT; Init_IFlag++;}   // Keep the latched pos. at the first index

    if (lsw!=0) QEP_MACRO(1,qep1);

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------

    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen1.Ualpha = ipark1.Alpha;
    svgen1.Ubeta = ipark1.Beta;
    SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = svgen1.Ta;
    pwm1.MfuncC2 = svgen1.Tb;
    pwm1.MfuncC3 = svgen1.Tc;
    PWM_MACRO(1,2,3,pwm1)                           // Calculate the new PWM compare values

// ------------------------------------------------------------------------------
//  Connect inputs of the PWMDAC module (monitor from oscilloscope)
// ------------------------------------------------------------------------------
    pwmdac1.MfuncC1 = rg1.Out;
    pwmdac1.MfuncC2 = qep1.ElecTheta;
    PWMDAC_MACRO(6,pwmdac1)                         // PWMDAC 6A, 6B

    pwmdac1.MfuncC1 = svgen1.Tc;
    pwmdac1.MfuncC2 = svgen1.Tb-svgen1.Tc;
    PWMDAC_MACRO(7,pwmdac1)

// ------------------------------------------------------------------------------
//  Connect inputs of the DATALOG module (monitor from Graph window)
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(lqr1.iqs);
    DlogCh2 = _IQtoQ15(lqr1.ids);
    DlogCh3 = _IQtoQ15(lqr1.w);
    DlogCh4 = _IQtoQ15(lqr1.Tl);


// ------------------------------------------------------------------------------
//  Change SpeedRef
// ------------------------------------------------------------------------------
    if (LV6_Ticker < 100000)
    {
        SpeedRef = -0.3;
    }
    else if (LV6_Ticker < 150000)
    {
        SpeedRef = 0.3;
    }
    else if (LV6_Ticker < 200000)
    {
        SpeedRef = -0.3;
    }
    else if (LV6_Ticker > 250000)
    {
        SpeedRef = 0.1;
    }


    }
    else
    {
        ReloadCpuTimer1(); // Reload timer counter
    }
#endif // (BUILDLEVEL==LEVEL6)





    // =============================== LEVEL 7 ======================================
    //    Level 7 PK verifies SPI communication with SDRE on FPGA
    // ==============================================================================

#if (BUILDLEVEL==LEVEL7)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    if(lsw==0) rc1.TargetValue = 0;
    else rc1.TargetValue = SpeedRef;
    RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    #ifdef DSP2833x_DEVICE_H
        clarke1.As=((AdcMirror.ADCRESULT1)*0.00024414-offsetA)*2*0.909; // Phase A curr.
        clarke1.Bs=((AdcMirror.ADCRESULT2)*0.00024414-offsetB)*2*0.909; // Phase B curr.
    #endif                                                         // ((ADCmeas(q12)/2^12)-offset)*2*(3.0/3.3)

    CLARKE_MACRO(clarke1)
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;

    if(lsw==0) park1.Angle = 0;
    else if(lsw==1) park1.Angle = rg1.Out;
    else park1.Angle = qep1.ElecTheta;

    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);
    PARK_MACRO(park1);

// ------------------------------------------------------------------------------
//    Detect calibration angle and call the QEP module
// ------------------------------------------------------------------------------
    if (lsw==0) {EQep1Regs.QPOSCNT=0; EQep1Regs.QCLR.bit.IEL = 1;} // Reset position cnt.

    if ((EQep1Regs.QFLG.bit.IEL==1) && Init_IFlag==0)              // Check the index occurrence
       {qep1.CalibratedAngle= EQep1Regs.QPOSILAT; Init_IFlag++;}   // Keep the latched pos. at the first index

    if (lsw!=0) QEP_MACRO(1,qep1);

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  using SPI to send/receive data to FPGA
//  SEND: w, iqs, ids (speed1.Speed, park1.Qs, park1.Ds)
//  RECEIVE: Vq, Vd (ipark1.Qs, ipark1.Ds)
// ------------------------------------------------------------------------------

    // Stop the DSP timer and print the elapsed time
    CpuTimer1Regs.TCR.bit.TSS = 1; // Stop timer
    elapsed_time_cycles = CpuTimer1Regs.TIM.all;
    TIMER = (float)elapsed_time_cycles / 150000000.0;

    CpuTimer1Regs.TCR.bit.TIF = 1; // Clear timer interrupt flag
    CpuTimer1Regs.TCR.bit.TRB = 1; // Reload timer counter
    CpuTimer1Regs.TCR.bit.TSS = 0; // Start timer

    pk_received = pk_transmit((float)TIMER,(float)park1.Qs*BASE_CURRENT,(float)park1.Ds*BASE_CURRENT);

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
//  There are two option for trigonometric functions:
//  IQ sin/cos look-up table provides 512 discrete sin and cos points in Q30 format
//  IQ sin/cos PU functions interpolate the data in the lookup table yielding higher resolution.
// ------------------------------------------------------------------------------
    if (fabs(clarke1.As) >= 0.72 || fabs(clarke1.Bs) >= 0.72)
    {
        ipark1.Ds = VdTesting;
        ipark1.Qs = VqTesting;
    }
    else if (lsw == 2){        // received1.Vd, received1.Vq as input voltage
        ipark1.Ds = received1.Vd;
        ipark1.Qs = received1.Vq;
    }
    else
    {
        ipark1.Ds = VdTesting;
        ipark1.Qs = VqTesting;
    }

    ipark1.Sine = park1.Sine;
    ipark1.Cosine = park1.Cosine;
    IPARK_MACRO(ipark1)


// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen1.Ualpha = ipark1.Alpha;
    svgen1.Ubeta = ipark1.Beta;
    SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = svgen1.Ta;
    pwm1.MfuncC2 = svgen1.Tb;
    pwm1.MfuncC3 = svgen1.Tc;
    PWM_MACRO(1,2,3,pwm1)                           // Calculate the new PWM compare values

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    pwmdac1.MfuncC1 = svgen1.Ta;
    pwmdac1.MfuncC2 = svgen1.Tb;
    PWMDAC_MACRO(6,pwmdac1)                         // PWMDAC 6A, 6B

    pwmdac1.MfuncC1 = svgen1.Tc;
    pwmdac1.MfuncC2 = svgen1.Tb-svgen1.Tc;
    PWMDAC_MACRO(7,pwmdac1)

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(svgen1.Ta);
    DlogCh2 = _IQtoQ15(svgen1.Tb);
    DlogCh3 = _IQtoQ15(svgen1.Tc);
    DlogCh4 = _IQtoQ15(svgen1.Tb-svgen1.Tc);

#endif // (BUILDLEVEL==LEVEL7)


    // =============================== LEVEL 8 ======================================
    //    Level 8 verifies the SDRE method with both observer and controller has no
    //    time-dependent term in their SDCs, namely A matrices. In LEVEL 8 will not
    //    need to connect FPGA and the U matrices is constant, therefore, LEVEL 8
    //    tests out if the computation time is the main problem for motor vibrating.
    //    memory out of usage.
    // ==============================================================================

#if (BUILDLEVEL==LEVEL8)
        if(1){
    LV6_Ticker++;
// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    if(lsw==0) rc1.TargetValue = 0;
    else rc1.TargetValue = SpeedRef;
    RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    #ifdef DSP2833x_DEVICE_H
        clarke1.As=((AdcMirror.ADCRESULT1)*0.00024414-offsetA)*2*0.909; // Phase U current.
        clarke1.Bs=((AdcMirror.ADCRESULT2)*0.00024414-offsetB)*2*0.909; // Phase V current.

//        clarke3V.As=((AdcMirror.ADCRESULT4)*0.00024414-offsetE)*2*0.909;    // phase U voltage.
//        clarke3V.Bs=((AdcMirror.ADCRESULT5)*0.00024414-offsetF)*2*0.909;    // phase V voltage.
//        clarke3V.Cs=((AdcMirror.ADCRESULT6)*0.00024414-offsetG)*2*0.909;    // phase W voltage.

//        lqr1.DCbus=((AdcMirror.ADCRESULT7)*0.00024414)*0.909; //  DC bus voltage.
//        clarke3V.As=((AdcMirror.ADCRESULT4)*0.00024414)*2*0.909;    // phase U voltage.
//        clarke3V.Bs=((AdcMirror.ADCRESULT5)*0.00024414)*2*0.909;    // phase V voltage.
//        clarke3V.Cs=((AdcMirror.ADCRESULT6)*0.00024414)*2*0.909;    // phase W voltage.
    #endif                                                              // ((ADCmeas(q12)/2^12)-offset)*2*(3.0/3.3)

    CLARKE_MACRO(clarke1)
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;

//    CLARKE_MACRO(clarke3V)
//    park3V.Alpha = clarke3V.Alpha;
//    park3V.Beta = clarke3V.Beta;

//// ------------------------------------------------------------------------------
////  Detect calibration angle and call the QEP module
//// ------------------------------------------------------------------------------

    if(lsw==0) park1.Angle = 0;
    else if(lsw==1) park1.Angle = rg1.Out;
    else park1.Angle = qep1.ElecTheta;

    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);
    PARK_MACRO(park1);

// ------------------------------------------------------------------------------
//  Connect inputs of the LQR module and call the DQ-Voltage calculation macro
//  INPUT : Tl, iqs_est, ids_est, w_est, w, iqs, ids, wd, delta_t
//  OUTPUT: Tl, iqs_est, ids_est, w_est, Vq, Vd
// ------------------------------------------------------------------------------

    if (SpeedLoopCount==SpeedLoopPrescaler)
    {
        sdre1 = mySDRE((float)(speed1.SpeedRpm_fr*R2r), (float)(park1.Qs*BASE_CURRENT), (float)(park1.Ds*BASE_CURRENT), T/10, (float)(rc1.SetpointValue*2000*R2r));
        SpeedLoopCount=1;
    }
    else SpeedLoopCount++;


// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
//  There are two option for trigonometric functions:
//  IQ sin/cos look-up table provides 512 discrete sin and cos points in Q30 format
//  IQ sin/cos PU functions interpolate the data in the lookup table yielding higher resolution.
// ------------------------------------------------------------------------------

    if (lsw != 2)
    {
        ipark1.Ds = VdTesting;
        ipark1.Qs = VqTesting;
    }
    else
    {
        ipark1.Ds = _IQ(sdre1.Vd_out);
        ipark1.Qs = _IQ(sdre1.Vq_out);
    }

    ipark1.Sine = park1.Sine;
    ipark1.Cosine = park1.Cosine;
    IPARK_MACRO(ipark1)


// ------------------------------------------------------------------------------
//    Detect calibration angle and call the QEP module (origin)
// ------------------------------------------------------------------------------
    if (lsw==0) {EQep1Regs.QPOSCNT=0; EQep1Regs.QCLR.bit.IEL = 1;} // Reset position cnt.

    if ((EQep1Regs.QFLG.bit.IEL==1) && Init_IFlag==0)              // Check the index occurrence
       {qep1.CalibratedAngle= EQep1Regs.QPOSILAT; Init_IFlag++;}   // Keep the latched pos. at the first index

    if (lsw!=0) QEP_MACRO(1,qep1);

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------

    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen1.Ualpha = ipark1.Alpha;
    svgen1.Ubeta = ipark1.Beta;
    SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = svgen1.Ta;
    pwm1.MfuncC2 = svgen1.Tb;
    pwm1.MfuncC3 = svgen1.Tc;
    PWM_MACRO(1,2,3,pwm1)                           // Calculate the new PWM compare values

// ------------------------------------------------------------------------------
//  Connect inputs of the PWMDAC module (monitor from oscilloscope)
// ------------------------------------------------------------------------------
    pwmdac1.MfuncC1 = rg1.Out;
    pwmdac1.MfuncC2 = qep1.ElecTheta;
    PWMDAC_MACRO(6,pwmdac1)                         // PWMDAC 6A, 6B

    pwmdac1.MfuncC1 = svgen1.Tc;
    pwmdac1.MfuncC2 = svgen1.Tb-svgen1.Tc;
    PWMDAC_MACRO(7,pwmdac1)

// ------------------------------------------------------------------------------
//  Connect inputs of the DATALOG module (monitor from Graph window)
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(speed1.SpeedRpm_fr);
    DlogCh2 = _IQtoQ15(park1.Qs);
    DlogCh3 = _IQtoQ15(park1.Ds);
    DlogCh4 = _IQtoQ15(rc1.SetpointValue);


// ------------------------------------------------------------------------------
//  Change SpeedRef
// ------------------------------------------------------------------------------
    if (LV6_Ticker < 100000)
    {
        SpeedRef = -0.3;
    }
    else if (LV6_Ticker < 150000)
    {
        SpeedRef = 0.3;
    }
    else if (LV6_Ticker < 200000)
    {
        SpeedRef = -0.3;
    }
    else if (LV6_Ticker > 250000)
    {
        SpeedRef = 0.1;
    }


    }
    else
    {
        ReloadCpuTimer1(); // Reload timer counter
    }
#endif // (BUILDLEVEL==LEVEL8)



// ------------------------------------------------------------------------------
//    Call the DATALOG update function.
// ------------------------------------------------------------------------------
    dlog.update(&dlog);

// Enable more interrupts from this timer
    EPwm1Regs.ETCLR.bit.INT = 1;

// Acknowledge interrupt to recieve more interrupts from PIE group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}// MainISR Ends Here




// OffsetMainISR
interrupt void OffsetISR(void)
{
// Verifying the ISR
    IsrTicker++;

// DC offset measurement for ADC

    if (IsrTicker>=5000)
    {

        #ifdef DSP2833x_DEVICE_H
            offsetA= K1*offsetA + K2*(AdcMirror.ADCRESULT1)*0.00024414;             //Phase U current offset
            offsetB= K1*offsetB + K2*(AdcMirror.ADCRESULT2)*0.00024414;             //Phase V current offset
            offsetC= K1*offsetC + K2*(AdcMirror.ADCRESULT3)*0.00024414;             //Phase W current offset
        #endif

        #ifdef DSP2803x_DEVICE_H
            offsetA= _IQmpy(K1,offsetA)+_IQmpy(K2,_IQ12toIQ(AdcResult.ADCRESULT1));         //Phase A offset
            offsetB= _IQmpy(K1,offsetB)+_IQmpy(K2,_IQ12toIQ(AdcResult.ADCRESULT2));         //Phase B offset
            offsetC= _IQmpy(K1,offsetC)+_IQmpy(K2,_IQ12toIQ(AdcResult.ADCRESULT3));         //Phase C offset
        #endif
    }

    if (IsrTicker > 20000)
    {
        EALLOW;
        PieVectTable.EPWM1_INT = &MainISR;
        EDIS;
    }


// Enable more interrupts from this timer
    EPwm1Regs.ETCLR.bit.INT = 1;

// Acknowledge interrupt to receive more interrupts from PIE group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}


union datatype sdata[3];
union datatype rdata;

Uint16 Tx[6] = {1,2,3,4,5,6};
Uint16 Rx[6] = {0,0,0,0,0,0};

Uint32 tx_test = 0;
char flag_int = 0;

interrupt void
spiTxFifoIsr(void)
{
    Uint16 i;
    Uint16 j;

    sdata[0].f_type = (float)speed1.SpeedRpm_fr*R2r;
    sdata[1].f_type = (float)park1.Qs*BASE_CURRENT;
    sdata[2].f_type = (float)park1.Ds*BASE_CURRENT;


    for(i=0;i<3;i++)
    {
        for(j=0;j<2;j++)
        {
            Tx[i*2+j] = sdata[i].u_type[1-j];
        }
    }

    for(i=0;i<6;i++)
    {
       SpiaRegs.SPITXBUF=Tx[i];      // Send data
    }

    SpiaRegs.SPIFFTX.bit.TXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ACK

    tx_test++;

}

interrupt void
spiRxFifoIsr(void)
{
    Uint16 i;
    rx_test++;


    for(i=0;i<6;i++)
    {
        Rx[i]=SpiaRegs.SPIRXBUF;     // Read data
    }

    rdata.u_type[0] = Rx[1];
    rdata.u_type[1] = Rx[0];
    received1.Vq = rdata.f_type;

    rdata.u_type[0] = Rx[3];
    rdata.u_type[1] = Rx[2];
    received1.Vd = rdata.f_type;

    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack
}

void HVDMC_Protection(void)
{

      EALLOW;

// Configure Trip Mechanism for the Motor control software
// -Cycle by cycle trip on CPU halt
// -One shot IPM trip zone trip
// These trips need to be repeated for EPWM1 ,2 & 3

//===========================================================================
//Motor Control Trip Config, EPwm1,2,3
//===========================================================================

// CPU Halt Trip
      EPwm1Regs.TZSEL.bit.CBC6=0x1;
      EPwm2Regs.TZSEL.bit.CBC6=0x1;
      EPwm3Regs.TZSEL.bit.CBC6=0x1;

      EPwm1Regs.TZSEL.bit.OSHT1 = 1;  //enable TZ1 for OSHT
      EPwm2Regs.TZSEL.bit.OSHT1 = 1;  //enable TZ1 for OSHT
      EPwm3Regs.TZSEL.bit.OSHT1 = 1;  //enable TZ1 for OSHT

// What do we want the OST/CBC events to do?
// TZA events can force EPWMxA
// TZB events can force EPWMxB

      EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
      EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low

      EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
      EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low

      EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
      EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low


      EDIS;

     // Clear any spurious OV trip
      EPwm1Regs.TZCLR.bit.OST = 1;
      EPwm2Regs.TZCLR.bit.OST = 1;
      EPwm3Regs.TZCLR.bit.OST = 1;
//************************** End of Prot. Conf. ***************************//
}

//
// spi_fifo_init
//
void
spi_fifo_init()
{
    SpiaRegs.SPICCR.bit.SPISWRESET=0; // Reset SPI

    SpiaRegs.SPICCR.all=0x004F; // 16-bit character, polarity = 1
    SpiaRegs.SPICTL.all=0x0013; // Interrupt enabled, Slave XMIT enabled, phase = 0
    SpiaRegs.SPISTS.all=0x0000;
    SpiaRegs.SPIBRR=0x0003;     // Baud rate

    SpiaRegs.SPIFFTX.all=0xC026;       // Enable FIFO's, set TX FIFO level to 6
    SpiaRegs.SPIFFRX.all=0x0026;       // Set RX FIFO level to 6

    SpiaRegs.SPIFFCT.all=0x0000;

    SpiaRegs.SPICCR.bit.SPISWRESET=1;  // Enable SPI

    SpiaRegs.SPIFFTX.bit.TXFIFO=1;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;
}


