/* =================================================================================
File name:  HVPM_Sensored-Settings.H                     
                 
Description: Incremental Build Level control file.
=================================================================================  */
#ifndef PROJ_SETTINGS_H

/*------------------------------------------------------------------------------
Following is the list of the Build Level choices.
------------------------------------------------------------------------------*/
#define LEVEL1  1      		// Module check out (do not connect the motors) 
#define LEVEL2  2           // Verify ADC, park/clarke, calibrate the offset 
#define LEVEL3  3           // Verify closed current(torque) loop, QEP and speed meas.
#define LEVEL4  4           // Verify close speed loop and speed PID
#define LEVEL5  5           // Verify SDRE_fpga
#define LEVEL6  6           // Verify LQR
#define LEVEL7  7           // Verify PK SDRE_fpga
#define LEVEL8  8           // Verify SDRE_DSP

/*------------------------------------------------------------------------------
This line sets the BUILDLEVEL to one of the available choices.
------------------------------------------------------------------------------*/
#define   BUILDLEVEL LEVEL5



#ifndef BUILDLEVEL    
#error  Critical: BUILDLEVEL must be defined !!
#endif  // BUILDLEVEL
//------------------------------------------------------------------------------


#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define PI 3.14159265358979

// Define the system frequency (MHz)
#if (DSP2833x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 150
#elif (DSP2803x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 60
#endif

// user added
#if (DSP2803x_DEVICE_H==1)
#define MATH_TYPE 0
#elif (DSP2833x_DEVICE_H==1)
#define MATH_TYPE 1
#endif


// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 10

// Define the electrical motor parameters (Estun Servomotor)
//#define RS 		2.35		    	    // Stator resistance (ohm)
//#define RR   			               	    // Rotor resistance (ohm)
//#define LS   	0.0065					    // Stator inductance (H)
//#define LR   			  				    // Rotor inductance (H)
//#define LM   			   				    // Magnatizing inductance (H)
//#define POLES  	8						// Number of poles

// Define the base quantities
#define BASE_DCBUS    409.0                   // Base peak DC bus voltage (volt)
#define BASE_VOLTAGE    236.14             // Base peak phase voltage (volt): 410/sqrt(3)
//#define BASE_VOLTAGE    236.713             // Base peak phase voltage (volt): 410/sqrt(3)
#define BASE_CURRENT    10.0                  // Base peak phase current (amp)
#define BASE_TORQUE     		            // Base torque (N.m)
#define BASE_FLUX       		            // Base flux linkage (volt.sec/rad)
#define BASE_FREQ      	60.0                  // Base electrical frequency (Hz)



// Define my IPMSM parameters
//#define RS      2.35                    // Stator resistance (ohm)
//#define RR                              // Rotor resistance (ohm)
//#define LS      0.0065                  // Stator inductance (H)
//#define LR                              // Rotor inductance (H)
//#define LM                              // Magnatizing inductance (H)
#define POLES   6                         // Number of poles
#define R2r     0.314159265358979         // RPM (electrical angle) to rad/s (mechanical angle): 2*PI/60*3

// Define my base quantities
//#define BASE_VOLTAGE    140             // Base peak phase voltage (volt)
//#define BASE_CURRENT    10                // Base peak phase current (amp)
//#define BASE_TORQUE     4.77            // Base torque (N.m)
//#define BASE_FLUX       63.2            // Base flux linkage (volt.sec/rad)
//#define BASE_FREQ       60              // Base electrical frequency (Hz)



//// motor coefficients
//#define J               0.000782        // Equivalent rotor inertia (kg*m^2)
//#define Lambda          63.2            // Magnet flux linkage (V*s/rad) (Ke?) (currently V)
//#define B               0.0024          // Viscous friction coefficient (N*m*s/rad) (T/w, rated?)
//#define Rs              2.5             // Stator Resistance (Ohm)
//#define p               6               // Number of poles (x)
//#define Ld              18              // d-axis inductances(mH)
//#define Lq              32              // q-axis inductances(mH)
//
//
//
//// define motor constants
//#define k1              3/2/J*p*p/4*Lambda
//#define k2              B/J
//#define k3              p/2/J
//#define k4              Rs/Lq
//#define k5              Lambda/Lq
//#define k6              1/Lq
//#define k7              Rs/Lq
//#define k8              1/Ld
//#define k9              Lq/Ld
//#define k10             Ld/Lq
//#define k11             3/2/J*p*p/4*(Ld-Lq)
//#define k12             (Ld-Lq)/Lambda
//


#endif

