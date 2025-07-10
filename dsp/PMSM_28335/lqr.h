/* =================================================================================
File name:       LQR.H
Created on: 2023/03/14
Author: smart
===================================================================================*/


#ifndef _LQR_H_
#define _LQR_H_

typedef struct  { float Tl;             // Data: estimated torque
                  float iqs_est;        // Data: estimated Q-axis current
                  float ids_est;        // Data: estimated D-axis current
                  float w_est;          // Data: estimated motor speed
                  float32 w;            // Input: motor speed (RPM)
                  float iqs;            // Input: real Q-axis current
                  float ids;            // Input: real D-axis current
                  float wd;             // Input: desired speed (RPM)
                  float delta_t;        // Input: time period
                  float Vq;             // Output: Q-axis voltage
                  float Vd;             // Output: D-axis voltage
                  float Tl_dot;       // Variable: differential of torque
                  float w_est_dot;    // Variable: differential of estimated motor speed
                  float iqs_est_dot;  // Variable: differential of estimated Q-axis current
                  float ids_est_dot;  // Variable: differential of estimated D-axis current
                  float iqsd;         // Variable: desired D-axis current
                  float idsd;         // Variable: desired Q-axis current
                  _iq DCbus;          // Variable: DC bus voltage feedback (pu)
                  float Vq_in;        // Input: Q-axis voltage
                  float Vd_in;        // Input: D-axis voltage
                  _iq Vq_out;         // Output: Q-axis voltage (pu)
                  _iq Vd_out;         // Output: D-axis voltage (pu)
                  float Tle;          // Data: estimated torque
                  float k1_Tl;        // Variable: Runge Kutta parameter
                  float k1_w;         // Variable: Runge Kutta parameter
                  float k1_iqs;       // Variable: Runge Kutta parameter
                  float k1_ids;       // Variable: Runge Kutta parameter
                  float k2_Tl;        // Variable: Runge Kutta parameter
                  float k2_w;         // Variable: Runge Kutta parameter
                  float k2_iqs;       // Variable: Runge Kutta parameter
                  float k2_ids;       // Variable: Runge Kutta parameter
                } LQR;


/*-----------------------------------------------------------------------------
Default initalizer for the SVGEN object.
2023/11/30 Change initial value of Tl, iqs_est, ids_est, w_est 0,0.008,0.008,0.001 to 0,0,0,0
-----------------------------------------------------------------------------*/
#define LQR_DEFAULTS { 0,0,0,0,0,0,0,0,0,  \
                       0,0,0,0,0,0,0,0,0,0,0,0,0,0,     \
                       0,0,0,0,0,0,0,0     \
                     }


// motor coefficients
#define J               0.000782        // Equivalent rotor inertia (kg*m^2)
#define Lambda          0.30175         // Magnet flux linkage (V*s/rad) (Ke?) (currently V)
#define B               0.02277         // Viscous friction coefficient (N*m*s/rad) (T/w, rated?)
#define Rs              2.5             // Stator Resistance (Ohm)
#define p               6.0             // Number of poles (x)
#define Ld              0.018           // d-axis inductances(mH)
#define Lq              0.032           // q-axis inductances(mH)



// Define motor constants
#define k1              (3.0/2.0/J*p*p/4.0*Lambda)
#define k2              (B/J)
#define k3              (p/2.0/J)
#define k4              (Rs/Lq)
#define k5              (Lambda/Lq)
#define k6              (1/Lq)
#define k7              (Rs/Ld)
#define k8              (1/Ld)
#define k9              (Lq/Ld)
#define k10             (Ld/Lq)
#define k11             (3.0/2.0/J*p*p/4.0*(Ld-Lq))
#define k12             ((Ld-Lq)/Lambda)


#define LIMITS(A,B)     (((A) > (B)) ? (B) : ((A) < -(B)) ? -(B) : (A))

/*  sda_care
Final u_obs matrix:
-83.9371826103988 54.2990076619623 -2.48337318081849
5025.67992883502 1707.62556279163 -76.8352318221911
1707.62556279163 2578.08556925333 23.1955837522952
-76.8352318221911 23.1955837522952 3025.37646717239

Final u_con matrix:
-30.5662784559950 -120.741847316761 1.61355293847622
1.76693447765969 2.86853855729106 -68.3450431051885
*/


/*  N = 0
Final u_obs matrix:
-83.9607235690765   54.3193970673205    1.05584373651630e-13
5023.44325307055    1708.83223537395    -8.51801553650411e-13
1708.83223537395    2577.76814525884    3.63835774457513e-13
-8.51801553650411e-13   3.63835774457513e-13    3026.43733813050

Final u_con matrix:
-30.6354883761756   -120.868778177730   4.66743359295407e-15
2.15148037852906e-16    8.29765972080723e-15    -68.2548584904246
*/

/* N = 1
Final u_con matrix:
D:
-0.00283004010950613 0.000709690302321033 1.10760334833458e-19
-1.04642190125303e-18 1.96907261926147e-19 1.99100650789766e-35

Q:
6.47835098843518e-21 -1.62457869628088e-21 0.669034182428763
4.03638677641073 1.18939410209558 1.38529451332876e-16

Final u_obs matrix:
D:
-3.49401090742399e-16 0.167902011362889 -2.28893719198317e-16
0.0665264012222053 -0.195621336402209 -5.29484814437444e-18
-0.195621336402209 -0.623177459582425 8.85404678101986e-17
-5.29484814437444e-18 8.85404678101986e-17 -3.96696854521471e-33

Q:
-1.95000406869208e-28 -1.68925988018843e-14 93.5026604978133
2.82089420455221e-14 -1.40003351475086e-15 -77.1874890984327
-1.40003351475086e-15 4.90669213593721e-15 -46.7648933688062
-77.1874890984327 -46.7648933688062 -1.89934342765508e-14
*/

/* N = 2
Final u_con matrix:
D:
-3.22544016034279e-07 8.29257728298760e-08 9.60930320411975e-24
-6.98589857352727e-23 1.70832056962129e-23 1.87767360180053e-39
Q:
-0.0471807205732386 0.0175649819686312 1.42518223692091e-18
2.91258734460848e-18 2.53365731008162e-18 -0.245015366641986
DQ:
2.47734408714929e-20 -4.31407905998490e-21 0.000645987740355297
-0.00796386932557339 0.00114842264952053 1.75850096076407e-19

Final u_obs matrix:
D:
-3.67423813303473e-06 4.23602212350519e-05 -1.65345987182452e-20
-1.56686801796868e-06 -7.99559722749719e-06 -1.02525354779707e-22
-7.99559722749719e-06 -5.02884502519483e-05 1.11096581533706e-20
-1.02525354779709e-22 1.11096581533706e-20 -1.99579808234865e-36
Q:
-1.13947142679541 -4.91054359039742 -4.40081928336167e-15
2.51491713781551 1.67255711597169 5.49293345931882e-16
1.67255711597169 1.89310726273426 5.37815067127157e-16
5.49293345931882e-16 5.37815067127157e-16 -1.32992739562389
DQ:
6.31816197041036e-18 8.67951583651988e-19 0.00290746061375340
7.45505459020472e-19 -2.80268531002615e-18 -0.000352197597123911
-2.80268531002615e-18 -7.47594397979969e-19 -0.000440272407902410
-0.000352197597123911 -0.000440272407902410 1.03486281934440e-18
*/



/* 1500*0.314159265358979 = 471.2388980384685 */

/*------------------------------------------------------------------------------
    Calculate Vd Vq (CALVDQ) Macro Definition
------------------------------------------------------------------------------*/
#define CALVDQ_MACRO(v)                                                                                                         \
    /* ///////////////////////////////// Observer ////////////////////////////////////////////////////////////////////// */     \
                                                                                                                                \
    v.Tl = v.Tl_dot*v.delta_t + v.Tl;                                                                                           \
    v.w_est = v.w_est_dot*v.delta_t + v.w_est;                                                                                  \
    v.iqs_est = v.iqs_est_dot*v.delta_t + v.iqs_est;                                                                            \
    v.ids_est = v.ids_est_dot*v.delta_t + v.ids_est;                                                                            \
                                                                                                                                \
    v.Tl = LIMITS(v.Tl, 4.8);                                                                                                   \
    v.w_est = LIMITS(v.w_est, 471.2388980384685);                                                                               \
    v.iqs_est = LIMITS(v.iqs_est, 4.8);                                                                                         \
    v.ids_est = LIMITS(v.ids_est, 4.8);                                                                                         \
                                                                                                                                \
/*    v.Tl_dot = -83.9371826103988 * (v.w - v.w_est) + 54.2990076619623 * (v.iqs - v.iqs_est) +                          */         \
/*                 (-2.48337318081849) * (v.ids - v.ids_est);                                                            */         \
/*    v.w_est_dot = 5025.67992883502 * (v.w - v.w_est) + 1707.62556279163 * (v.iqs - v.iqs_est) +                        */         \
/*                  (-76.8352318221911) * (v.ids - v.ids_est) + k1 * v.iqs_est -k2*v.w_est - k3*v.Tl +                   */         \
/*                  k11*v.iqs_est*v.ids_est;                                                                             */         \
/*    v.iqs_est_dot = 1707.62556279163 * (v.w - v.w_est) + 2578.08556925333 * (v.iqs - v.iqs_est) +                      */         \
/*                    23.1955837522952 * (v.ids - v.ids_est) - k4*v.iqs_est - k5*v.w_est + k6*v.Vq_in -                  */         \
/*                    k10*v.w_est*v.ids_est;                                                                             */         \
/*    v.ids_est_dot = -76.8352318221911 * (v.w - v.w_est) + 23.1955837522952 * (v.iqs - v.iqs_est) +                     */         \
/*                    3025.37646717239 * (v.ids - v.ids_est) - k7*v.ids_est + k9*v.w_est*v.iqs_est + k8*v.Vd_in;         */         \
                                                                                                                                \
    v.Tl_dot = -83.9371826103988 * (v.w - v.w_est) + 54.2990076619623 * (v.iqs - v.iqs_est) +                                   \
                (-2.48337318081849) * (v.ids - v.ids_est) +                                                                     \
                (-1.95000406869208e-28 * (v.w - v.w_est) + (-1.68925988018843e-14) * (v.iqs - v.iqs_est) +                      \
                (93.5026604978133) * (v.ids - v.ids_est)) * v.iqs_est +                                                         \
                (-3.49401090742399e-16 * (v.w - v.w_est) + 0.167902011362889 * (v.iqs - v.iqs_est) +                            \
                (-2.28893719198317e-16) * (v.ids - v.ids_est)) * v.ids_est;                                                     \
    v.w_est_dot = 5025.67992883502 * (v.w - v.w_est) + 1707.62556279163 * (v.iqs - v.iqs_est) +                                 \
                  (-76.8352318221911) * (v.ids - v.ids_est) +                                                                   \
                  (2.82089420455221e-14 * (v.w - v.w_est) + (-1.40003351475086e-15) * (v.iqs - v.iqs_est) +                     \
                  (-77.1874890984327) * (v.ids - v.ids_est)) * v.iqs_est +                                                      \
                  (0.0665264012222053 * (v.w - v.w_est) + (-0.195621336402209) * (v.iqs - v.iqs_est) +                          \
                  (-5.29484814437444e-18) * (v.ids - v.ids_est)) * v.ids_est +                                                  \
                  k1 * v.iqs_est -k2*v.w_est - k3*v.Tl + k11*v.iqs_est*v.ids_est;                                               \
    v.iqs_est_dot = 1707.62556279163 * (v.w - v.w_est) + 2578.08556925333 * (v.iqs - v.iqs_est) +                               \
                    23.1955837522952 * (v.ids - v.ids_est) +                                                                    \
                    (-1.40003351475086e-15 * (v.w - v.w_est) + (4.90669213593721e-15) * (v.iqs - v.iqs_est) +                   \
                    (-46.7648933688062) * (v.ids - v.ids_est)) * v.iqs_est +                                                    \
                    (-0.195621336402209 * (v.w - v.w_est) + (-0.623177459582425) * (v.iqs - v.iqs_est) +                        \
                    (8.85404678101986e-17) * (v.ids - v.ids_est)) * v.ids_est -                                                 \
                    k4*v.iqs_est - k5*v.w_est + k6*v.Vq_in - k10*v.w_est*v.ids_est;                                             \
    v.ids_est_dot = -76.8352318221911 * (v.w - v.w_est) + 23.1955837522952 * (v.iqs - v.iqs_est) +                              \
                    3025.37646717239 * (v.ids - v.ids_est) +                                                                    \
                    (-77.1874890984327 * (v.w - v.w_est) + (-46.7648933688062) * (v.iqs - v.iqs_est) +                          \
                    (-1.89934342765508e-14) * (v.ids - v.ids_est)) * v.iqs_est +                                                \
                    (-5.29484814437444e-18 * (v.w - v.w_est) + (8.85404678101986e-17) * (v.iqs - v.iqs_est) +                   \
                    (-3.96696854521471e-33) * (v.ids - v.ids_est)) * v.ids_est -                                                \
                    k7*v.ids_est + k9*v.w_est*v.iqs_est + k8*v.Vd_in;                                                           \
    v.Tle = v.Tl;                                                                                                               \
    /*v.Tle = 0;*/                                                                                                              \
    /* ///////////////////////////////// Contoller ////////////////////////////////////////////////////////////////////// */    \
    v.idsd = k12*v.iqs*v.iqs;                                                                                                   \
    v.iqsd = (k2*v.wd + k3*v.Tle - k11*v.idsd*v.iqs) / (k1 + k11*(v.ids-v.idsd));                                               \
                                                                                                                                \
    /*v.Vq = (k4*v.iqsd + k5*v.wd + k10*(v.ids - v.idsd)*v.wd + k10*v.w*v.idsd)*Lq +                                        */      \
    /*       (-30.5662784559950 * (v.w-v.wd) + (-120.741847316761) * (v.iqs-v.iqsd) + 1.61355293847622 * (v.ids-v.idsd));   */      \
    /*v.Vd = (k7*v.idsd - k9*(v.iqs - v.iqsd)*v.wd - k9*v.w*v.iqsd)* Ld +                                                 */        \
    /*       (1.76693447765969 * (v.w-v.wd) + 2.86853855729106 * (v.iqs-v.iqsd) + (-68.3450431051885) * (v.ids-v.idsd));*/          \
                                                                                                                                \
    v.Vq = (k4*v.iqsd + k5*v.wd + k10*(v.ids - v.idsd)*v.wd + k10*v.w*v.idsd)*Lq +                                              \
           (-30.5662784559950 * (v.w-v.wd) + (-120.741847316761) * (v.iqs-v.iqsd) + 1.61355293847622 * (v.ids-v.idsd)) +        \
           (-0.00283004010950613 * (v.w-v.wd) + 0.000709690302321033 * (v.iqs-v.iqsd) +                                         \
            1.10760334833458e-19 * (v.ids-v.idsd)) * (v.ids-v.idsd) +                                                           \
           (6.47835098843518e-21 * (v.w-v.wd) + (-1.62457869628088e-21) * (v.iqs-v.iqsd) +                                      \
            0.669034182428763 * (v.ids-v.idsd)) * (v.iqs-v.iqsd);                                                               \
                                                                                                                                \
    v.Vd = (k7*v.idsd - k9 * (v.iqs - v.iqsd) * v.wd - k9 * v.w * v.iqsd) * Ld +                                                \
           (1.76693447765969 * (v.w-v.wd) + 2.86853855729106 * (v.iqs-v.iqsd) + (-68.3450431051885) * (v.ids-v.idsd));          \
           (-1.04642190125303e-18 * (v.w-v.wd) + 1.96907261926147e-19 * (v.iqs-v.iqsd) +                                        \
            1.99100650789766e-35 * (v.ids-v.idsd)) * (v.ids-v.idsd) +                                                           \
           (4.03638677641073 * (v.w-v.wd) + 1.18939410209558 * (v.iqs-v.iqsd) +                                                 \
            1.38529451332876e-16 * (v.ids-v.idsd)) * (v.iqs-v.iqsd);                                                            \
                                                                                                                                \
    v.Vq = LIMITS(v.Vq, BASE_VOLTAGE*0.3);                                                                                      \
    v.Vd = LIMITS(v.Vd, BASE_VOLTAGE*0.1);                                                                                      \
    v.Vq_in = v.Vq;                                                                                                             \
    v.Vd_in = v.Vd;                                                                                                             \
    v.Vq_out = _IQ(v.Vq / BASE_VOLTAGE);                                                                                        \
    v.Vd_out = _IQ(v.Vd / BASE_VOLTAGE);


/*------------------------------------------------------------------------------
    Remove observer and set Tl=0
------------------------------------------------------------------------------*/
//#define CALVDQ_MACRO(v)                                                                                                         \
//                                                                                                                                \
//    v.Tl = 0;                                                                                                                   \
//                                                                                                                                \
//    v.idsd = k12*v.iqs*v.iqs;                                                                                                   \
//    v.iqsd = (k2*v.wd + k3*v.Tl - k11*v.idsd*v.iqs) / (k1 + k11*(v.ids-v.idsd));                                              \
//                                                                                                                                \
//    v.Vq = (k4*v.iqsd + k5*v.wd + k10*(v.ids - v.idsd)*v.wd + k10*v.w*v.idsd)*Lq +                                              \
//           (-30.5662784559950 * (v.w-v.wd) + (-120.741847316761) * (v.iqs-v.iqsd) + 1.61355293847622 * (v.ids-v.idsd));         \
//                                                                                                                                \
//    v.Vd = (k7*v.idsd - k9*(v.iqs - v.iqsd)*v.wd - k9*v.w*v.iqsd)* Ld +                                                         \
//           (1.76693447765969 * (v.w-v.wd) + 2.86853855729106 * (v.iqs-v.iqsd) + (-68.3450431051885) * (v.ids-v.idsd));          \
//                                                                                                                                \
//    v.Vq = (v.Vq > BASE_VOLTAGE*0.3) ? BASE_VOLTAGE*0.3 : (v.Vq < -BASE_VOLTAGE*0.3) ? -BASE_VOLTAGE*0.3 : v.Vq;                \
//    v.Vd = (v.Vd > BASE_VOLTAGE*0.1) ? BASE_VOLTAGE*0.1 : (v.Vd < -BASE_VOLTAGE*0.1) ? -BASE_VOLTAGE*0.1 : v.Vd;                \
//    v.Vq_out = _IQ(v.Vq / BASE_VOLTAGE);                                                                                        \
//    v.Vd_out = _IQ(v.Vd / BASE_VOLTAGE);                                                                                        \



/*--------------------------------------------------------------------------------------------
    Calculate observer state with Second-order Runge Kutta method (Modified Euler Method)
--------------------------------------------------------------------------------------------*/
//#define CALVDQ_MACRO(v)                                                                                                     \
//    /* ///////////////////////////////// Observer ////////////////////////////////////////////////////////////////////// */ \
//                                                                                                                            \
//    v.k1_Tl = -83.9371826103988 * (v.w - v.w_est) + 54.2990076619623 * (v.iqs - v.iqs_est) +                                \
//                 (-2.48337318081849) * (v.ids - v.ids_est);                                                                 \
//    v.k1_w = 5025.67992883502 * (v.w - v.w_est) + 1707.62556279163 * (v.iqs - v.iqs_est) +                                  \
//                  (-76.8352318221911) * (v.ids - v.ids_est) + k1 * v.iqs_est -k2*v.w_est - k3*v.Tl +                        \
//                  k11*v.iqs_est*v.ids_est;                                                                                  \
//    v.k1_iqs = 1707.62556279163 * (v.w - v.w_est) + 2578.08556925333 * (v.iqs - v.iqs_est) +                                \
//                    23.1955837522952 * (v.ids - v.ids_est) - k4*v.iqs_est - k5*v.w_est + k6*v.Vq_in -                       \
//                    k10*v.w_est*v.ids_est;                                                                                  \
//    v.k1_ids = -76.8352318221911 * (v.w - v.w_est) + 23.1955837522952 * (v.iqs - v.iqs_est) +                               \
//                    3025.37646717239 * (v.ids - v.ids_est) - k7*v.ids_est + k9*v.w_est*v.iqs_est + k8*v.Vd_in;              \
//                                                                                                                            \
//    v.k1_Tl = v.k1_Tl*v.delta_t;                                                                                            \
//    v.k1_w = v.k1_w*v.delta_t;                                                                                              \
//    v.k1_iqs = v.k1_iqs*v.delta_t;                                                                                          \
//    v.k1_ids = v.k1_ids*v.delta_t;                                                                                          \
//                                                                                                                            \
//    v.k2_Tl = -83.9371826103988 * (v.w - (v.w_est+v.k1_w)) + 54.2990076619623 * (v.iqs - (v.iqs_est+v.k1_iqs)) +            \
//                 (-2.48337318081849) * (v.ids - (v.ids_est+v.k1_ids));                                                      \
//    v.k2_w = 5025.67992883502 * (v.w - (v.w_est+v.k1_w)) + 1707.62556279163 * (v.iqs - (v.iqs_est+v.k1_iqs)) +              \
//                  (-76.8352318221911) * (v.ids - (v.ids_est+v.k1_ids)) + k1 * (v.iqs_est+v.k1_iqs) +                        \
//                  (-k2*(v.w_est+v.k1_w)) - k3*(v.Tl+v.k1_Tl) + k11*(v.iqs_est+v.k1_iqs)*(v.ids_est+v.k1_ids);               \
//    v.k2_iqs = 1707.62556279163 * (v.w - (v.w_est+v.k1_w)) + 2578.08556925333 * (v.iqs - (v.iqs_est+v.k1_iqs)) +            \
//                    23.1955837522952 * (v.ids - (v.ids_est+v.k1_ids)) - k4*(v.iqs_est+v.k1_iqs) +                           \
//                    (-k5*(v.w_est+v.k1_w)) + k6*v.Vq_in - k10*(v.w_est+v.k1_w)*(v.ids_est+v.k1_ids);                        \
//    v.k2_ids = -76.8352318221911 * (v.w - (v.w_est+v.k1_w)) + 23.1955837522952 * (v.iqs - (v.iqs_est+v.k1_iqs)) +           \
//                    3025.37646717239 * (v.ids - (v.ids_est+v.k1_ids)) - k7*(v.ids_est+v.k1_ids) +                           \
//                    k9*(v.w_est+v.k1_w)*(v.iqs_est+v.k1_iqs) + k8*v.Vd_in;                                                  \
//                                                                                                                            \
//    v.k2_Tl = v.k2_Tl*v.delta_t;                                                                                            \
//    v.k2_w = v.k2_w*v.delta_t;                                                                                              \
//    v.k2_iqs = v.k2_iqs*v.delta_t;                                                                                          \
//    v.k2_ids = v.k2_ids*v.delta_t;                                                                                          \
//                                                                                                                            \
//    v.Tl = (v.k1_Tl + v.k2_Tl)*0.5 + v.Tl;                                                                                  \
//    v.w_est = (v.k1_w + v.k2_w)*0.5 + v.w_est;                                                                              \
//    v.iqs_est = (v.k1_iqs + v.k2_iqs)*0.5 + v.iqs_est;                                                                      \
//    v.ids_est = (v.k1_ids + v.k2_ids)*0.5 + v.ids_est;                                                                      \
//                                                                                                                            \
//    v.Tle = v.Tl;                                                                                                           \
//    /* ///////////////////////////////// Contoller /////////////////////////////////////////////////////////// */           \
//    v.idsd = k12*v.iqs*v.iqs;                                                                                               \
//    v.iqsd = (k2*v.wd + k3*v.Tle - k11*v.idsd*v.iqs) / (k1 + k11*(v.ids-v.idsd));                                           \
//                                                                                                                            \
//    v.Vq = (k4*v.iqsd + k5*v.wd + k10*(v.ids - v.idsd)*v.wd + k10*v.w*v.idsd)*Lq +                                          \
//    (-30.5662784559950 * (v.w-v.wd) + (-120.741847316761) * (v.iqs-v.iqsd) + 1.61355293847622 * (v.ids-v.idsd));            \
//                                                                                                                            \
//    v.Vd = (k7*v.idsd - k9*(v.iqs - v.iqsd)*v.wd - k9*v.w*v.iqsd)* Ld +                                                     \
//    (1.76693447765969 * (v.w-v.wd) + 2.86853855729106 * (v.iqs-v.iqsd) + (-68.3450431051885) * (v.ids-v.idsd));             \
//                                                                                                                            \
//    v.Vq = LIMITS(v.Vq, BASE_VOLTAGE*0.3);                                                                                  \
//    v.Vd = LIMITS(v.Vd, BASE_VOLTAGE*0.1);                                                                                  \
//    v.Vq_in = v.Vq;                                                                                                         \
//    v.Vd_in = v.Vd;                                                                                                         \
//    v.Vq_out = _IQ(v.Vq / BASE_VOLTAGE);                                                                                    \
//    v.Vd_out = _IQ(v.Vd / BASE_VOLTAGE);                                                                                    \

#endif // _LQR_H_
