/*
 * spi_setting.h
 *
 *  Created on: 2022/11/06
 *      Author: smart
 */


#include "PeripheralHeaderIncludes.h"
#include "IQmathLib.h"


#ifndef SPI_SETTING_H_
#define SPI_SETTING_H_


typedef struct{ float Vq;
                float Vd;
              } RECEIVED;

typedef struct{ float Vq;
                float Vd;
                float Test;
            }  PK_RECEIVED;



#define SPI_DEFAULTS {   0,                     \
                         0,                     \
}


void spi_xmit(Uint16);
void spi_send(float, float, float);
char spi_check(char);
RECEIVED spi_receive(void);
char check2(char Rx_flag);
RECEIVED transmit(float w, float iqs, float ids);
PK_RECEIVED pk_transmit(float w, float iqs, float ids);
void pi_data(float w, float iqs, float ids);
void pi_data2(float w, float iqs, float ids);

union datatype{
    float f_type;
    Uint16 u_type[2];
};


#define SPI_MACRO(v)                                                \
                                                                    \
    v.Vq = (v.Vq > 0.3) ? 0.3 : (v.Vq < -0.3) ? -0.3 : v.Vq;        \
    v.Vd = (v.Vd > 0.1) ? 0.1 : (v.Vd < -0.1) ? -0.1 : v.Vd;        \
    /* v.Vq = v.Vq / BASE_VOLTAGE;  */                              \
    /* v.Vd = v.Vd / BASE_VOLTAGE;  */                              \

#endif /* SPI_SETTING_H_ */
