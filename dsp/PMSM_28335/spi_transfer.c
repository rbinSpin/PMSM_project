//
// Included Files
//

#include "spi_setting.h"


void spi_send(float w, float iqs, float ids)
{
    union datatype sdata;
    float data[3] = {w, iqs, ids};
    sdata.f_type = data[0];
    char data_switch = 0;
    int count = 0;
    int i = 0;

    while(count < 3)
    {
        spi_xmit(sdata.u_type[data_switch]);

        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1){}

        unsigned int sRx = SpiaRegs.SPIRXBUF;

        if(data_switch==0)
        {
            data_switch=1;
        }
        else
        {
            count ++;
            sdata.f_type = data[count];
            data_switch=0;
        }
        for (i = 0; i < 1500; i++){};
    }

}


char spi_check(char flag)
{
    unsigned int Rx = 0;
    int i = 0;

    spi_xmit(65535);

    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1){}

    Rx = SpiaRegs.SPIRXBUF;

    if(Rx != 65535)
    {
        flag = 1;
    }

//    for (i = 0; i < 1500; i++){};

    return flag;
}


unsigned int f[6] = {0,0,0,0,0,0};
RECEIVED spi_receive(void)
{
    RECEIVED received;
    union datatype rdata;
    unsigned int Rx = 0;
    long i = 0;
    int k = 0;
//    unsigned int f[6] = {0, 0, 0, 0, 0, 0};

//    for (i = 0; i < 1500; i++){};

//    while(k < 4)
//    {
//        spi_xmit(65535);
//
//        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1){}
//
//        Rx = SpiaRegs.SPIRXBUF;
//
//        if(Rx != 0)
//        {
//            f[k] = Rx;
//            k = k + 1;
//        }
//    }

    spi_xmit(65535);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1){}
    f[0] = SpiaRegs.SPIRXBUF;

    spi_xmit(65535);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1){}
    f[1] = SpiaRegs.SPIRXBUF;

    spi_xmit(65535);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1){}
    f[2] = SpiaRegs.SPIRXBUF;

    spi_xmit(65535);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1){}
    f[3] = SpiaRegs.SPIRXBUF;



    rdata.u_type[0] = f[0];
    rdata.u_type[1] = f[1];
    received.Vq = rdata.f_type;
    rdata.u_type[0] = f[2];
    rdata.u_type[1] = f[3];
    received.Vd = rdata.f_type;

    return received;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
float data[6] = {0, 0, 0, 0, 0, 0};
RECEIVED transmit(float w, float iqs, float ids)
{
    int i;
    unsigned int Rx;
    RECEIVED received;
    union datatype rdata;
    union datatype sdata;
    float f[6];

    sdata.f_type = w;
    f[0] = sdata.u_type[0];
    f[1] = sdata.u_type[1];
    sdata.f_type = iqs;
    f[2] = sdata.u_type[0];
    f[3] = sdata.u_type[1];
    sdata.f_type = ids;
    f[4] = sdata.u_type[0];
    f[5] = sdata.u_type[1];

    for(i = 0; i < 6; i++)
    {
        spi_xmit(f[i]);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1){}
        Rx = SpiaRegs.SPIRXBUF;
        data[i] = Rx;
    }
    SpiaRegs.SPICTL.bit.TALK = 0;           // disable data transfer

    rdata.u_type[0] = data[0];
    rdata.u_type[1] = data[1];
    received.Vq = rdata.f_type;
    rdata.u_type[0] = data[2];
    rdata.u_type[1] = data[3];
    received.Vd = rdata.f_type;

    return received;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
char check2(char Rx_flag)
{
    unsigned int Rx;

    spi_xmit(1);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1){}
    Rx = SpiaRegs.SPIRXBUF;

    if (Rx == 0)
    {
        Rx_flag = 0;
        SpiaRegs.SPICTL.bit.TALK = 1;           // enable data transfer
        spi_xmit(1);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1){}
        Rx = SpiaRegs.SPIRXBUF;
    }

    return Rx_flag;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void pi_data(float w, float iqs, float ids)
{
    int i;
    unsigned int Rx;
    union datatype sdata;
    Uint16 f[7];

    f[0] = 1;
    sdata.f_type = w;
    f[1] = sdata.u_type[0];
    f[2] = sdata.u_type[1];
    sdata.f_type = iqs;
    f[3] = sdata.u_type[0];
    f[4] = sdata.u_type[1];
    sdata.f_type = ids;
    f[5] = sdata.u_type[0];
    f[6] = sdata.u_type[1];

    SpiaRegs.SPIFFTX.bit.TXFIFO = 1;        // release Tx FIFO
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;   // release Rx FIFO
    SpiaRegs.SPICTL.bit.TALK = 1;           // enable data transfer

    for(i = 0; i < 7; i++)
    {
        spi_xmit(f[i]);
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1){}
        Rx = SpiaRegs.SPIRXBUF;
    }

    SpiaRegs.SPICTL.bit.TALK = 0;         // disable data transfer
    SpiaRegs.SPIFFTX.bit.TXFIFO = 0;      // clear Tx FIFO
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0; // clear Rx FIFO
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void pi_data2(float w, float iqs, float ids)
{
    int i;
    unsigned int Rx;
    union datatype sdata;
    Uint16 f[7];

    f[0] = 1;
    sdata.f_type = w;
    f[1] = sdata.u_type[0];
    f[2] = sdata.u_type[1];
    sdata.f_type = iqs;
    f[3] = sdata.u_type[0];
    f[4] = sdata.u_type[1];
    sdata.f_type = ids;
    f[5] = sdata.u_type[0];
    f[6] = sdata.u_type[1];

    SpiaRegs.SPIFFTX.bit.TXFIFO = 1;        // release Tx FIFO
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;   // release Rx FIFO
    SpiaRegs.SPICTL.bit.TALK = 1;           // enable data transfer

    spi_xmit(f[0]);
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1){}
    Rx = SpiaRegs.SPIRXBUF;

    for(i = 1; i < 7; i++)
    {
        spi_xmit(f[i]);
    }
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=6){}

    for(i = 0; i < 6; i++)
    {
        Rx = SpiaRegs.SPIRXBUF;
    }

    SpiaRegs.SPICTL.bit.TALK = 0;         // disable data transfer
    SpiaRegs.SPIFFTX.bit.TXFIFO = 0;      // clear Tx FIFO
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0; // clear Rx FIFO
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

Uint16 TEMP = 0;
PK_RECEIVED pk_transmit(float w, float iqs, float ids)
{
    int k;
    Uint16 num_rx;     // Number of data in RX buffer
    Uint16 rx_in = 0;  // Raw input from RX buffer
    int num_din = 2;   // [Vq, Vd]
    int num_dout = 3;  // [w, iqs, ids]
    PK_RECEIVED received;
    union datatype rdata[2];
    union datatype tdata[3];

    Uint16 SIGNAL_DUMMY = 65535;
    Uint16 SIGNAL_RECV = 65535;
    Uint16 SIGNAL_SPEC = 65534;

//    tdata[0].u_type[0] = 65535;
//    tdata[0].u_type[1] = 16396;
    tdata[0].f_type = w;  //w
    tdata[1].f_type = iqs;  //iqs
    tdata[2].f_type = ids;  //ids

    //  Send to PYNQ-Z2
    for(k=0;k<num_dout;k++){
        int j = 0;

        if (tdata[k].u_type[j] == SIGNAL_DUMMY){
            tdata[k].u_type[j] = SIGNAL_SPEC;
        }

        while(1){
            num_rx = SpiaRegs.SPIFFRX.bit.RXFFST;
            if(num_rx != 0){
                rx_in = SpiaRegs.SPIRXBUF;
                spi_xmit(tdata[k].u_type[j]);
                j++;
                if(j==2){
                    break;
                }
            }
        }
    }

    //  Read From PYNQ-Z2
    for(k=0;k<num_din;k++){
        int j = 0;
        while(1){
            num_rx = SpiaRegs.SPIFFRX.bit.RXFFST;
            if(num_rx != 0){
                rx_in = SpiaRegs.SPIRXBUF;
                if (rx_in != SIGNAL_DUMMY){
                    if (rx_in == SIGNAL_SPEC){
                        rdata[k].u_type[j] = SIGNAL_DUMMY;
                    }else{
                        rdata[k].u_type[j] = rx_in;
                    }
                    spi_xmit(SIGNAL_RECV);
                    j++;
                    if(j==2){
                        break;
                    }
                }
            }
        }
    }


    //  Send to PYNQ-Z2 (check whether F28335 received data correctly)
    for(k=0;k<num_din;k++){
        int j = 0;
        if (rdata[k].u_type[j] == SIGNAL_DUMMY){
            rdata[k].u_type[j] = SIGNAL_SPEC;
        }
        while(1){
            num_rx = SpiaRegs.SPIFFRX.bit.RXFFST;
            if(num_rx != 0){
                rx_in = SpiaRegs.SPIRXBUF;
                spi_xmit(rdata[k].u_type[j]);
                j++;
                if(j==2){
                    break;
                }
            }
        }
    }

    received.Vq = rdata[0].f_type;
    received.Vd = rdata[1].f_type;




    return received;
}


//
// spi_xmit -
//
void
spi_xmit(Uint16 a)
{
    SpiaRegs.SPITXBUF=a;
}

//
// End of File
//
