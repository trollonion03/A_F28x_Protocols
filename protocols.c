//#############################################################################
// A_F28xProtocols by Trollonion03
//
// FILE:   protocols.c
// TARGET: TMS320F280049C (LAUNCHXL-F280049C)
// LANGUAGE: C
// COMPILER: TI 21.6.0.LTS
// DEBUGGER: XDS110(On-board-USB-Debugger)
// REFERENCE: C2000Ware_3_04_00_00
//#############################################################################


#include "driverlib.h"
#include "device.h"


uint16_t RxReadyFlag = 0;
uint16_t RxCopyCount = 0;
extern uint16_t receivedChar[16];

void sendDataSCI(uint32_t SelSCI, uint16_t * TrsData, SCI_TxFIFOLevel size) {
    uint16_t i, rACK[16];


    for (i=0;i<size;i++) {
        SCI_writeCharBlockingNonFIFO(SelSCI, TrsData[i]);
    }

    //ADD check ACK
    SCI_resetChannels(SelSCI);
    SCI_disableInterrupt(SelSCI, SCI_INT_TXRDY | SCI_INT_RXFF);
    SCI_clearInterruptStatus(SelSCI, SCI_INT_TXRDY | SCI_INT_RXRDY_BRKDT);
    SCI_enableFIFO(SelSCI);
    SCI_enableModule(SelSCI);
    SCI_performSoftwareReset(SelSCI);

    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX0 , SCI_FIFO_RX16);
    SCI_enableInterrupt(SCIA_BASE, SCI_INT_TXRDY | SCI_INT_RXFF);

    while(RxReadyFlag == 0);
    memcpy(rACK, receivedChar, SCI_FIFO_RX16*2);
    if(rACK[1] != 8) {
        ESTOP0;
    }
}

void rcvCmdData(uint32_t SelSCI, uint16_t * RcvData, SCI_RxFIFOLevel size) {
    RxReadyFlag = 0;
    RxCopyCount = size;

    SCI_resetChannels(SelSCI);
    SCI_disableInterrupt(SelSCI, SCI_INT_TXRDY | SCI_INT_RXFF);
    SCI_clearInterruptStatus(SelSCI, SCI_INT_TXRDY | SCI_INT_RXRDY_BRKDT);
    SCI_enableFIFO(SelSCI);
    SCI_enableModule(SelSCI);
    SCI_performSoftwareReset(SelSCI);

    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX0, size);
    SCI_enableInterrupt(SCIA_BASE, SCI_INT_TXRDY | SCI_INT_RXFF);

    while(RxReadyFlag == 0);


    memcpy(RcvData, receivedChar, size*2);
}

/*******************************************************************
* parseMsgSCI - Parsing received data
*
* DataFrame[16]---------------
* ID 1(0)
* Target 2(1)
* Length 3(2)       - MAX:13
* Data 4~16(3 ~ 15)
*
* CmdData[16]-----------------
* ID 1(0)
* Target 2(1)
* Data 3~16(2~15)
*
*******************************************************************/
void parseMsgSCI(uint16_t* DataFrame, uint16_t* CmdData) {
    uint16_t i;

    if(DataFrame[0] > 0) {
        CmdData[0] = DataFrame[0];
        CmdData[1] = DataFrame[1];
        if(DataFrame[2] <= 13) {
            for (i=0;i<DataFrame[2];i++) {
                CmdData[i+2] = DataFrame[i*2+3] & 0xFF;
                CmdData[i+2] |= (DataFrame[i*2+4] & 0XFF) << 8;
            }
        }
    }
}
