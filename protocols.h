//#############################################################################
// A_F28xProtocols By Trollonion03
//
// FILE:   protocols.h
// TARGET: TMS320F280049C (LAUNCHXL-F280049C)
// LANGUAGE: C
// COMPILER: TI 21.6.0.LTS
// DEBUGGER: XDS110(On-board-USB-Debugger)
// REFERENCE: C2000Ware_3_04_00_00
//#############################################################################

#ifndef INC_PROTOCOLS_H_
#define INC_PROTOCOLS_H_

extern uint16_t RxReadyFlag;
extern uint16_t RxCopyCount;
extern uint16_t receivedChar[16];

void sendDataSCI(uint32_t SelSCI, uint16_t * TrsData, SCI_TxFIFOLevel size);
void rcvCmdData(uint32_t SelSCI, uint16_t * RcvData, SCI_RxFIFOLevel size);
void parseMsgSCI(uint16_t *DataFrame, uint16_t* CmdData);
void makePacketSCI(uint16_t* dataFrame, uint16_t* packet_data, uint16_t ID, uint16_t len, uint16_t target);

#endif /* INC_PROTOCOLS_H_ */
