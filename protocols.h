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



void sendDataSCI(uint32_t SelSCI, uint16_t * TrsData, SCI_TxFIFOLevel size);
void rcvCmdData(uint32_t SelSCI, uint16_t * RcvData, SCI_RxFIFOLevel size);
void parseMsgSCI(uint16_t *DataFrame, uint16_t* CmdData);

#endif /* INC_PROTOCOLS_H_ */
