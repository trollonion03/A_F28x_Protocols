//#############################################################################
// A_F28xProtocols by Trollonion03
//
// FILE:   main.c
// TARGET: TMS320F280049C (LAUNCHXL-F280049C)
// LANGUAGE: C
// COMPILER: TI 21.6.0.LTS
// DEBUGGER: XDS110(On-board-USB-Debugger)
// REFERENCE: C2000Ware_3_04_00_00
//#############################################################################
#include "driverlib.h"
#include "device.h"

#include "protocols.h"


//Defines
#define AUTOBAUD 0
#define RESTRICTED_REGS 0
#define USE_TX_INTERRUPT 0

/***************************************************************************************
* Globals, for large data (Do not use malloc)
* Read stdint.h.In TMS320C2000, uin16_t is unsigned int.
***************************************************************************************/
uint16_t counter = 0; //unsigned int.
volatile uint16_t sData[16];
volatile uint16_t rData[16];
unsigned char *msg;
unsigned char data[16];

uint16_t cmd[16];

// Function Prototypes

//for setup


//Interrupt Service Routine
#if USE_TX_INTERRUPT
__interrupt void sciaTxISR(void);
#endif
__interrupt void sciaRxISR(void);


void main(void)
{
    // Configure Device.
    Device_init();
    Device_initGPIO();
    PinMux_setup_SCI();
    PinMux_setup_GPIO();

#if RESTRICTED_REGS
    EALLOW;
    EDIS;
#endif

    // Disable global interrupts.
    DINT;

    // Initialize interrupt controller and vector table.
    Interrupt_initModule();
    Interrupt_initVectorTable();
    IER = 0x0000;
    IFR = 0x0000;

    // Map the ISR to the wake interrupt.
#if USE_TX_INTERRUPT
    Interrupt_register(INT_SCIA_TX, sciaTxISR);
#endif
    Interrupt_register(INT_SCIA_RX, sciaRxISR);

    // Initialize SCIA and its FIFO.
    SCI_performSoftwareReset(SCIA_BASE);

    /************************************************************************
    * Configure SCIA (UART_A)
    * Bits per second = 9600
    * Data Bits = 8
    * Parity = None
    * Stop Bits = 1
    * Hardware Control = None
    ************************************************************************/
    SCI_setConfig(SCIA_BASE, 25000000, 9600, (SCI_CONFIG_WLEN_8 |
                                             SCI_CONFIG_STOP_ONE |
                                             SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(SCIA_BASE);
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXRDY | SCI_INT_RXRDY_BRKDT);
    SCI_enableFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);
    SCI_performSoftwareReset(SCIA_BASE);

    // Enable the TXRDY and RXRDY interrupts.
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX0, SCI_FIFO_RX16);
    SCI_enableInterrupt(SCIA_BASE, SCI_INT_TXFF | SCI_INT_RXFF);

#if AUTOBAUD
    // Perform an autobaud lock.
    // SCI expects an 'a' or 'A' to lock the baud rate.
    SCI_lockAutobaud(SCIA_BASE);
#endif

    // Send starting message.
    msg = "\r\nKJPL_Launcher\n\0";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 17);

    // Clear the SCI interrupts before enabling them.
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF | SCI_INT_RXFF);


    // Enable the interrupts in the PIE: Group 9 interrupts 1 & 2.
    Interrupt_enable(INT_SCIA_RX);
//    Interrupt_enable(INT_SCIA_TX);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

    // Enable global interrupts.
    EINT;

    for(;;) {
    }
}


#if USE_TX_INTERRUPT
/*******************************************************************
* sciaTxISR - Disable the TXRDY interrupt and print message asking
*             for a character.
*******************************************************************/
__interrupt void sciaTxISR(void) {
    // Disable the TXRDY interrupt.
    SCI_disableInterrupt(SCIA_BASE, SCI_INT_TXRDY);

    msg = "\r\nEnter a character: \0";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 22);

    // Ackowledge the PIE interrupt.
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
#endif


//sciaRxISR - Read the character from the RXBUF.
__interrupt void sciaRxISR(void) {
    uint16_t i;
    unsigned char Ack[2];

    // Enable the TXRDY interrupt again.
    //SCI_enableInterrupt(SCIA_BASE, SCI_INT_TXRDY);

    // Read a character from the RXBUF.
    for (i=0;i<RxCopyCount;i++) {
        receivedChar[i] = SCI_readCharBlockingNonFIFO(SCIA_BASE);
    }

    //return Ack via SCI
    if(receivedChar[0] == 1) {
        Ack[0] = 1;
        Ack[1] = 8;
    }
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)Ack, sizeof(Ack));

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

    RxReadyFlag = 1;
}
