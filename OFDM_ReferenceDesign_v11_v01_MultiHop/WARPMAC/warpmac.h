/*! \file warpmac.h
\brief Header file for the WARPMAC Framework.

@version 10
@author Chris Hunter and Patrick Murphy

This header file contains the macros, function prototypes, and typedefs required for WARPMAC.
*/



/*! \mainpage WARP APIs
* @version Used with Reference Design v10
*
* \section change_sec CHANGELOG
* \subsection refdes10 Reference Design 10
*  \li OPB is deprecated; every core hangs off PLB
*  \li Emac interrupts removed; polling to improve performance
*  \li Reduced Fall of TxData -> Rise of TxACK turn around time from 30 to 23 microseconds
* \subsection refdes9 Reference Design 9
*  \li Reduced Fall of TxData -> Rise of TxACK turn around time from 80 microseconds to 30 microseconds
*  \li Rx MAC processing is now pipelined so that processing starts upon the reception of good header information
*  \li MIMO kits can easily select which antenna to use SISO model on via pushbuttons
*  \li Tweaked timing parameters (DIFs, Contention Window) and thesholds (agc target gain) 
*
* \section intro_sec Introduction
*
* This document is the collective API for WARPMAC (the MAC design framework),
* WARPPHY (the PHY driver/interface), RADIO CONTROLLER, CSMAMAC (the carrier-sensing random access
* reference), and the workshop MAC exercises.
*
* \section start_sec Getting Started
*
* Obtain the latest reference design: http://warp.rice.edu/trac/wiki/OFDMReferenceDesign .
* In addition to being in the repository, all the source code for WARPMAC is available in
* the reference design project and is configured to compile "as-is."
*
* \section hardcopy_sec Hardcopy
* You can download a hardcopy of this reference manual at:
* http://warp.rice.edu/WARP_API/refman.pdf
*
* \section licence_sec License
* http://warp.rice.edu/license/
*
*
*/



#ifndef WARPMAC_H
#define WARPMAC_H

#include "xstatus.h"
#include "warpphy.h"

//Define which radios get used below
//RADIOx_ADDR are defined by the radio controller driver
//Address of the first radio
#define FIRST_RADIO RADIO2_ADDR
//Address of the second radio
#define SECOND_RADIO RADIO3_ADDR


#define MY_XEM_MAX_FRAME_SIZE 2000
#define PING 0
#define PONG 1

//Debug GPIO Pins
#define NONE	0
#define ONE		0x1
#define TWO		0x2
#define THREE	0x4
#define FOUR	0x8


//Masks for supported modulation schemes; equivalent to number of bits per symbol
#define BPSK	1
#define QPSK	2
#define QAM_16	4
#define QAM_64	6

//Constants used for packet headers specifying full-rate modulation per-packet
// This constant fills in an 8-bit field in the header
// Each 4-bit nibble corresponds to an antenna
#define HDR_FULLRAATE_BPSK		(BPSK | (BPSK<<4))
#define HDR_FULLRAATE_QPSK		(QPSK | (QPSK<<4))
#define HDR_FULLRAATE_QAM_16	(QAM_16 | (QAM_16<<4))
#define HDR_FULLRAATE_QAM_64	(QAM_64 | (QAM_64<<4))

//5 GHz Band
#define GHZ_5 0
//2.4 GHz Band
#define GHZ_2 1

//Interrupts - Position in the interrupt vector inside the Intc peripheral
//Timer interrupt identifier
#define TMA_INTERRUPT 3
//Receive "good" interrupt identifier
#define RX_INTERRUPT 2
//Receive "bad" interrupt identifier
#define BADRX_INTERRUPT 1
//Push button interrupt identifier
#define PB_INTERRUPT 0

//Identifier for deterministic timeout countdowns
#define TIMEOUT 0
//Identifier for random backoff countdowns
#define BACKOFF 1

//Borrowed from xemaclite.c; extracts the EtherType field from a received frame
#define XEmacLite_mGetReceiveDataLength(BaseAddress) ((XEmacLite_mReadReg((BaseAddress), XEL_HEADER_OFFSET + XEL_RXBUFF_OFFSET) >> XEL_HEADER_SHIFT) & (XEL_RPLR_LENGTH_MASK_HI | XEL_RPLR_LENGTH_MASK_LO))



//Shortcuts for xparameters.h constants
// Device IDs
#define EMACLITE_DEVICE_ID	 	XPAR_EMACLITE_0_DEVICE_ID
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define DMA_CTRL_DEVICE_ID		XPAR_DMACENTRAL_0_DEVICE_ID



#define DMADIR_IDLE			0
#define DMADIR_EMAC_TO_PHY	1
#define DMADIR_PHY_TO_EMAC	2

//The number of bytes in the header must be fixed and known by every node ahead of time
// It also must occupy an integral number of OFDM symbols (i.e. the base-rate symbols)
#define NUM_HEADER_BYTES 24

///Structure contains the header contents of a packet
typedef struct {
	///Physical layer header struct
	phyHeader header;
	///Flag for protecting a Macframe. This is strictly optional and is only for use by user code; the framework ignores this field completely
	unsigned char isNew;
} Macframe __attribute__((__aligned__(8)));

///Structure of miscellaneous control bits needed for correct operation of the MAC.
typedef struct {
	///Timeout time of system
	volatile unsigned int timeout;
	///Smallest time interval in system
	volatile unsigned int slotTime;
	///The node's self MAC address
	volatile unsigned char selfAddr[6];
	///Maximum number of retransmissions before dropping a packet
	volatile unsigned char maxReSend;
	///Current backoff window
	volatile unsigned char currBackoff;
	///Maximum backoff window
	volatile unsigned char maxBackoff;
	///Type of timer (#TIMEOUT or #BACKOFF)
	volatile unsigned char timerType;
	///Whether the Emac is currently enabled
	volatile unsigned char enableEthernetInt;
	///Which of the PHY's buffers is currently associated with transmission
	volatile unsigned char txBuffIndex;
	///Which of the PHY's buffers is currently associated with reception
	volatile unsigned char rxBuffIndex;
	///Constellation order of the full rate symbols (#BPSK, #QPSK, #QAM_16, or #QAM_64)
	volatile unsigned char mod_fullRateA;
	//volatile unsigned char mod_fullRateB;
} Maccontrol;



///@brief Halts a currently running timer
///
///NOTE: This is a macro wrapper around ofdm_timer_stop()
#define warpmac_stopTimer() ofdm_timer_stop()

void nullCallback(void* param);
void warpmac_init();
void phyRx_goodHeader_int_handler();
void phyRx_badHeader_int_handler();
void timer_int_handler();
int warpmac_clearTimer();
void warpmac_setTimerVal(Xuint32 clocks);
void warpmac_startTimer(unsigned char mode);
unsigned int randNum(unsigned int N);
unsigned int warpmac_carrierSense();
void warpmac_setTimer(int type);
int warpmac_startEmacXmit(Macframe* packet);
void warpmac_prepEmacForXmit(Macframe* packet);
void warpmac_setUpButtonCallback(void(*handler)());
void warpmac_setLeftButtonCallback(void(*handler)());
void warpmac_setRightButtonCallback(void(*handler)());
void warpmac_setMiddleButtonCallback(void(*handler)());
void warpmac_setTimerCallback(void(*handler)());
void warpmac_setEmacCallback(void(*handler)());
void warpmac_setGoodPacketCallback(void(*handler)());
void warpmac_setGoodHeaderCallback(void(*handler)());
void warpmac_setBadPacketCallback(void(*handler)());
void warpmac_setTxDoneCallback(void(*handler)());
void userIO_int_handler(void *InstancePtr);
void warpmac_incrementLEDLow();
void warpmac_incrementLEDHigh();
void warpmac_setBaseRate(unsigned char rate);
void warpmac_setFullRate(unsigned char rate);
void warpmac_prepPhyForXmit(Macframe* packet,unsigned char buffer);
void warpmac_startPhyXmit(unsigned char buffer);
void warpmac_setRxBuffer(Macframe* packet,unsigned int rxBuff);
void warpmac_setTxBuffer(unsigned int txBuff);
int warpmac_getRxBuffer();
int warpmac_getTxBuffer();
int warpmac_incrementResend(Macframe* packet);
int warpmac_addressedToMe(Macframe* packet);
int warpmac_addressedFromThem(Macframe* packet,unsigned char* addr);
int warpmac_addressedToThem(Macframe* packet,unsigned char* addr);
void warpmac_setMaxResend(unsigned int c);
void warpmac_setMaxCW(unsigned int c);
void warpmac_setMacAddr(unsigned char* addr);
void warpmac_setTimeout(unsigned int time);
void warpmac_setSlotTime(unsigned int time);
int warpmac_inTimeout();
int warpmac_getMyId();
void warpmac_enableCSMA();
void warpmac_disableCSMA();
unsigned char sevenSegmentMap(unsigned char x);
void warpmac_leftHex(unsigned char x);
void warpmac_rightHex(unsigned char x);
void warpmac_setDebugGPIO(unsigned char val);
void warpmac_enableEthernet();
void warpmac_disableEthernet();
void ErrorTrap(char *Message);
void warpmac_finishPhyXmit();
void warpmac_pollEthernet();
void warpmac_enableSisoMode();
void warpmac_enableMimoMode();
void warpmac_enableMisoMode();
void warpmac_disableMisoMode();

void warpmac_uartSendHandler(void *CallBackRef, unsigned int EventData);
//void warpmac_uartRecvHandler(void *CallBackRef, unsigned int EventData);
void warpmac_uartRecvHandler(void *CallBackRef);
void warpmac_setUartRecvCallback(void(*handler)());

#endif


