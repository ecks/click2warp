/*! \file warpphy.h
\brief Header file for the WARPPHY functions

@version 10
@author Patrick Murphy and Chris Hunter

This header file contains the macros, function prototypes, and typedefs required for WARPPHY.
*/

//WARPPHY Interface
/***************CHANGELOG*****************

******************************************/
/*****************WARPPHY*****************
Description: This file specifies the
interface between to the PHY.
******************************************/

#ifndef WARPPHY_H
#define WARPPHY_H

//"XPAR_OFDM_TXRX_MIMO_PLBW_0_XC_VERSION" will be defined only when using the PLB46 flow in EDK/Sysgen 10.1+
#define OFDM_BASEADDR 0
#define OFDM_PKTBUFF_BASEADDR XPAR_XPS_BRAM_IF_CNTLR_2_BASEADDR

//Masks for configuring modulation settings in the PHY
//Each is 8 copies of a 4-bit modulation value
#define MODMASK_BPSK 0x11111111
#define MODMASK_QPSK 0x22222222
#define MODMASK_16QAM 0x44444444
#define MODMASK_64QAM 0x66666666

#define NUMPKTBUFFS 4

//Define a bunch of cryptic constants for the PHY's phase tracking filter
#define INIT_A_KPVAL 0//0x7FFFFFFF//0xA6800
#define INIT_B_KIVAL 0x2500//xD000//x380000//0x7FFFFFFF//0xA6800
#define INIT_B_KPVAL 0xD0000//x2F000//x2620000//x7FFFFFFF//0x54afb0//0x1BC7D10//0x2C3C38//0xA6800
#define INIT_PN_KVAL 0x2700000//xFFFFFFFF
#define INIT_RXFFTOFSET 12

//Define scaling values for the PHY's FFT cores
#define TX_FFT_SCALING_STAGE1 1
#define TX_FFT_SCALING_STAGE2 2
#define TX_FFT_SCALING_STAGE3 3

#define RX_FFT_SCALING_STAGE1 0	//1
#define RX_FFT_SCALING_STAGE2 1	//2
#define RX_FFT_SCALING_STAGE3 1	//1

//Define thresholds for the AGC
#define AGC_THRESH_1 0xE2 //-30
#define AGC_THRESH_2 0xCB //-53
#define AGC_THRESH_3 0xA6 //-90
//RX Status register values
#define INCOMPLETE 0
#define GOODPACKET 1
#define BADPACKET 2

//Bit masks for the options configured in Rx_ControlBits
#define RESET_BER 			0x1
#define REQ_LONG_CORR		0x2
#define DYNAMC_PKT_LENGTHS	0x4
#define BIG_PKTBUF_MODE		0x8
#define RX_SISO_MODE		0x10
#define REQ_TWO_LONG_CORR	0x20
#define REQ_SHORT_CORR		0x40
#define EXT_PKT_DETECT 		0x80
#define INT_PKT_DETECT 		0x100
#define BYPASS_CARR_REC		0x200
#define COARSE_CFO_EN		0x400
#define DEBUG_CFO_OUTSEL	0x800
#define CFO_USE_LONGCORR	0x1000
#define USE_PILOT_ARCTAN	0x2000
#define EQ_BYPASS_DIVISION  0x4000
#define SIMPLE_DYN_MOD_EN	0x10000
#define SWITCHING_DIV_EN	0x20000
#define SISO_ON_ANTB		0x40000
#define RESET_ON_BAD_HDR	0x80000
#define RX_GLOBAL_RESET		0x80000000

//Bit masks for the Tx/Rx interrupt enables
#define INTR_RST_HEADER		0x1
#define INTR_RST_PKTS			0x2
#define INTR_RST_TXDONE		0x4
#define INTR_EN_GOOD_PKT	0x8
#define INTR_EN_BAD_PKT		0x10
#define INTR_EN_GOOD_HEADER	0x20
#define INTR_EN_BAD_HEADER		0x40
#define INTR_EN_TX_DONE			0x80

#define ALL_INTERRUPT_ENABLE (INTR_EN_GOOD_HEADER|INTR_EN_BAD_HEADER|INTR_EN_BAD_PKT|INTR_EN_GOOD_PKT|INTR_EN_TX_DONE)
#define DEFAULT_INTERRUPTS (INTR_EN_GOOD_HEADER|INTR_EN_BAD_HEADER|INTR_EN_BAD_PKT|INTR_EN_GOOD_PKT)
#define DEFAULT_INTERRUPTRESETS (INTR_RST_HEADER|INTR_RST_PKTS|INTR_RST_TXDONE)

//Define which radios get used
//RADIOx_ADDR are defined by the radio controller driver
#define FIRST_RADIO RADIO2_ADDR
#define SECOND_RADIO RADIO3_ADDR

//Shorthand for configuring the radio controller's selected band
#define GHZ_5 0
#define GHZ_2 1

//Bit masks for OFDM Tx options
#define TX_SISO_MODE		0x1
#define TX_RANDOM_MODE		0x2
#define TX_DISABLE_ANTB_PREAMBLE 0x4
#define TX_PILOT_SCRAMBLING 0x8
//Bits 0xF0 are used for 4-bit preable shift value
#define TX_SISO_ON_ANTB 0x100

//MAC2PHY Options
#define TXBLOCK		0x0
#define TXNOBLOCK	0x1

//Macros for accessing the OFDM packet buffer; buff is an integer in [0,NUMPKTBUFFS-1]
#define warpphy_copyBytesToPhy(buff,src,len) memcpy(OFDM_PKTBUFF_BASEADDR + buff * 0x1000,(src),(len))
#define warpphy_copyBytesFromPhy(buff,dest,len) memcpy((dest), OFDM_PKTBUFF_BASEADDR + buff * 0x1000, (len))

//Macro to retrieve the physical memory address for a given packet buffer index 
#define warpphy_getBuffAddr(c) OFDM_PKTBUFF_BASEADDR + (c < NUMPKTBUFFS)*c*(0x800)

//Macros to read/write PHY registers
#define mimo_ofdmRx_setByteNums(c) ofdm_txrx_mimo_WriteReg_Rx_pktByteNums(OFDM_BASEADDR, c)
#define mimo_ofdmRx_setRxScaling(c) ofdm_txrx_mimo_WriteReg_Rx_Constellation_Scaling(OFDM_BASEADDR, c)
#define mimo_ofdmRx_setLongCorrParams(c) ofdm_txrx_mimo_WriteReg_Rx_PktDet_LongCorr_Params(OFDM_BASEADDR, c)
#define mimo_ofdmRx_setNumOFDMSyms(c) ofdm_txrx_mimo_WriteReg_Rx_OFDM_SymbolCounts(OFDM_BASEADDR, c)
#define mimo_ofdmRx_setCFO_B_KI(c) ofdm_txrx_mimo_WriteReg_Rx_FreqOffFilt_B_KI(OFDM_BASEADDR, c)
#define mimo_ofdmRx_setCFO_B_KP(c) ofdm_txrx_mimo_WriteReg_Rx_FreqOffFilt_B_KP(OFDM_BASEADDR, c)
#define mimo_ofdmRx_setPNTrack_K(c) ofdm_txrx_mimo_WriteReg_Rx_PhaseNoiseTrack_K(OFDM_BASEADDR, c)
#define mimo_ofdmTx_setPilot1Index(c) ofdm_txrx_mimo_WriteReg_Tx_Pilots_Index1(OFDM_BASEADDR, c)
#define mimo_ofdmTx_setPilot1Value(c) ofdm_txrx_mimo_WriteReg_Tx_Pilots_Value1(OFDM_BASEADDR, c)
#define mimo_ofdmTx_setPilot2Index(c) ofdm_txrx_mimo_WriteReg_Tx_Pilots_Index2(OFDM_BASEADDR, c)
#define mimo_ofdmTx_setPilot2Value(c) ofdm_txrx_mimo_WriteReg_Tx_Pilots_Value2(OFDM_BASEADDR, c)
#define warpphy_setNumSyms(c) ofdm_txrx_mimo_WriteReg_Tx_OFDM_SymCounts(OFDM_BASEADDR, c)
#define mimo_ofdmTx_setPreambleScaling(c) ofdm_txrx_mimo_WriteReg_Tx_PreambleScaling(OFDM_BASEADDR, c)
#define mimo_ofdmTx_setControlBits(c) ofdm_txrx_mimo_WriteReg_Tx_ControlBits(OFDM_BASEADDR, c)
#define mimo_ofdmTx_getOptions() ofdm_txrx_mimo_ReadReg_Tx_ControlBits(OFDM_BASEADDR)
#define mimo_ofdmTxRx_setFFTScaling(c) ofdm_txrx_mimo_WriteReg_TxRx_FFT_Scaling(OFDM_BASEADDR, c)
#define mimo_ofdmRx_setFFTWindowOffset(c) ofdm_txrx_mimo_WriteReg_Rx_PktDet_Delay(OFDM_BASEADDR, (ofdm_txrx_mimo_ReadReg_Rx_PktDet_Delay(OFDM_BASEADDR) & 0xFFFFE07F) | ((c&0x3F)<<7))
#define mimo_ofdmRx_setPktDetDly(c) ofdm_txrx_mimo_WriteReg_Rx_PktDet_Delay(OFDM_BASEADDR, (ofdm_txrx_mimo_ReadReg_Rx_PktDet_Delay(OFDM_BASEADDR) & 0xFFFFFF80)| (c&0x7F))
#define mimo_ofdmTx_setPktDoneReset(c) ofdm_txrx_mimo_WriteReg_Tx_Start_Reset_Control(OFDM_BASEADDR, (c<<2)&0x4)

///Structure contains PHY header
typedef struct {
	///Full-rate modulation order
	unsigned char fullRate;
	///Reserved byte
	unsigned char reserved4;
	///The length of the packet (in bytes). NOTE: This should only specify the length of the payload to-be-sent.
	unsigned short int length;
	///Type of packet this particular Macframe corresponds to (e.g. DATA, ACKPACKET, etc.)
	unsigned char pktType;
	///Destination MAC address.
	unsigned char destAddr[6];
	///Source MAC address.
	unsigned char srcAddr[6];
	///Number of times this packet has been retransmitted.
	unsigned char currReSend;
	///Reserved byte
	unsigned char reserved1;
	///Reserved byte
	unsigned char reserved2;
	///Reserved byte
	unsigned char reserved3;
	///Checksum of the packet will be automatically inserted by PHY
	unsigned short int checksum;
} phyHeader __attribute__((__aligned__(8)));

//Prototypes for functions in warpphy.c
int warpphy_init();
void warpphy_clearRxInterrupts();
void warpphy_clearTxInterrupts();
int warpphy_pktTx(unsigned int block);
void mimo_ofdmRx_enable();
void mimo_ofdmRx_disable();
void mimo_ofdmRx_setOptions(unsigned int someOptions, unsigned int intType);
unsigned int mimo_ofdmRx_getOptions();
void mimo_ofdmTx_disable();
void mimo_ofdmTx_enable();
void warpphy_setBuffs(unsigned char txBufOffset, unsigned char rxBufOffset);
void warpphy_enableSisoMode();
void warpphy_enableMimoMode();
void warpphy_enableMisoMode();
void warpphy_disableMisoMode();
void warpphy_setNumTrainingSyms(unsigned int c);
void warpphy_setPktDlyPlus();
void warpphy_setPktDlyMinus();
void warpphy_set_PN_KPlus(unsigned int increment);
void warpphy_set_PN_KMinus(unsigned int decrement);
void warpphy_set_CFODebugOutput(unsigned char outputSel);
void print_CFO_constants();
void warpphy_set_B_KPPlus(unsigned int increment);
void warpphy_set_B_KPMinus(unsigned int decrement);
void warpphy_set_B_KIPlus(unsigned int increment);
void warpphy_set_B_KIMinus(unsigned int decrement);
void warpphy_set_FFTOffset_Plus();
void warpphy_set_FFTOffset_Minus();
void warpphy_setNoiseTargetPlus();
void warpphy_setNoiseTargetMinus();
void warpphy_setTargetPlus();
void warpphy_setTargetMinus();
void warpphy_set_modulation(unsigned char baseRate, unsigned char antAFullRate, unsigned char antBFullRate);
void warpphy_setSISOAntenna(unsigned char antSel);
int warpphy_setChannel(unsigned char band,unsigned int c);
int warpphy_applyTxDCOCorrection(unsigned int radioSelection);
void warpphy_clearRxHeaderInterrupt();
void warpphy_setPktDetPlus(unsigned int offset);
void warpphy_setPktDetMinus(unsigned int offset);
void warpphy_setCSMAPlus(unsigned int offset);
void warpphy_setCSMAMinus(unsigned int offset);
int warpphy_isFree();
char warpphy_pollRxStatus();
void ofdm_AGC_SetTarget(unsigned int target);
void ofdm_AGC_SetDCO(unsigned int AGCstate);
void ofdm_AGC_Reset();
void ofdm_AGC_MasterReset();
void ofdm_AGC_Initialize(int noise_estimate);
void ofdm_AGC_setNoiseEstimate(int noise_estimate);
unsigned int ofdm_AGC_GetGains(void);
void ofdm_timer_start();
void ofdm_timer_stop();
void ofdm_timer_clearInterrupt();

void warp_timer_init();

void warp_timer_start(unsigned char timer);
void warp_timer_stop(unsigned char timer);
void warp_timer_resume(unsigned char timer);
void warp_timer_pause(unsigned char timer);
void warp_timer_setMode(unsigned char timer, unsigned char mode);
void warp_timer_resetInterrupt(unsigned char timer);
void warp_timer_resetInterrupts();
void warp_timer_setVal(unsigned char timer, unsigned int val);
unsigned int warp_timer_timeLeft(unsigned char timer);
unsigned char warp_timer_getStatus(unsigned char timer);
unsigned char warp_timer_isExpired(unsigned char timer);
unsigned char warp_timer_isActive(unsigned char timer);
unsigned char warp_timer_isPaused(unsigned char timer);
unsigned int warp_timer_getStatuses();
unsigned char warp_timer_getInterrupts();

#endif


