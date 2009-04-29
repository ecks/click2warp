/*! \file warpphy.c
 \brief Provide PHY-specific functions for interfacing WARPMAC (and MAC code) to the OFDM PHY peripherals
 
 @version 10
 @author Chris Hunter & Patrick Murphy
 
 Many of the register names and bit assignments in this file depend on the design of the OFDM PHY. You must
 use matching versions of WARPMAC, WARPPHY and ofdm_txrx_mimo. Refer to the wireless reference designs for
 known-good combinations of versions.
 */

//XPS-generated header with peripheral parameters
#include "xparameters.h"

//Header for WARPPHY Interface
#include "warpphy.h"

//Header for WARPMAC framework (required for some global values)
#include "warpmac.h"

//These are used for 10.1 PLB46-based designs
// These macros map the old sysgen2opb register access macros to the new PLB46 export macros
#include "ofdm_txrx_mimo_regMacros.h"
#include "ofdm_agc_mimo_regMacros.h"
#include "ofdm_pktdetector_mimo_regMacros.h"
#include "warp_timer_regMacros.h"

//Headers for other WARP peripheral cores
#include "EEPROM.h"
#include "radio_controller_basic.h"
#include "radio_controller_ext.h"
#include "radio_controller_adv.h"
#include "radio_controller_5ghz.h"

//Other standard header files
#include <stdio.h>
#include <string.h>

//********Globals********
int agctarget;
int agcnoiseEst;
unsigned int warpphy_sisoMode;
unsigned char baseRateMod;
unsigned int numTrainingSyms;
unsigned int pktDly;
unsigned char globalReset;
unsigned int numBaseRate;
unsigned char txGain;

unsigned int pktdetthresh = 9000;
unsigned int csmathresh = 4000;

unsigned int bothRadios = FIRST_RADIO | SECOND_RADIO;
unsigned int activeRadio, lastActiveRadio;

unsigned int A_KPval;
unsigned int B_KIval;
unsigned int B_KPval;
unsigned int PN_KVal;
unsigned int RxFFT_Window_Offset;
/////////////////////////////////

///@brief Initializes the OFDM PHY core
///The OFDM PHY and radio controller cores have many options which must be configured before they can
///be used over the air. This funciton configures these cores with sensible defaults.
int warpphy_init(){

	xil_printf("  Initializing WARPPHY:\r\n");

	//Initialize global variables
	agctarget = -15;
	agcnoiseEst = -95;
	warpphy_sisoMode = 1;
	baseRateMod = QPSK;
	numTrainingSyms = 2;
	pktDly = 58;
	globalReset = 0;
	numBaseRate = 2;
	txGain = 0x3f;

	pktdetthresh = 9000;
	csmathresh = 4000;

	activeRadio = FIRST_RADIO;	

	A_KPval = INIT_A_KPVAL;
	B_KIval = INIT_B_KIVAL;
	B_KPval = INIT_B_KPVAL;
	PN_KVal = INIT_PN_KVAL;
	RxFFT_Window_Offset = INIT_RXFFTOFSET;

	/*****************EEPROM Setup******************/
/*
	int eepromStatus;
	unsigned int calReadback;
	Xuint8 memory[8], version, revision, valid, MAC[6], i;
	Xuint16 serial;
	//Extract the various numbers from the EEPROM bytes
	version = (memory[0] & 0xE0) >> 5;
	revision = (memory[1] & 0xE0) >> 5;
	valid = memory[1] & 0x1F;

	//Choose the EEPROM on the selected radio board; second arg is [0,1,2,3,4] for [FPGA, radio1, radio2, radio3, radio4]
	eepromStatus = WarpEEPROM_EEPROMSelect((unsigned int *)XPAR_EEPROM_0_MEM0_BASEADDR, 0);

	if(eepromStatus != 0)
	{
		xil_printf("EEPROM Select Failed!\r\n");
		return;
	}

	//Initialize the EEPROM controller
	eepromStatus = WarpEEPROM_Initialize((unsigned int *)XPAR_EEPROM_0_MEM0_BASEADDR);
	if(eepromStatus != 0)
	{
		xil_printf("EEPROM Init Returned %x\r\n", eepromStatus);
		xil_printf("EEPROM Init Failed!\r\n");
		return;
	}

	eepromStatus = WarpEEPROM_EEPROMSelect((unsigned int*)XPAR_EEPROM_0_MEM0_BASEADDR, 2);
	usleep(100);

	//Read the first page from the EERPOM
	WarpEEPROM_ReadMem((unsigned int*)XPAR_EEPROM_0_MEM0_BASEADDR, 0, 0, memory);

	xil_printf("\r\n\r\nEEPROM Values for Radio Board in Slot %d\r\n", 2);

	xil_printf("    WARP Radio Board Version %d.%d\r\n", version, revision);

	serial = WarpEEPROM_ReadWARPSerial((unsigned int*)XPAR_EEPROM_0_MEM0_BASEADDR);

	xil_printf("    Serial Number (WARP): WR-a-%05d\r\n", serial);

	WarpEEPROM_ReadDSSerial((unsigned int*)XPAR_EEPROM_0_MEM0_BASEADDR, memory);
	print("    EEPROM Hard-wired Serial Number: ");
	for(i=1;i<7;i++)
		xil_printf(" %x",memory[7-i]);

	calReadback = WarpEEPROM_ReadRadioCal((unsigned int*)XPAR_EEPROM_0_MEM0_BASEADDR, 2, 1);
	xil_printf("\r\n  Current TxDCO Values: I: %d Q: %d\r\n", (signed short)(((signed char)(calReadback & 0xFF))<<1), (signed short)(((signed char)((calReadback>>8) & 0xFF))<<1));
*/

	/*****************END EEPROM Setup*************/


	/*****************Radio Setup******************/
	xil_printf("	Initializing Radio Transmitter...\r\n");
	
	//The second argument sets the clock ratio in the radio controller's SPI controller
	// 2 is the right value for an 80MHz bus
	// 1 is good for a 40MHz bus
	WarpRadio_v1_Reset((unsigned int *)XPAR_RADIO_CONTROLLER_0_BASEADDR, 1);
	
	xil_printf("	Starting TxDCO...\r\n");

	//Apply the stored TX DC offset correction values for each radio
	warpphy_applyTxDCOCalibration(FIRST_RADIO);
	warpphy_applyTxDCOCalibration(SECOND_RADIO);
	
	//Set Tx bandwidth - 0x1 = 24MHz (minimum)
	WarpRadio_v1_TxLpfCornFreqCoarseAdj(0x1, FIRST_RADIO | SECOND_RADIO);
	
	//Enable hardware control of Tx gains (handled by the radio controller Tx state machine)
	WarpRadio_v1_SoftwareTxGainControl(0, FIRST_RADIO | SECOND_RADIO);
	
	//Four arguments: dly_TxEn, dly_TxStart, dly_GainRampEn, dly_PowerAmpEn, each in units of bus clock cycles
	WarpRadio_v1_SetTxTiming(FIRST_RADIO | SECOND_RADIO, 0, 250, 100, 0);
	
	//Three arguments: targetGain, gainStep, stepInterval; max targetGain is 0x3F
	WarpRadio_v1_SetTxGainTiming(FIRST_RADIO | SECOND_RADIO, 0x3F, 0xF, 1);
	
	//0x3 is max gain
	WarpRadio_v1_BaseBandTxGain(0x3, FIRST_RADIO | SECOND_RADIO);
	
	xil_printf("	complete!\r\n");
	
	xil_printf("	Initializing Radio Receiver...");

	//Enable hardware AGC control of receive gains & RxHP
	WarpRadio_v1_RxHpSoftControlDisable(FIRST_RADIO | SECOND_RADIO);
	WarpRadio_v1_SoftwareRxGainControl(0, FIRST_RADIO | SECOND_RADIO);

	/*
	 //Bypass AGC - contorl Rx gain via software
	 WarpRadio_v1_RxHpSoftControlEnable(FIRST_RADIO | SECOND_RADIO);
	 WarpRadio_v1_SoftwareRxGainControl(1, FIRST_RADIO | SECOND_RADIO);
	 
	 WarpRadio_v1_RxLNAGainControl(0x2, FIRST_RADIO | SECOND_RADIO);
	 WarpRadio_v1_RxVGAGainControl(0x10, FIRST_RADIO | SECOND_RADIO);
	*/
	
	//Set bandwith with RxHP=0; RxHighPassCornerFreq(0) is critical for good performance
	WarpRadio_v1_RxHighPassCornerFreq(0, FIRST_RADIO | SECOND_RADIO);
	
	//Set Rx bandwidth; 0x0 = 15MHz (minimum)
	WarpRadio_v1_RxLpfCornFreqCoarseAdj(0, FIRST_RADIO | SECOND_RADIO);
	
	xil_printf("complete!\r\n");
	/**********************************************************/
	
	/*************************PHY******************************/
	//Disable processing in the Tx PHY to prevent any accidental transmissions during setup
	mimo_ofdmTx_disable();
	//Disable processing in the Rx PHY to prevent any accidental receptions during setup
	mimo_ofdmRx_disable();

	xil_printf("	Initializing OFDM Transmitter...");
	
	//Configure the FFT scaling values in the PHY Tx and Rx
	mimo_ofdmTxRx_setFFTScaling((unsigned int)(((16*RX_FFT_SCALING_STAGE1 + 4*RX_FFT_SCALING_STAGE2 + 1*RX_FFT_SCALING_STAGE3)<<6 ) | (16*TX_FFT_SCALING_STAGE1 + 4*TX_FFT_SCALING_STAGE2 + 1*TX_FFT_SCALING_STAGE3)));
	
	//Subcarrier indicies for pilot tones - the values here must line up with zeros in the modulation setup for each subcarrier
	// Subcarriers [-21,-7,7,21] are the default, matching the 802.11a spec
	mimo_ofdmTx_setPilot1Index((7 + ((64-7)<<16) ));
	mimo_ofdmTx_setPilot2Index((21 + ((64-21)<<16)));
	
	//Pilot tone values are Fix16_15, so
	// 0x7FFF is +1, 0x8000 is -1
	// 0xA57D is ~-0.707, 0x5A82 is ~+0.707
	mimo_ofdmTx_setPilot1Value(0xA57D);
	mimo_ofdmTx_setPilot2Value(0x5A82);
	
	//3 values in this write: number of training symbols, number of base rate symbols and number of full rate symbols
	//The number of full rate symbols is updated with each packet transmission, so it's set to a sensible non-zero value here
	warpphy_setNumSyms(numTrainingSyms + (numBaseRate*256) + (40*65536));
	
	//Scaling constant for the transmitted preamble; helps normalize the amplitude of the stored preamble and actual IFFT output
	mimo_ofdmTx_setPreambleScaling(18350);

	//Configure various options in the Tx PHY
	mimo_ofdmTx_setControlBits (
								(2 << 4) | //Preamble shift for antenna B
								(TX_SISO_MODE * warpphy_sisoMode) | //SISO mode
								TX_PILOT_SCRAMBLING | //Pseudo-random scrambling of pilot tones
								//TX_DISABLE_ANTB_PREAMBLE | //Disables preamble on antenna B
								//TX_SISO_ON_ANTB | //Uses antenna B for SISO mode
								0
								);
	
	xil_printf("complete!\r\n");
	
	//Finally enable the transmitter; it won't actually do anything until user code transmits a packet
	mimo_ofdmTx_enable();
	
	xil_printf("	Initializing OFDM Receiver...");

	//Configures the default number of samples of the cyclic prefix to use for synchronization offset tolerance
	// Larger values here reduce multipath tolerance
	mimo_ofdmRx_setFFTWindowOffset(INIT_RXFFTOFSET);
	
	//Scaling value is a 32-bit value, composed of two UFix16_11 values
	// So 0x08000800 scales A/B by exactly 1
	// This value is dependent on the number of training symbols
	// For SISO mode and numTraining=2, use 0x10001000 (the default)
	mimo_ofdmRx_setRxScaling(0x10001000); 
	
	//Long correlator parameters (for packet detection confirmation and symbol timing)
	mimo_ofdmRx_setLongCorrParams(251-16-3  + (3000*65536));
	
	mimo_ofdmRx_setPktDetDly(50);
	mimo_ofdmRx_setNumOFDMSyms(numTrainingSyms + (numBaseRate<<16));
	
	//Bottom 8 bits are the number of header bytes per packet; nodes must agree on this at build time
	//	bits[7:0]: number of header bytes (bytes in base-rate symbols)
	//Three more 8-bit values are stored here - each is an index of a byte in the header:
	//	bits[15:8]: less-significant byte of pkt length
	//	bits[23:16]: more-significant byte of pkt length
	//	bits[31:24]: dynamic modulation masks
	//	mimo_ofdmRx_setByteNums( (unsigned int)(NUM_HEADER_BYTES + (2<<8) + (3<<16) + (0<<24) ));
	mimo_ofdmRx_setByteNums( (unsigned int)(NUM_HEADER_BYTES + (3<<8) + (2<<16) + (0<<24) ));
	
	//Set filter coefficients for the CFO and phase noise tracking systems
	mimo_ofdmRx_setCFO_B_KI(INIT_B_KIVAL);
	mimo_ofdmRx_setCFO_B_KP(INIT_B_KPVAL);
	mimo_ofdmRx_setPNTrack_K(INIT_PN_KVAL);
	
	//Configure a bunch of options in the Rx PHY
	mimo_ofdmRx_setOptions
	(
		//RESET_BER |
		REQ_LONG_CORR |
		//BIG_PKTBUF_MODE |
		DYNAMC_PKT_LENGTHS |
		REQ_TWO_LONG_CORR |
		REQ_SHORT_CORR |
		(RX_SISO_MODE * warpphy_sisoMode) |
		CFO_USE_LONGCORR |
		DEBUG_CFO_OUTSEL |
		COARSE_CFO_EN | //Enable coarse CFO estimation
		USE_PILOT_ARCTAN |
		//SWITCHING_DIV_EN | //Enable switching diversity at the receiver
		SIMPLE_DYN_MOD_EN |
		EXT_PKT_DETECT |
		//SISO_ON_ANTB | //Receive SISO stream on second radio
		//INT_PKT_DETECT |
		//EQ_BYPASS_DIVISION |
		RESET_ON_BAD_HDR | //Reset Rx PHY if header fails CRC
		0,
		DEFAULT_INTERRUPTS
	 );
	
	//Finally enable the Rx PHY
	mimo_ofdmRx_enable();
	
	//Set the modulation schemes (baseRate, antA_fullRate, antB_fullRate):
	// By default, we're in SISO mode, so antenna B gets 0 bits
	// 0xF enables dynamic modulation for antenna A (0xF gets AND'd with the header's full-rate field)
	warpphy_set_modulation(baseRateMod, 0xF, 0x0);
	
	//If you disable dynamic modulation, replace 0xF with an actual modulation scheme:
	//warpphy_set_modulation(QPSK, QPSK, 0x0);
	
	xil_printf("complete!\r\n");
	/**********************************************************/
	
	/***************************AGC****************************/
	xil_printf("	Initializing AGC...");
	ofdm_AGC_Initialize(agcnoiseEst);
	ofdm_AGC_setNoiseEstimate(agcnoiseEst);
	ofdm_AGC_SetDCO(1);
	ofdm_AGC_SetTarget(agctarget);
	ofdm_AGC_Reset();
	xil_printf("complete!\r\n");
	/**********************************************************/
	
	/*******************Packet Detector************************/
	xil_printf("	Initializing Packet Detection...");
	ofdm_pktDetector_mimo_WriteReg_csma_enableBusy(PKTDET_BASEADDR, 1);
	ofdm_pktDetector_mimo_WriteReg_csma_enableIdle(PKTDET_BASEADDR, 1);
	ofdm_pktDetector_mimo_WriteReg_pktDet_masterReset(PKTDET_BASEADDR, 1);
	//		ofdm_pktDetector_mimo_WriteReg_pktDet_detectionMask(PKTDET_BASEADDR, 2); //2 = radio slot 3
	ofdm_pktDetector_mimo_WriteReg_pktDet_detectionMask(PKTDET_BASEADDR, 1); //1 = radio slot 2
	ofdm_pktDetector_mimo_WriteReg_pktDet_detectionMode(PKTDET_BASEADDR, 1); //0 = AND, 1 = OR
	ofdm_pktDetector_mimo_WriteReg_pktDet_resetDuration(PKTDET_BASEADDR, 32);
	ofdm_pktDetector_mimo_WriteReg_pktDet_avgLen(PKTDET_BASEADDR, 16);
	ofdm_pktDetector_mimo_WriteReg_pktDet_avgThresh(PKTDET_BASEADDR, pktdetthresh); //7000
	ofdm_pktDetector_mimo_WriteReg_csma_avgThresh(PKTDET_BASEADDR, 16000);
	ofdm_pktDetector_mimo_WriteReg_csma_difsPeriod(PKTDET_BASEADDR, 1000); //625
	ofdm_pktDetector_mimo_WriteReg_pktDet_masterReset(PKTDET_BASEADDR, 0);
	ofdm_pktDetector_mimo_WriteReg_pktDet_reset(PKTDET_BASEADDR, 0);
	xil_printf("complete!\r\n");
	/**********************************************************/
	
	/**************************Timer***************************/
	warp_timer_init();
	/**********************************************************/
	
	//Finally, enable the Rx paths in the radios
	WarpRadio_v1_RxEnable(activeRadio);
	
	warpphy_enableSisoMode();
	warpphy_setNumTrainingSyms(2);
	return 0;
}

///@brief Clears any pending Rx interrupts in the PHY. Was warpphy_pktAck() in previous versions.
///
///The Rx PHY blocks after asserting either its good or bad packet interrupt output. The interrupts
///are cleared by asserting then de-asserting a register bit per interrupt. This funciton clears
///both interrupts; there is no harm in clearing an interrupt that isn't actually asserted.
void warpphy_clearRxInterrupts(){
	
	//The TxRx_Interrupt_PktBuf_Ctrl register has many bits.
	// bits[2:0] are the Rx interrupt enables
	// bits[6:4] are the Rx interrupt resets
	// bit[3] and bit[7] are used for the TxDone interrupt and are preserved here
	ofdm_txrx_mimo_WriteReg_TxRx_Interrupt_PktBuf_Ctrl(OFDM_BASEADDR, ofdm_txrx_mimo_ReadReg_TxRx_Interrupt_PktBuf_Ctrl(OFDM_BASEADDR) | DEFAULT_INTERRUPTRESETS);
	ofdm_txrx_mimo_WriteReg_TxRx_Interrupt_PktBuf_Ctrl(OFDM_BASEADDR, ofdm_txrx_mimo_ReadReg_TxRx_Interrupt_PktBuf_Ctrl(OFDM_BASEADDR) &  ~DEFAULT_INTERRUPTRESETS);

	return;
}

///@brief Releases the OFDM Rx PHY master reset
///
///De-asserting RX_GLOBAL_RESET allows the Rx PHY to begin processing packets.
void mimo_ofdmRx_enable(){
	
	//Clear any stale interrupts; this should never really be required
	warpphy_clearRxInterrupts();
	
	//De-asert the global reset
	ofdm_txrx_mimo_WriteReg_Rx_ControlBits(OFDM_BASEADDR, ofdm_txrx_mimo_ReadReg_Rx_ControlBits(OFDM_BASEADDR) & ~RX_GLOBAL_RESET);
	
	return;
}

///@brief Holds the OFDM Rx PHY in reset
///
///Asserting the RX_GLOBAL_RESET bit clears nearly all the state in the OFDM Rx. Configuration registers are not cleared.
void mimo_ofdmRx_disable(){
	
	//Assert the global reset
	ofdm_txrx_mimo_WriteReg_Rx_ControlBits(OFDM_BASEADDR, ofdm_txrx_mimo_ReadReg_Rx_ControlBits(OFDM_BASEADDR) | RX_GLOBAL_RESET);

	//Clear any stale interrupts; this should never really be required
	warpphy_clearRxInterrupts();
	
	return;
}

///@brief Configures options in the Rx PHY
///
///@param someOptions 32-bit options value, composed on bitwise OR'd values from warpphy.h
///@param intType Selects whether to interrupt on good or bad packets. Bitwise OR'd combination of INTR_BAD_PKTS and  INTR_GOOD_PKTS
void mimo_ofdmRx_setOptions(unsigned int someOptions, unsigned int intType){
	
	//Write the full controlBits register
	ofdm_txrx_mimo_WriteReg_Rx_ControlBits(OFDM_BASEADDR, someOptions);
	
	//The interrupt control bits are in the Interrupt_PktBuf_Ctrl register - bits[7:4]
	//Clear the interrupt control bits
	ofdm_txrx_mimo_WriteReg_TxRx_Interrupt_PktBuf_Ctrl(OFDM_BASEADDR, ofdm_txrx_mimo_ReadReg_TxRx_Interrupt_PktBuf_Ctrl(OFDM_BASEADDR) & 0xFFFFFF00);
	
	//Write just the interrupt control bits
	ofdm_txrx_mimo_WriteReg_TxRx_Interrupt_PktBuf_Ctrl(OFDM_BASEADDR, ofdm_txrx_mimo_ReadReg_TxRx_Interrupt_PktBuf_Ctrl(OFDM_BASEADDR) | (ALL_INTERRUPT_ENABLE & intType));
	
	return;
}

///@brief Returns the current value of the Rx PHY configuration register
///
/// Returns the value of the OFDM Rx ControlBits register. Use the bit masks from warpphy.h to decode the indivitual bits.
unsigned int mimo_ofdmRx_getOptions(){
	
	return ofdm_txrx_mimo_ReadReg_Rx_ControlBits(OFDM_BASEADDR);
}


///@brief Holds the OFDM Tx in reset
///
/// Holds the OFDM Tx in reset; this prevents any state changes in the Tx PHY. Configuration registers are not affected.
void mimo_ofdmTx_disable(){
	
	//Assert the OFDM Tx master reset and pktDone reset
	ofdm_txrx_mimo_WriteReg_Tx_Start_Reset_Control(OFDM_BASEADDR, 0x1 | 0x4);
	
	return;
}

///@brief Releases the OFDM Tx reset
///
/// Releases the OFDM Tx reset
void mimo_ofdmTx_enable(){
	
	//Assert then clear the OFDM Tx master reset and pktDone reset
	ofdm_txrx_mimo_WriteReg_Tx_Start_Reset_Control(OFDM_BASEADDR, 0x1 | 0x4);
	ofdm_txrx_mimo_WriteReg_Tx_Start_Reset_Control(OFDM_BASEADDR, 0x0);
	
	return;
}


///@brief Initiates the transmission of a packet
///
///Starts the transmission of a packet from the OFDM Tx PHY. If blocking is enabled, this function returns only
///after the transmission finishes. In this mode, the radio receiver is automatically re-enabled. If blocking is disabled,
///the receiver must be re-enabled in user code later.
///
///@param block Selects whether this funciton blocks until the transmission finishes; use TXBLOCK or TXNOBLOCK
///@param radio Selects which radios are enabled for the transmission
int warpphy_pktTx(unsigned int block){
	
	//First checks whether the Tx PHY is already transmitting a packet
	//If so, give up and return
	if(ofdm_txrx_mimo_ReadReg_Tx_PktRunning(OFDM_BASEADDR) == 1)
	{
		//Tx PHY was already busy sending a packet, which should be impossible
		xil_printf("Tx PHY was already transmitting! Failing...\r\n");
		
		// Return with a failure
		return 1;
	}
	
	/* Should there be a check here whether the Rx PHY is receiving a packet?
	 And what should happen if it is? It's sort of carrier sensing, but if
	 CSMA is disabled, should the Tx pkt be discarded, or held until the Rx
	 is no longer busy...
	 */
	WarpRadio_v1_TxEnable(activeRadio);
	usleep(6);//Sleep long enough for the PHY to start transmitting; depends on second argument to SetTxTiming
	
	//If the user requested blocking, wait here until the PHY is done, then re-enable the radio Rx
	// Otherwise, return immediately - NOTE! in this mode, the user must re-enable the radio Rx at some point
	if(block == TXBLOCK)
	{
		//Do nothing while the Tx PHY is still busy
		while(ofdm_txrx_mimo_ReadReg_Tx_PktRunning(OFDM_BASEADDR) == 1) {}
		
		//Re-enable the radio receiver
		WarpRadio_v1_RxEnable(activeRadio);
	}
	
	//Return successfully
	return 0;
}

///@brief Polls PHY transmitter and re-enables reception upon completion
///
///This function blocks until the transmitter is complete and then re-enables
///reception by enabing the radio receiver and enabling packet detection.
int warpphy_waitForTx(){

	//Poll the PHY transmitter until it's finished transmitting
	while(ofdm_txrx_mimo_ReadReg_Tx_PktRunning(OFDM_BASEADDR) == 1) {}

	//Re-enable the radio receiver
	WarpRadio_v1_RxEnable(activeRadio);

	//Re-enable packet detection
	ofdm_pktDetector_mimo_WriteReg_pktDet_reset(PKTDET_BASEADDR, 0);

	return 0;
}


///@brief Sets the packet buffer indicies for the OFDM Tx and Rx PHY.
///
///The PLB_BRAM used a the PHY packet buffer is large enough to hold many PHY packets.
///This BRAM is divided into many sub-buffers; the PHY can be set to use any sub-buffer for Tx or Rx.
///
///@param txBufOffset 6-bit integer selecting the sub-buffer for the PHY Tx
///@param rxBufOffset 6-bit integer selecting the sub-buffer for the PHY Rx
void warpphy_setBuffs(unsigned char txBufOffset, unsigned char rxBufOffset){
	
	//TxRx_Interrupt_PktBuf_Ctrl[21:16] compose the Rx buffer offset
	//TxRx_Interrupt_PktBuf_Ctrl[13:8]  compose the Tx buffer offset

	//First, zero out the current pkt buff offsets
	//Preserve the bottom 8 bits of the register (used for interrupt control)
	ofdm_txrx_mimo_WriteReg_TxRx_Interrupt_PktBuf_Ctrl(OFDM_BASEADDR, ofdm_txrx_mimo_ReadReg_TxRx_Interrupt_PktBuf_Ctrl(OFDM_BASEADDR) & 0x000000FF);
	
	//Then write the new pkt buff offsets
	ofdm_txrx_mimo_WriteReg_TxRx_Interrupt_PktBuf_Ctrl(OFDM_BASEADDR,
		ofdm_txrx_mimo_ReadReg_TxRx_Interrupt_PktBuf_Ctrl(OFDM_BASEADDR) |
		( (txBufOffset & 0x3F) << 8 ) |
		( (rxBufOffset & 0x3F) << 16 )
	);

	return;
}

///@brief Configures the PHY for SISO mode
void warpphy_enableSisoMode(){
	
	//Disable the TxRx paths during the switch
	WarpRadio_v1_TxRxDisable(FIRST_RADIO | SECOND_RADIO);

	//Update the global variable
	warpphy_sisoMode = 1;
	
	//Disable full-rate modulation for second anteanna
	warpphy_set_modulation(baseRateMod, 0xF, 0);

	//Enable only one radio
	activeRadio = FIRST_RADIO;
	
	//Set SISO mode in the Tx and Rx PHY registers
	mimo_ofdmTx_setControlBits(mimo_ofdmTx_getOptions() | TX_SISO_MODE);
	mimo_ofdmRx_setOptions(mimo_ofdmRx_getOptions() | RX_SISO_MODE, DEFAULT_INTERRUPTS);

	//Re-enable reception on just the first radio
	WarpRadio_v1_RxEnable(activeRadio);
	
	return;
}

///@brief Configures the PHY for MIMO mode
void warpphy_enableMimoMode(){

	//Disable the TxRx paths during the switch
	WarpRadio_v1_TxRxDisable(FIRST_RADIO | SECOND_RADIO);

	//Update the global variable
	warpphy_sisoMode = 0;
	
	//Enable full-rate modulation for both anteannas
	warpphy_set_modulation(baseRateMod, 0xF, 0xF);

	//Enable both radios
	activeRadio = FIRST_RADIO | SECOND_RADIO;
	
	//Un-set SISO mode in the Tx and Rx PHY registers
	mimo_ofdmTx_setControlBits(mimo_ofdmTx_getOptions() & (~TX_SISO_MODE));
	mimo_ofdmRx_setOptions(mimo_ofdmRx_getOptions() & (~RX_SISO_MODE),  DEFAULT_INTERRUPTS);

	//Re-enable reception on both radios
	WarpRadio_v1_RxEnable(activeRadio);

	return;
}

///@brief Configures the TX PHY for MISO mode
void warpphy_enableMisoMode(){
	WarpRadio_v1_TxRxDisable(FIRST_RADIO | SECOND_RADIO);
	warpphy_sisoMode = 0;
	lastActiveRadio = activeRadio;
	activeRadio = FIRST_RADIO | SECOND_RADIO;
	mimo_ofdmTx_setControlBits(mimo_ofdmTx_getOptions() & (~TX_SISO_MODE));
	WarpRadio_v1_RxEnable(activeRadio);
}

///@brief Configures the TX PHY for SISO mode
void warpphy_disableMisoMode(){
	WarpRadio_v1_TxRxDisable(FIRST_RADIO | SECOND_RADIO);
	warpphy_sisoMode = 1;
	activeRadio = lastActiveRadio;
	mimo_ofdmTx_setControlBits(mimo_ofdmTx_getOptions() | TX_SISO_MODE);
	WarpRadio_v1_RxEnable(activeRadio);
}

///@brief Sets the number of training symbol periods used per packet
///
///Configures the number of training symbols which are transmitted with each packet. The Tx and Rx nodes must be
///configured for the same number. In SISO mode, the single channel is trained c times. In MIMO mode, each channel
///is trained c/2 times
///
///@param c number of training periods; must be even
void warpphy_setNumTrainingSyms(unsigned int c){
	//Update the global variable; use each time a packet is transmitted
	numTrainingSyms = c;
	
	//Configure the receiver
	mimo_ofdmRx_setNumOFDMSyms(numTrainingSyms + (numBaseRate<<16));
}

///@brief Configure the flexible modulation/demodulation in the OFDM PHY
///
///The OFDM PHY supports flexible modulation, allowing any combination of schemes per subcarrier
///Currently this code supports simple dynamic modulation, with 48 of 64 subcarriers assigned to carry data, 4 for pilots and 12 empty.
///The modulation scheme in the 48 data subcarriers is set by this funciton.
///
///@param baseRate Modulation scheme for base rate symbols
///@param antAFullRate Modulation scheme for full rate symbols on antenna A
///@param antBFullRate Modulation scheme for full rate symbols on antenna B
void warpphy_set_modulation(unsigned char baseRate, unsigned char antAFullRate, unsigned char antBFullRate)
{
	unsigned int modIndex;

	//Update the global baseRate modulation variable
	baseRateMod = baseRate;
	
	//Define the standard subcarrier mapping - 48 used for data (0xF here), 4 pilots & 12 unused (0x0 here)
	// The vector contains 192 elements:
	//    0:63 - Antenna A full rate masks for subcarriers [0,1,2,...31,-31,-30,...-1]
	//  64:127 - Antenna B full rate masks for subcarriers [0,1,2,...31,-31,-30,...-1]
	// 128:191 - Base rate masks for subcarriers [0,1,2,...31,-31,-30,...-1]
	//The default masks are 3 copies of the same 64-length vector; yes, it's inefficient, but this scheme maintains flexibility in changing the mapping per antenna
	unsigned char modMasks[192] = {
			0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF,
			0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF,
			0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF
	};

	//The PHY's shared memories for modulation masks have 192 4-bit entries; each entry's address is 4-byte aligned (hence the *sizeof(int) below )
	//Configure Tx and Rx antenna A full rate
	 for(modIndex=0; modIndex<64; modIndex++)
	 {
		XIo_Out32(XPAR_OFDM_TXRX_MIMO_PLBW_0_MEMMAP_TXMODULATION+(modIndex*sizeof(int)), modMasks[modIndex] & antAFullRate);
		XIo_Out32(XPAR_OFDM_TXRX_MIMO_PLBW_0_MEMMAP_RXMODULATION+(modIndex*sizeof(int)), modMasks[modIndex] & antAFullRate);
	 }

	 //Configure Tx and Rx antenna B full rate
	 for(modIndex=64; modIndex<128; modIndex++)
	 {
		XIo_Out32(XPAR_OFDM_TXRX_MIMO_PLBW_0_MEMMAP_TXMODULATION+(modIndex*sizeof(int)), modMasks[modIndex] & antBFullRate);
		XIo_Out32(XPAR_OFDM_TXRX_MIMO_PLBW_0_MEMMAP_RXMODULATION+(modIndex*sizeof(int)), modMasks[modIndex] & antBFullRate);
	 }
	 
	 //Configure the Tx and Rx base rate
	 for(modIndex=128; modIndex<192; modIndex++)
	 {
		XIo_Out32(XPAR_OFDM_TXRX_MIMO_PLBW_0_MEMMAP_TXMODULATION+(modIndex*sizeof(int)), modMasks[modIndex] & baseRate);
		XIo_Out32(XPAR_OFDM_TXRX_MIMO_PLBW_0_MEMMAP_RXMODULATION+(modIndex*sizeof(int)), modMasks[modIndex] & baseRate);
	 }

	return;
}

///@brief Selects which antenna is used when the PHY is in SISO mode
///
/// The OFDM PHY can use either antenna when in SISO mode. By default, antenna A is used.
///
///@param anteSel 0: use Antenna A, 1: use Antenna B
void warpphy_setSISOAntenna(unsigned char antSel)
{
	
	if(warpphy_sisoMode != 1) return; //Not valid if using MIMO mode
	if(mimo_ofdmRx_getOptions()  & SWITCHING_DIV_EN) return; //Not valid if using switching diversity
	
	if(antSel == 0) //choose radio in slot 2
	{
		//Temporarily disable packet detections
		ofdm_pktDetector_mimo_WriteReg_pktDet_reset(PKTDET_BASEADDR, 1);
		
		activeRadio = FIRST_RADIO;
		
		//Switch the pkt detector to listen to just radio #2
		ofdm_pktDetector_mimo_WriteReg_pktDet_detectionMask(PKTDET_BASEADDR, 1);
		
		//Disable antB selection in the PHY
		mimo_ofdmTx_setControlBits(mimo_ofdmTx_getOptions() & (~TX_SISO_ON_ANTB) );
		mimo_ofdmRx_setOptions(mimo_ofdmRx_getOptions() & (~SISO_ON_ANTB),  DEFAULT_INTERRUPTS);
		
		//Turn packet detection back on
		ofdm_pktDetector_mimo_WriteReg_pktDet_reset(PKTDET_BASEADDR, 0);
		WarpRadio_v1_RxEnable(activeRadio);
	}
	else if(antSel == 1) //chose radio in slot 3
	{
		//Temporarily disable packet detections
		ofdm_pktDetector_mimo_WriteReg_pktDet_reset(PKTDET_BASEADDR, 1);
		
		activeRadio = SECOND_RADIO;
		
		//Switch the pkt detector to listen to just radio #3
		ofdm_pktDetector_mimo_WriteReg_pktDet_detectionMask(PKTDET_BASEADDR, 2);
		
		//Disable antB selection in the PHY
		mimo_ofdmTx_setControlBits(mimo_ofdmTx_getOptions() | (TX_SISO_ON_ANTB) );
		mimo_ofdmRx_setOptions(mimo_ofdmRx_getOptions() | (SISO_ON_ANTB),    DEFAULT_INTERRUPTS);
		
		//Turn packet detection back on
		ofdm_pktDetector_mimo_WriteReg_pktDet_reset(PKTDET_BASEADDR, 0);
		WarpRadio_v1_RxEnable(activeRadio);
	}
	
	return;
}

///@brief Set the center frequency of the radio transceivers
///@param band Selects 2.4GHz or 5GHz bands (using GHZ_2 or GHZ_5)
///@param c Selects the channel number in the chosen band
///@return Returns -1 if an invalid band or channel was specified; otherwise returns the new center frequency in MHz
int warpphy_setChannel(unsigned char band, unsigned int c){

	int newFreq = -1;
	
	
	if (band == GHZ_2){

		newFreq = WarpRadio_v1_SetCenterFreq2GHz(c, FIRST_RADIO | SECOND_RADIO);
	}
	if (band == GHZ_5){
		newFreq = WarpRadio_v1_SetCenterFreq5GHz(c, FIRST_RADIO | SECOND_RADIO);
	}
	
			xil_printf("New Frequency %d\r\n",newFreq);
	return newFreq;
}

///@brief Set the center frequency of the radio transceivers independently
///@param antA_band Selects 2.4GHz or 5GHz bands (using GHZ_2 or GHZ_5) for antenna A
///@param antB_band Selects 2.4GHz or 5GHz bands (using GHZ_2 or GHZ_5) for antenna B
///@param antA_chan Selects the channel number in the chosen band for antenna A
///@param antB_chan Selects the channel number in the chosen band for antenna B
///@return Returns -1 if an invalid band or channel was specified; otherwise returns the new center frequency for antenna A in MHz
int warpphy_setSeparateChannels(unsigned char antA_band, unsigned int antA_chan, unsigned char antB_band, unsigned int antB_chan){

	int newFreq_A = -1;
	int newFreq_B = -1;
	
	if (antA_band == GHZ_2)
		newFreq_A = WarpRadio_v1_SetCenterFreq2GHz(antA_chan, FIRST_RADIO);

	if (antB_band == GHZ_2)
		newFreq_B = WarpRadio_v1_SetCenterFreq2GHz(antB_chan, SECOND_RADIO);

	if (antA_band == GHZ_5)
		newFreq_A = WarpRadio_v1_SetCenterFreq5GHz(antA_chan, FIRST_RADIO);

	if (antB_band == GHZ_5)
		newFreq_B = WarpRadio_v1_SetCenterFreq5GHz(antB_chan, SECOND_RADIO);
	
	if(newFreq_A == -1 || newFreq_B == -1)
		return -1;
	else
		return newFreq_A;

}

/*********************************************************************/
/* A whole bunch of debugging functions - these will go away someday */
/*********************************************************************/

void warpphy_setPktDlyPlus(){
	pktDly++;
	xil_printf("+PktDly = %d\r\n",pktDly);
	mimo_ofdmRx_setPktDetDly(pktDly);
}
void warpphy_setPktDlyMinus(){
	pktDly--;
	xil_printf("-PktDly = %d\r\n",pktDly);
	mimo_ofdmRx_setPktDetDly(pktDly);
}

/****************************/
/* Phase noise tracking */
/****************************/
void warpphy_set_PN_KPlus(unsigned int increment){
	PN_KVal=PN_KVal+increment;
	mimo_ofdmRx_setPNTrack_K(PN_KVal);
	xil_printf("K: %x\r\n", PN_KVal);
}

void warpphy_set_PN_KMinus(unsigned int decrement){
	PN_KVal=PN_KVal-decrement;
	mimo_ofdmRx_setPNTrack_K(PN_KVal);
	xil_printf("K: %x\r\n", PN_KVal);
}

/*****************/
/* CFO Constants */
/*****************/

void print_CFO_constants ()
{
	xil_printf("AK: %x\tBP: %x\tBI: %x\r\n", A_KPval, B_KPval, B_KIval);
}

void warpphy_set_B_KPPlus(unsigned int increment){
	B_KPval=B_KPval+increment;
	mimo_ofdmRx_setCFO_B_KP(B_KPval);
	print_CFO_constants();
}

void warpphy_set_B_KPMinus(unsigned int decrement){
	B_KPval=B_KPval-decrement;
	mimo_ofdmRx_setCFO_B_KP(B_KPval);
	print_CFO_constants();
}

void warpphy_set_B_KIPlus(unsigned int increment){
	B_KIval=B_KIval+increment;
	mimo_ofdmRx_setCFO_B_KI(B_KIval);
	print_CFO_constants();
}

void warpphy_set_B_KIMinus(unsigned int decrement){
	B_KIval=B_KIval-decrement;
	mimo_ofdmRx_setCFO_B_KI(B_KIval);
	print_CFO_constants();
}

void warpphy_set_CFODebugOutput(unsigned char outputSel){
	
	if(outputSel)
		ofdm_txrx_mimo_WriteReg_Rx_ControlBits(OFDM_BASEADDR, ofdm_txrx_mimo_ReadReg_Rx_ControlBits(OFDM_BASEADDR) | DEBUG_CFO_OUTSEL);
	else
		ofdm_txrx_mimo_WriteReg_Rx_ControlBits(OFDM_BASEADDR, ofdm_txrx_mimo_ReadReg_Rx_ControlBits(OFDM_BASEADDR) & ~DEBUG_CFO_OUTSEL);
	
	xil_printf("CFO Debug Output %d\r\n",outputSel);
}


/*********************/
/* Other PHY options */
/*********************/

void warpphy_set_FFTOffset_Plus(){
	RxFFT_Window_Offset++;
	mimo_ofdmRx_setFFTWindowOffset(RxFFT_Window_Offset);
	xil_printf("Rx FFT Offset: %d\r\n", RxFFT_Window_Offset);
}

void warpphy_set_FFTOffset_Minus(){
	RxFFT_Window_Offset--;
	mimo_ofdmRx_setFFTWindowOffset(RxFFT_Window_Offset);
	xil_printf("Rx FFT Offset: %d\r\n", RxFFT_Window_Offset);
}

void warpphy_setNoiseTargetPlus(){
	agcnoiseEst++;
	xil_printf("+Noise Estimate = %d\r\n",agcnoiseEst);
	ofdm_AGC_setNoiseEstimate(agcnoiseEst);
}

void warpphy_setNoiseTargetMinus(){
	agcnoiseEst--;
	xil_printf("-Noise Estimate = %d\r\n",agcnoiseEst);
	ofdm_AGC_setNoiseEstimate(agcnoiseEst);
}

void warpphy_setTargetPlus(){
	agctarget++;
	xil_printf("+agctarget = %d\r\n",agctarget);
	//XGpio_mSetDataReg(XPAR_LED_7SEGMENT_BASEADDR, 1, sevenSegmentMap(-agctarget%10));
	//XGpio_mSetDataReg(XPAR_LED_7SEGMENT_1_BASEADDR, 1, sevenSegmentMap(-agctarget/10));
	ofdm_AGC_SetTarget(agctarget);
}

void warpphy_setTargetMinus(){
	agctarget=agctarget-1;
	xil_printf("-agctarget = %d\r\n",agctarget);
	//XGpio_mSetDataReg(XPAR_LED_7SEGMENT_BASEADDR, 1, sevenSegmentMap(-agctarget%10));
	//XGpio_mSetDataReg(XPAR_LED_7SEGMENT_1_BASEADDR, 1, sevenSegmentMap(-agctarget/10));
	ofdm_AGC_SetTarget(agctarget);
}


///@brief Applies TX DC offset calibration to the specified radios; the calibration values are stored in the radio's EEPROM
///@param radioSelection OR'd combinaton of RADIOx_ADDR values, specifying which radios to update
///@return Returns -1 if an EEPROM error occurs; returns 0 if successful
int warpphy_applyTxDCOCalibration(unsigned int radioSelection)
{
	int eepromStatus = 0;
	short calReadback = 0;
	signed short best_I, best_Q;
	unsigned char radioNum;
	Xuint8 memory[8], version, revision, valid, MAC[6], i;
	Xuint16 serial;

	//Radio selection will be 0x11111111, 0x22222222, 0x44444444 or 0x88888888
	// corresponding to radios in slots 1, 2, 3 or 4
	// We need the slot number to initialize the EEPROM
	radioNum = (radioSelection & 0xF) == 1 ? 1 : ( (radioSelection & 0xF) == 2 ? 2 : ( (radioSelection & 0xF) == 4 ? 3 : 4 ) );
	xil_printf("Applying TxDCO correction for radio %d\r\n", radioNum);

	//Mimic the radio test code, in hopes of a more stable EEPROM read...
	//Choose the EEPROM on the selected radio board; second arg is [0,1,2,3,4] for [FPGA, radio1, radio2, radio3, radio4]
	eepromStatus = WarpEEPROM_EEPROMSelect((unsigned int *)XPAR_EEPROM_0_MEM0_BASEADDR, 0);
	if(eepromStatus != 0)
	{
		xil_printf("EEPROM Select Failed!\r\n");
		return;
	}
	
	//Initialize the EEPROM controller
	eepromStatus = WarpEEPROM_Initialize((unsigned int *)XPAR_EEPROM_0_MEM0_BASEADDR);
	if(eepromStatus != 0)
	{
		xil_printf("EEPROM Init Returned %x\r\n", eepromStatus);
		xil_printf("EEPROM Init Failed!\r\n");
	return;
	}

	//Select the EEPROM on the current radio board
	eepromStatus = WarpEEPROM_EEPROMSelect((unsigned int*)XPAR_EEPROM_0_MEM0_BASEADDR, radioNum);

	if(eepromStatus != 0)
	{
		xil_printf("TxDCO: EEPROM error\r\n");
		return -1;
	}
	
	//Read the first page from the EERPOM
	WarpEEPROM_ReadMem((unsigned int*)XPAR_EEPROM_0_MEM0_BASEADDR, 0, 0, memory);
	version = (memory[0] & 0xE0) >> 5;
	revision = (memory[1] & 0xE0) >> 5;
	valid = memory[1] & 0x1F;

	xil_printf("\r\n\r\nEEPROM Values for Radio Board in Slot %d\r\n", radioNum);

	xil_printf("    WARP Radio Board Version %d.%d\r\n", version, revision);

	serial = WarpEEPROM_ReadWARPSerial((unsigned int*)XPAR_EEPROM_0_MEM0_BASEADDR);

	xil_printf("    Serial Number (WARP): WR-a-%05d\r\n", serial);

	WarpEEPROM_ReadDSSerial((unsigned int*)XPAR_EEPROM_0_MEM0_BASEADDR, memory);
	print("    EEPROM Hard-wired Serial Number: ");
	for(i=1;i<7;i++)
		xil_printf(" %x",memory[7-i]);
	xil_printf("\r\n\r\n");
	//Read the Tx DCO values
	calReadback = WarpEEPROM_ReadRadioCal((unsigned int*)XPAR_EEPROM_0_MEM0_BASEADDR, 2, 1);
	
	//Scale the stored values
	best_I = (signed short)(((signed char)(calReadback & 0xFF))<<1);
	best_Q = (signed short)(((signed char)((calReadback>>8) & 0xFF))<<1);
	
	xil_printf("TxDCO: Applied values to radio %d - I: %d\tQ: %d\r\n", radioNum, best_I, best_Q);
	
	//Finally, write the Tx DCO values to the DAC
	WarpRadio_v1_DACOffsetAdj(ICHAN, best_I, radioSelection);
	WarpRadio_v1_DACOffsetAdj(QCHAN, best_Q, radioSelection);
	
	return 0;
}


void warpphy_setPktDetPlus(unsigned int offset){
	pktdetthresh=pktdetthresh+offset;
	ofdm_pktDetector_mimo_WriteReg_pktDet_avgThresh(PKTDET_BASEADDR, pktdetthresh);
	xil_printf("PktDetThresh = %d\r\n",pktdetthresh);
}

void warpphy_setPktDetMinus(unsigned int offset){
	pktdetthresh=pktdetthresh-offset;
	ofdm_pktDetector_mimo_WriteReg_pktDet_avgThresh(PKTDET_BASEADDR, pktdetthresh);
	xil_printf("PktDetThresh = %d\r\n",pktdetthresh);
}


void warpphy_setCSMAPlus(unsigned int offset){
	csmathresh=csmathresh+offset;
	ofdm_pktDetector_mimo_WriteReg_csma_avgThresh(PKTDET_BASEADDR, csmathresh);
	xil_printf("CSMATHRESH = %d\r\n",csmathresh);
}

void warpphy_setCSMAMinus(unsigned int offset){
	csmathresh=csmathresh-offset;
	ofdm_pktDetector_mimo_WriteReg_csma_avgThresh(PKTDET_BASEADDR, csmathresh);
	xil_printf("CSMATHRESH = %d\r\n",csmathresh);
}


void warpphy_setGainPlus(unsigned int offset){
	txGain=txGain+offset;
	WarpRadio_v1_SetTxGainTiming(FIRST_RADIO | SECOND_RADIO, txGain, 0xF, 2);
	xil_printf("TxGain = %x\r\n",txGain);
}

void warpphy_setGainMinus(unsigned int offset){
	txGain=txGain-offset;
	WarpRadio_v1_SetTxGainTiming(FIRST_RADIO | SECOND_RADIO, txGain, 0xF, 2);
	xil_printf("TxGain = %x\r\n",txGain);
}

/******* AGC core control functions ********/
void ofdm_AGC_SetDCO(unsigned int AGCstate){
	
	// Enables DCO and DCO subtraction (correction scheme and butterworth hipass are active)
	
	unsigned int bits;
	
	bits = OFDM_AGC_MIMO_ReadReg_Bits(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR);
	
	if(AGCstate)
		bits = bits | 0x6;
	else
		bits = bits & 0x1;
	
	OFDM_AGC_MIMO_WriteReg_Bits(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, bits);
	
	return;
}

void ofdm_AGC_Reset(){
	
	// Cycle the agc's software reset port
	
	OFDM_AGC_MIMO_WriteReg_SRESET_IN(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 1);
	usleep(10);
	OFDM_AGC_MIMO_WriteReg_SRESET_IN(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 0);
	usleep(100);
	
	return;
}


void ofdm_AGC_MasterReset(){
	
	// Cycle the master reset register in the AGC and enable it
	
	OFDM_AGC_MIMO_WriteReg_AGC_EN(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 0);
	usleep(10);
	OFDM_AGC_MIMO_WriteReg_MRESET_IN(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 0);
	usleep(10);
	OFDM_AGC_MIMO_WriteReg_MRESET_IN(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 1);
	usleep(10);
	OFDM_AGC_MIMO_WriteReg_MRESET_IN(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 0);
	usleep(10);
	OFDM_AGC_MIMO_WriteReg_AGC_EN(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 1);
	
	return;
}

void ofdm_AGC_Initialize(int noise_estimate){
	
	int g_bbset = 0;
	
	// First set all standard parameters
	
	// Turn off both resets and the master enable
	OFDM_AGC_MIMO_WriteReg_AGC_EN(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 0);
	OFDM_AGC_MIMO_WriteReg_SRESET_IN(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 0);
	OFDM_AGC_MIMO_WriteReg_MRESET_IN(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 0);
	
	// An adjustment parameter
	OFDM_AGC_MIMO_WriteReg_ADJ(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 8);
	
	// Timing for the DC-offset correction
	OFDM_AGC_MIMO_WriteReg_DCO_Timing(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 0x46403003);
	
	// Initial baseband gain setting
	OFDM_AGC_MIMO_WriteReg_GBB_init(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 52);
	
	// RF gain AGCstate thresholds
	OFDM_AGC_MIMO_WriteReg_Thresholds(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 
		((AGC_THRESH_1&0xFF)<<16) +
		((AGC_THRESH_2&0xFF)<<8) + 
		 (AGC_THRESH_3&0xFF)
	);
	
	// Overall AGC timing
	OFDM_AGC_MIMO_WriteReg_Timing(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 0x9A962A28);//0x826E3C0A;
	
	// vIQ and RSSI average lengths
	OFDM_AGC_MIMO_WriteReg_AVG_LEN(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 0x10F); //103
	
	// Disable DCO, disable DCO subtraction, set filter to straight downsampling
	OFDM_AGC_MIMO_WriteReg_Bits(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, 0x0);
	
	// Compute and set the initial g_BB gain value from the noise estimate
	// The initial g_bb sets noise to -19 db, assuming 32 db RF gain
	
	g_bbset = -19 - 32 - noise_estimate;
	OFDM_AGC_MIMO_WriteReg_GBB_init(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, g_bbset);
	
	// Perform a master reset
	ofdm_AGC_MasterReset();
	
	// Agc is now reset and enabled, ready to go!
	return;
}


void ofdm_AGC_setNoiseEstimate(int noise_estimate){
	int g_bbset;
	
	g_bbset = -19 - 32 - noise_estimate;
	
	OFDM_AGC_MIMO_WriteReg_GBB_init(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, g_bbset);
	
	return;
}

unsigned int ofdm_AGC_GetGains(void){
	
	unsigned int gBB_A, gRF_A, gBB_B, gRF_B, gains;
	
	// Get the gains from the registers
	gBB_A = OFDM_AGC_MIMO_ReadReg_GBB_A(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR);
	gRF_A = OFDM_AGC_MIMO_ReadReg_GRF_A(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR);
	
	gBB_B = OFDM_AGC_MIMO_ReadReg_GBB_B(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR);
	gRF_B = OFDM_AGC_MIMO_ReadReg_GRF_B(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR);
	
	// First concatenate the two radios together, into the gRF register
	// 2 lowest bits are RF, 5 higher bits are BB, last bit is unused
	// Multiply by 2^2, shift gBB right by 2 bits
	
	gRF_A = gRF_A + (gBB_A * 4);
	gRF_B = gRF_B + (gBB_B * 4);
	
	// Multiply by 2^8 shift gRF right by 8 bits
	gains = gRF_A + (gRF_B * 256);
	
	// Returns hi[0 gBB_B g_RF_B | 0 g_BB_A g_RF_A]lo
	return gains;
}

void ofdm_AGC_SetTarget(unsigned int target){
	OFDM_AGC_MIMO_WriteReg_T_dB(XPAR_OFDM_AGC_MIMO_OPBW_0_BASEADDR, target);
	return;
}
/******* END AGC core control functions ********/



/******* WARP Timer core control functions *******/
#define BITMASK(byte,bit) (1 << ((byte * 8) + bit))

void warp_timer_start(unsigned char timer) {
	XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_W, (Xuint32)(XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_R) | BITMASK(timer,0)));
	XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_W, (Xuint32)(XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_R) & ~BITMASK(timer,0)));
}

void warp_timer_stop(unsigned char timer) {
	XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_W, (Xuint32)(XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_R) | BITMASK(timer,1)));
	XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_W, (Xuint32)(XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_R) & ~BITMASK(timer,1)));
}
	
void warp_timer_resume(unsigned char timer) {
	XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_W, (Xuint32)(XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_R) | BITMASK(timer,2)));
	XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_W, (Xuint32)(XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_R) & ~BITMASK(timer,2)));
}

void warp_timer_pause(unsigned char timer) {
	XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_W, (Xuint32)(XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_R) | BITMASK(timer,3)));
	XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_W, (Xuint32)(XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_R) & ~BITMASK(timer,3)));
}

void warp_timer_setMode(unsigned char timer, unsigned char mode) {
	if(mode == 1)
		XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_W, (Xuint32)(XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_R) | BITMASK(timer,4)));
	else
		XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_W, (Xuint32)(XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_R) & ~BITMASK(timer,4)));
}

void warp_timer_resetInterrupt(unsigned char timer) {
	XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_W, (Xuint32)(XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_R) | BITMASK(timer,5)));
	XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_W, (Xuint32)(XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_R) & ~BITMASK(timer,5)));
}

void warp_timer_resetInterrupts() {
	XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_W, (Xuint32)(XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_R) | 0x20202020));
	XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_W, (Xuint32)(XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_CONTROL_R) & ~0x20202020));
}

void warp_timer_setVal(unsigned char timer, unsigned int val) {
	if(val == 0) val = 1; // prevent timer from being set to 0, causing oscillation of interrupt trigger
	switch(timer) {
	case 0:
		XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER0_COUNTTO, (Xuint32)(val));
		return;
	case 1:
		XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER1_COUNTTO, (Xuint32)(val));
		return;
	case 2:
		XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER2_COUNTTO, (Xuint32)(val));
		return;
	case 3:
		XIo_Out32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER3_COUNTTO, (Xuint32)(val));
		return;
	}
}

unsigned int warp_timer_timeLeft(unsigned char timer) {
	switch(timer) {
	case 0:
		return XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER0_TIMELEFT);
	case 1:
		return XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER1_TIMELEFT);
	case 2:
		return XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER2_TIMELEFT);
	case 3:
		return XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER3_TIMELEFT);
	}
}

unsigned char warp_timer_getStatus(unsigned char timer) {
	return (XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_STATUS) >> (timer*8)) & 0xff;
}

unsigned char warp_timer_isExpired(unsigned char timer) {
	return (XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_STATUS) & BITMASK(timer, 0)) != 0;
}

unsigned char warp_timer_isActive(unsigned char timer) {
	return (XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_STATUS) & BITMASK(timer, 1)) != 0;
}

unsigned char warp_timer_isPaused(unsigned char timer) {
	return (XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_STATUS) & BITMASK(timer, 2)) != 0;
}

unsigned int warp_timer_getStatuses() {
	return XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_STATUS);
}

unsigned char warp_timer_getInterrupts() {
	unsigned int status = XIo_In32(XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_STATUS);
	return ((status & 0x01) | ((status >> 7) & 0x02) | ((status >> 14) & 0x04) | ((status >> 21) & 0x08));
}

#undef BITMASK
/******* END WARP Timer core control functions *******/

/******* OFDM Timer core control functions *******/
/*
//ofdm_timer_timeLeft(), ofdm_timer_setVal(val) and ofdm_timer_setMode(mode) now defined as macros in ofdm_timer_regMacros.h
void ofdm_timer_start(){
	ofdm_timer_WriteReg_timer_start(XPAR_OFDM_TIMER_OPBW_0_BASEADDR, 1);
	ofdm_timer_WriteReg_timer_start(XPAR_OFDM_TIMER_OPBW_0_BASEADDR, 0);
	return;
}

void ofdm_timer_stop(){
	ofdm_timer_WriteReg_timer_stop(XPAR_OFDM_TIMER_OPBW_0_BASEADDR, 1);
	ofdm_timer_WriteReg_timer_stop(XPAR_OFDM_TIMER_OPBW_0_BASEADDR, 0);
	return;
}

void ofdm_timer_clearInterrupt(){
	ofdm_timer_WriteReg_timer_interruptReset(XPAR_OFDM_TIMER_OPBW_0_BASEADDR, 1);
	ofdm_timer_WriteReg_timer_interruptReset(XPAR_OFDM_TIMER_OPBW_0_BASEADDR, 0);
	return;
}

void ofdm_timer_init(){
	*(unsigned int *) XPAR_OFDM_TIMER_PLBW_0_MEMMAP_TIMER_INTRLEVEL = 1; //1 = no inversion, 0 = inversion
	ofdm_timer_WriteReg_timer_countTo(XPAR_OFDM_TIMER_OPBW_0_BASEADDR, 500);
	ofdm_timer_WriteReg_timer_interruptReset(XPAR_OFDM_TIMER_OPBW_0_BASEADDR, 1);
	ofdm_timer_WriteReg_timer_interruptReset(XPAR_OFDM_TIMER_OPBW_0_BASEADDR, 0);
	
	return;
}
*/

void warp_timer_init(){
	//*(unsigned int *) XPAR_WARP_TIMER_PLBW_0_MEMMAP_TIMER_INTRLEVEL = 1; //1 = no inversion, 0 = inversion
	warp_timer_setVal(0, 500);
	warp_timer_setVal(1, 500);
	warp_timer_setVal(2, 500);
	warp_timer_setVal(3, 500);
	warp_timer_resetInterrupts();
	return;
}

/******* END OFDM Timer core control functions *******/
///@brief Polls the receiver and returns #INCOMPLETE, #GOODPACKET, or #BADPACKET
///
///This function is used by the user's good header callback to check wait until the entire
///packet is received as either good or bad.
///@return #INCOMPLETE, #GOODPACKET, or #BADPACKET
char warpphy_pollRxStatus(){

	return (0x00000003 & ofdm_txrx_mimo_ReadReg_Rx_packet_done(OFDM_BASEADDR)); //only good or bad
	
}

