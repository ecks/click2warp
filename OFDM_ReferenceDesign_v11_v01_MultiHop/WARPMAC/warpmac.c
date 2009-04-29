/*! \file warpmac.c
 \brief This framework allows for custom MAC implementations on WARP.
 
 @version 10
 @author Chris Hunter and Patrick Murphy
 
 This code is a collection of high-level and low-level functions to enable custom MAC layers on warp.
 At the highest level, these functions provide many MAC commonalities like carrier sensing and random
 exponential backoffs. At the lowest level, these functions wrap around hardware drivers to provide
 give user code access to features of the PHY, radio controller, and many other peripherals.
 */

#include "xparameters.h"
#include "xstatus.h"
#include "errno.h"
#include "xexception_l.h"
#include "stddef.h"
#include "stdio.h"
#include "string.h"
#include "xdmacentral.h"
#include "xdmacentral_l.h"
#include "xemaclite.h"
#include "xemaclite_l.h"
#include "xuartlite.h"
#include "xuartlite_l.h"
#include "xintc.h"
#include "xgpio.h"
#include "warpmac.h"
#include "ofdm_txrx_mimo_regMacros.h"
#include "ofdm_agc_mimo_regMacros.h"
#include "ofdm_pktdetector_mimo_regMacros.h"
#include "warp_timer_regMacros.h"
#include "radio_controller_basic.h"
#include "radio_controller_ext.h"
#include "radio_controller_adv.h"
#include "warp_userio.h"
#include "warpphy.h"

//Instantiates the Maccontrol control structure
Maccontrol controlStruct;

unsigned char timerIntStatus;

//Instantiates the general-purpose input-output peripheral driver
static XGpio GPIO_UserIO;

//Instantiates the Xilinx interrupt controller peripheral driver
static XIntc Intc;

//Instantiates the UART driver instance
static XUartLite UartLite;
unsigned char ReceiveBuffer[16];

//Instantiates Ethernet MAC lite
static XEmacLite EmacLiteInstance;   /* Instance of the EmacLite */
static XEmacLite_Config *EmacLiteConfigPtr;
unsigned char currentEmacBuff;

//Instantiates central DMA controller
static XDmaCentral DmaCentralInst;
static XDmaCentral_Config *DMAConfigPtr;

//Points to a user-provided Macframe where parsed header information will be copied to
Macframe* rxPacket;
//Points to a user-provided Macframe where parsed header information will be copied from
Macframe* txPacket;

//Global variable for tracking active LED outputs
unsigned int LEDHEX_Outputs;
unsigned int ledStates;
unsigned int leftHex;
unsigned int rightHex;

//"Low" LED states
unsigned int ledStatesLow[2];
unsigned char ledStatesIndexLow;
//"High" LED states
unsigned int ledStatesHigh[2];
unsigned char ledStatesIndexHigh;
//Dip-switch State
unsigned char dipswState;

//Global variable to track SISO vs. MIMO mode
unsigned char warpmac_sisoMode;

//These the addresses pointed to by these variables are set by the
//user via the callback registration functions

void (*usr_upbutton) ();
void (*usr_leftbutton) ();
void (*usr_middlebutton) ();
void (*usr_rightbutton) ();
void (*usr_badHeaderCallback) ();
void (*usr_goodHeaderCallback) ();
void (*usr_timerCallback) ();
void (*usr_emacCallback) ();
void (*usr_uartRecvCallback) ();

//Null handler. Catches calls if user fails to attach callbacks.
void nullCallback(void* param){};

///@brief Initializes the framework and all hardware peripherals.
///
///This function sets reasonable default values for many of the parameters of the MAC, configures
///interrupts and exceptions, configures Ethernet, and finally initializes the custom peripherals
///such as the radio controller, the PHY, the packet detector, and the automatic gain control block.
void warpmac_init() {
	
	xil_printf("Initializing WARPMAC v10:\r\n");

	//Initialize global variables
	LEDHEX_Outputs = 0;
	ledStates = 0;
	leftHex = 0;
	rightHex = 0;
	ledStatesLow[0] = 1;
	ledStatesLow[1] = 2;
	ledStatesIndexLow = 0;
	ledStatesHigh[0] = 4;
	ledStatesHigh[1] = 8;
	ledStatesIndexHigh = 0;
	warpmac_sisoMode = 1;

	XStatus Status;
	
	//Assign a null handler to all the interrupts; the user can re-define these later if desired
	usr_upbutton = nullCallback;
	usr_leftbutton = nullCallback;
	usr_middlebutton = nullCallback;
	usr_rightbutton = nullCallback;
	usr_badHeaderCallback = nullCallback;
	usr_goodHeaderCallback = nullCallback;
	usr_timerCallback = nullCallback;
	usr_uartRecvCallback = nullCallback;
	
	//Initialize the PHY
	warpphy_init();
	
	/*********************MAC Parameters*************************/
	xil_printf("	Initializing controlStruct...");
	controlStruct.maxReSend = 4;
	controlStruct.currBackoff = 0;
	controlStruct.maxBackoff = 5;
	controlStruct.rxBuffIndex = 2;
	warpphy_setBuffs(controlStruct.txBuffIndex, controlStruct.rxBuffIndex);
	xil_printf("complete!\r\n");
	/************************************************************/
	
	/************************UART*****************************/
	XUartLite_Initialize(&UartLite, XPAR_UARTLITE_0_DEVICE_ID);
//    XUartLite_SetSendHandler(&UartLite, warpmac_uartSendHandler, &UartLite);
//  XUartLite_SetRecvHandler(&UartLite, warpmac_uartRecvHandler, &UartLite);

    XUartLite_EnableInterrupt(&UartLite);
	/************************************************************/

	/************************USER IO*****************************/
	xil_printf("	Initializing UserIO...");
	//Initialize the UserIO GPIO core
	Status = XGpio_Initialize(&GPIO_UserIO, XPAR_USER_IO_DEVICE_ID);
	
	//We use both channels in the GPIO core- one for inputs, one for outputs
	XGpio_SetDataDirection(&GPIO_UserIO, USERIO_CHAN_INPUTS, USERIO_MASK_INPUTS);
	XGpio_SetDataDirection(&GPIO_UserIO, USERIO_CHAN_OUTPUTS, 0);
	
	//Make sure the LEDs are all off by default
	XGpio_DiscreteClear(&GPIO_UserIO, USERIO_CHAN_OUTPUTS, USERIO_MASK_OUTPUTS);
	
	//Configure & enable the GPIO interrupt output
	// The interrupt is only enabled for the GPIO input channel
    XGpio_InterruptEnable(&GPIO_UserIO, USERIO_CHAN_INPUTS);
    XGpio_InterruptGlobalEnable(&GPIO_UserIO);
	xil_printf("complete!\r\n");
	/************************************************************/
	
	/***********************Interrupts***************************/
	xil_printf("	Initializing Interrupts...");
	//Initialize the interrupt controller
	XIntc_Initialize(&Intc, INTC_DEVICE_ID);
	XIntc_SetIntrSvcOption(XPAR_XPS_INTC_0_BASEADDR,XIN_SVC_ALL_ISRS_OPTION);
	
	//Connect the interrupt controller to the interrupt handlers
	XIntc_Connect(&Intc, XPAR_XPS_INTC_0_OFDM_TXRX_MIMO_PLBW_0_RX_INT_BADPKT_INTR,(XInterruptHandler)phyRx_badHeader_int_handler, NULL); //Workaround... bad PHY state
	XIntc_Connect(&Intc, XPAR_XPS_INTC_0_USER_IO_IP2INTC_IRPT_INTR, (XInterruptHandler)userIO_int_handler, &GPIO_UserIO);
	XIntc_Connect(&Intc, XPAR_XPS_INTC_0_OFDM_TXRX_MIMO_PLBW_0_RX_INT_BADHEADER_INTR,(XInterruptHandler)phyRx_badHeader_int_handler, NULL);
	XIntc_Connect(&Intc, XPAR_XPS_INTC_0_OFDM_TXRX_MIMO_PLBW_0_RX_INT_GOODHEADER_INTR,(XInterruptHandler)phyRx_goodHeader_int_handler, NULL);
	XIntc_Connect(&Intc, XPAR_XPS_INTC_0_WARP_TIMER_PLBW_0_TIMEREXPIRE_INTR,(XInterruptHandler)timer_int_handler, NULL);
	XIntc_Connect(&Intc, XPAR_XPS_INTC_0_RS232_INTERRUPT_INTR, (XInterruptHandler)warpmac_uartRecvHandler, (void *)&UartLite);

	xil_printf("complete!\r\n");
	/************************************************************/
	
	/*************************Ethernet***************************/
	xil_printf("	Initializing Ethernet...");
	//Initialize the EMAC config struct
	EmacLiteConfigPtr = XEmacLite_LookupConfig(EMACLITE_DEVICE_ID);
	if (EmacLiteConfigPtr == NULL){
		ErrorTrap("EMAClite LookupConfig failed!\r\n");
		return;
	}
	
	Status = XEmacLite_CfgInitialize(&EmacLiteInstance, EmacLiteConfigPtr, EmacLiteConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS){
		ErrorTrap("EMAClite CfgInitialize failed!\r\n");
		return;
	}

	XEmacLite_FlushReceive(&EmacLiteInstance);
	currentEmacBuff = PING;
	xil_printf("complete!\r\n");
	/************************************************************/
	
	/**************************DMA*******************************/
	//Lookup the DMA configuration information
	DMAConfigPtr = XDmaCentral_LookupConfig(DMA_CTRL_DEVICE_ID);
	
	//Initialize the config struct
	Status = XDmaCentral_CfgInitialize(&DmaCentralInst, DMAConfigPtr, DMAConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS){
		ErrorTrap("DMA CfgInitialize failed!\r\n");
		return;
	}
	
	//Set the DMA addressing options
	// XDMC_DMACR_SOURCE_INCR_MASK - increment src address on each beat
	// XDMC_DMACR_DEST_INCR_MASK - increment dst address on each beat
	XDmaCentral_SetControl(&DmaCentralInst, XDMC_DMACR_SOURCE_INCR_MASK | XDMC_DMACR_DEST_INCR_MASK);
	/************************************************************/
	
	/*************************Ethernet***************************/
	//Start the interrupt controller and enable the interrupts
	XIntc_Start(&Intc, XIN_REAL_MODE);
	XIntc_Enable(&Intc, XPAR_XPS_INTC_0_OFDM_TXRX_MIMO_PLBW_0_RX_INT_BADPKT_INTR);
	XIntc_Enable(&Intc, XPAR_XPS_INTC_0_OFDM_TXRX_MIMO_PLBW_0_RX_INT_BADHEADER_INTR);
	XIntc_Enable(&Intc, XPAR_XPS_INTC_0_OFDM_TXRX_MIMO_PLBW_0_RX_INT_GOODHEADER_INTR);
    XIntc_Enable(&Intc, XPAR_XPS_INTC_0_USER_IO_IP2INTC_IRPT_INTR);
	XIntc_Enable(&Intc, XPAR_XPS_INTC_0_WARP_TIMER_PLBW_0_TIMEREXPIRE_INTR);
	XIntc_Enable(&Intc, XPAR_XPS_INTC_0_RS232_INTERRUPT_INTR);
	/************************************************************/
	
	/************************Exceptions**************************/
	xil_printf("	Enabling Exceptions...");
	XExc_Init();
	XExc_RegisterHandler(XEXC_ID_NON_CRITICAL_INT,(XExceptionHandler)XIntc_InterruptHandler,&Intc);
	XExc_mEnableExceptions(XEXC_NON_CRITICAL);
	xil_printf("complete!\r\n");
	/************************************************************/
	
	/*********************Final EMAC/DMA*************************/
	//Clear any stale EMAC interrupts
	XEmacLite_FlushReceive(&EmacLiteInstance);
	//Disable the EMAC interrupt output
	XEmacLite_DisableInterrupts(&EmacLiteInstance);
	//Disable Interrupts
	XDmaCentral_InterruptEnableSet(&DmaCentralInst, 0);
	/************************************************************/
	
	
	//Manually call the UserIO ISR once to store the initial value of the buttons/switches
	//This is especially important for applications where the value of the DIP switch means something at boot
	userIO_int_handler((void *)&GPIO_UserIO);
	
}

///@brief "Good Header" Interrupt handler for the PHY layer.
///
///This function is the ISR for the RX physical layer peripheral.
///This function is only called if the packet header has no bit errors
///(i.e., the packet header passes CRCs in the PHY). This interrupt will
///be asserted *before* the packet is fully received. This function assumes
///the user's callback will poll the PHY to determine if the packet is eventually
///good or bad
void phyRx_goodHeader_int_handler() {	
	unsigned int numPayloadBytes;
	int length;
	
	//Copy the received packet's header into the rxPacket struct
	memcpy((unsigned char *)(&(rxPacket->header)), (unsigned char *)warpphy_getBuffAddr(controlStruct.rxBuffIndex), (size_t)NUM_HEADER_BYTES);
	
	//Strip the header and checksum from the total number of bytes reported to user code
	if(rxPacket->header.length == NUM_HEADER_BYTES){
		//Header-only packet (like an ACK) has 0 payload bytes
		rxPacket->header.length = 0;
	}
	else{
		//Other packets have payload bytes; subtract off header and 4-byte payload checksum lengths
		rxPacket->header.length = rxPacket->header.length - NUM_HEADER_BYTES - 4;
	}
	
	//If the Rx packet is too big, ignore the payload
	// This should never happen; the Tx code will never send a pkt bigger than Ethernet MTU
	if(rxPacket->header.length>MY_XEM_MAX_FRAME_SIZE){
		rxPacket->header.length = 0;
	}
	
	//Pass the received packet to the user handler
	usr_goodHeaderCallback(rxPacket);
	
	unsigned char state = INCOMPLETE;
	//Poll PHY to insure ISR does not quit too early
				while(state==INCOMPLETE){
					//Blocks until the PHY reports the received packet as either good or bad
					state = warpphy_pollRxStatus();
				}
				
	//Clears every interrupt output of the PHY
	warpphy_clearRxInterrupts();
	XIntc_Acknowledge(&Intc,XPAR_XPS_INTC_0_OFDM_TXRX_MIMO_PLBW_0_RX_INT_GOODHEADER_INTR);
	return;
}

///@brief "Bad" Interrupt handler for the PHY layer.
///
///This function is the ISR for the RX physical layer peripheral.
///This function is only called if the packet fails CRC. The received
///packet is passed to the user's callback, and the PHY is reset.
void phyRx_badHeader_int_handler(){
	//Bad packets, by definition, have no payload bytes which can be trusted
	// The user handler is called with no arguments, since there is no real packet data to process
	// If the header happend to be good but the payload bad, the user should use the good header
	// interrupt to process the header
	usr_badHeaderCallback();
	
	//Clear the good/bad packet interrupts in the PHY to re-enable packet reception
	warpphy_clearRxInterrupts();
	
	XIntc_Acknowledge(&Intc,XPAR_XPS_INTC_0_OFDM_TXRX_MIMO_PLBW_0_RX_INT_BADHEADER_INTR);
	return;
}


///@brief Handles the reception of packets from the emac core.
///
///This function is the ISR for the EMAC. It starts a DMA transfer
///of the payload into the PHY's packet buffer and calls the user's
///callback to further processing.
///Note: The function name is a bit of a misnomer with this release
///of the reference design. Interrupts have been removed for performance
///reasons. However, in the interest of symmetry and legacy, the
///"int_handler" moniker is left in place.
void emacRx_int_handler(void *CallBackRef){
	XIntc_Acknowledge(&Intc,XPAR_XPS_INTC_0_ETHERNET_MAC_IP2INTC_IRPT_INTR);
	
	XEmacLite *EmacLiteInstPtr;
	unsigned int RxPktBaseAddress,RxPktBaseAddressPing,RxPktBaseAddressPong;
	unsigned int StatusRegister,StatusRegister2;
	unsigned short RxPktLength, RxPktLengthType;
	
	//Convert the arg to something useful
	EmacLiteInstPtr = (XEmacLite *)CallBackRef;
	
	//Lookup which EMAC Rx pkt buffer has the new packet
	RxPktBaseAddressPing = (EmacLiteInstPtr)->EmacLiteConfig.BaseAddress;
	StatusRegister = XEmacLite_mGetRxStatus(RxPktBaseAddressPing);
	
	if ((currentEmacBuff==PING) && ((StatusRegister & XEL_RSR_RECV_DONE_MASK) == XEL_RSR_RECV_DONE_MASK)) {
		RxPktBaseAddress = RxPktBaseAddressPing;
		StatusRegister &= ~XEL_RSR_RECV_DONE_MASK;
		XEmacLite_mSetRxStatus(RxPktBaseAddress, StatusRegister);
		currentEmacBuff = PONG;
	}
	
	else{
		
		RxPktBaseAddressPong = RxPktBaseAddressPing + XEL_BUFFER_OFFSET;
		StatusRegister2 = XEmacLite_mGetRxStatus(RxPktBaseAddressPong);
		
		if ((currentEmacBuff==PONG)&&((StatusRegister2 & XEL_RSR_RECV_DONE_MASK) == XEL_RSR_RECV_DONE_MASK)) {
			RxPktBaseAddress = RxPktBaseAddressPong;
			StatusRegister2 &= ~XEL_RSR_RECV_DONE_MASK;
			XEmacLite_mSetRxStatus(RxPktBaseAddress, StatusRegister2);
			currentEmacBuff= PING;
		}
		
		else{
			//Out of sync... should be corrected next packet
		}
		
		
	}
	
	//From here on out, RxPktBaseAddress is used... it is either PING or PONG
	
/*	xil_printf("IP type %d",XEL_ETHER_PROTO_TYPE_IP);
	xil_printf("ARP type %d",XEL_ETHER_PROTO_TYPE_ARP);
	xil_printf("Max Size %d",XEL_MAX_FRAME_SIZE);*/
	
	
	//Extract the EtherType field (bytes[12:13]) from the received frame
	RxPktLengthType = XEmacLite_mGetReceiveDataLength(RxPktBaseAddress);
	
	//Some Ethernet frames use bytes[12:13] for length; others use it for type
	// If the field is > 0x600, it's a type; otherwise it's a length
	// See http://en.wikipedia.org/wiki/Ethernet_II_framing for details
	// The code below is borrowed from xemaclite.c
	if (RxPktLengthType > XEL_MAX_FRAME_SIZE){	//length field is type, not length
		if (RxPktLengthType == XEL_ETHER_PROTO_TYPE_IP){
			//The packet is a an IP Packet
			// Calculate the length based on the IP header's length field
			RxPktLength = ((XEmacLite_mReadReg((RxPktBaseAddress),
											   XEL_HEADER_IP_LENGTH_OFFSET +
											   XEL_RXBUFF_OFFSET) >>
							XEL_HEADER_SHIFT) & (
												 XEL_RPLR_LENGTH_MASK_HI |
												 XEL_RPLR_LENGTH_MASK_LO));
			
	//		xil_printf("IP,%d\r\n",RxPktLength);
			RxPktLength += XEL_HEADER_SIZE + XEL_FCS_SIZE;
			
		}
		
		else if (RxPktLengthType == XEL_ETHER_PROTO_TYPE_ARP){
			//The packet is an ARP Packet
			// Update the length (every ARP pkt is the same length)
	//		xil_printf("ARP,%d\r\n",XEL_ARP_PACKET_SIZE);
			RxPktLength = XEL_ARP_PACKET_SIZE + XEL_HEADER_SIZE + XEL_FCS_SIZE;
			
		}
		
		else{
			//Type is something else (non-IP, non-ARP)
			//Punt on extracting the length here; tell the user
			// it's the full MTU and let them handle it
			RxPktLength = XEL_MAX_FRAME_SIZE;
	//		xil_printf("Non,%d\r\n",RxPktLength);
		}
	}
	else {	//Ethernet frame length field really is length
		
		//Use the length in the frame, plus the header and trailer
	//	xil_printf("Lgth,%d\r\n",RxPktLengthType);
		RxPktLength = RxPktLengthType + XEL_HEADER_SIZE + XEL_FCS_SIZE;
		
	}
	
	//Initiate the DMA transfer from EMAC rx buffer to OFDM PHY pkt buffer
	if(XDmaCentral_GetStatus(&DmaCentralInst) & XDMC_DMASR_BUSY_MASK == XDMC_DMASR_BUSY_MASK) {
		//DMA is busy cooking on something... just drop the packet
		return;
	}
	
	
	//Start the DMA
	XDmaCentral_Transfer(&DmaCentralInst, (u8 *)RxPktBaseAddress+XEL_RXBUFF_OFFSET, (u8 *)warpphy_getBuffAddr(controlStruct.txBuffIndex)+NUM_HEADER_BYTES, RxPktLength);
	
	//This is a bit fast and loose. The DMA transfer does not block, which means the payload will not be completely copied into the PHY before the user's
	//callback is executed. However, the PHY is much slower than a DMA transfer. Therefore, even if it begins transmitting, the DMA will write the last bytes
	//before the PHY even gets to them.
	
	usr_emacCallback(RxPktLength,warpphy_getBuffAddr(controlStruct.txBuffIndex)+NUM_HEADER_BYTES);
	
	return;
}

///@brief ErrorTrap
///This is a wrapper around a print statement. The purpose of this function is provide
///a single line to comment in or out to disable or enable debug messages.
void ErrorTrap(char *Message){
	xil_printf("%s\r\n", Message);
	return;
}

///@brief Interrupt handler for the User I/O.
///
///This is a low-priority interrupt because it is primarily used for debugging purposes.
///The various user callbacks are executed, depending upon which button was depressed. Additionally,
///the board's dip switches also trigger this ISR, though the framework does not currently call user
///code in this event. We assume that dipswitches are currently only used for beginning-of-time
///state assignments. This will change in future releases.
void userIO_int_handler(void *InstancePtr){
	
	static unsigned int previousAssertedInputs;
	unsigned int assertedInputs;
	unsigned int newAssertedInputs;
	
	//Re-interpret the generic input pointer as a GPIO driver instance
    XGpio *GpioPtr = (XGpio *)InstancePtr;
	
	//Disable the GPIO core's interrupt output
    XGpio_InterruptDisable(GpioPtr, USERIO_CHAN_INPUTS);
	
	//Read the GPIO inputs; each 1 is a currently-asserted input bit
	assertedInputs = XGpio_DiscreteRead(&GPIO_UserIO, USERIO_CHAN_INPUTS) & USERIO_MASK_INPUTS;
	
	//XOR the current active bits with the previously active bits
	newAssertedInputs = (assertedInputs ^ previousAssertedInputs) & assertedInputs;
	previousAssertedInputs = assertedInputs;
	
	//Check whether push buttons or DIP switch triggered the interrupt
	// We assume a user callback per button, and another for the DIP switch
	if(newAssertedInputs & USERIO_MASK_PBC) usr_middlebutton();
	if(newAssertedInputs & USERIO_MASK_PBR) usr_rightbutton();
	if(newAssertedInputs & USERIO_MASK_PBL) usr_leftbutton();
	if(newAssertedInputs & USERIO_MASK_PBU) usr_upbutton();
	if(newAssertedInputs & USERIO_MASK_DIPSW) {
		dipswState = USERIO_MAP_DIPSW(assertedInputs); 
	}
	
	//Clear, acknowledge and re-enable the GPIO interrupt output
	XGpio_InterruptClear(GpioPtr, USERIO_CHAN_INPUTS);
	XIntc_Acknowledge(&Intc, XPAR_XPS_INTC_0_USER_IO_IP2INTC_IRPT_INTR);
	XGpio_InterruptEnable(GpioPtr, USERIO_CHAN_INPUTS);
	return;
}

///@brief Interrupt handler for the timer peripheral.
///
///The timer interrupt handler gets called when a hardware timer set by the
///user expires. The user's callback is called upon the completion of this event
void timer_int_handler() {
	timerIntStatus = warp_timer_getInterrupts();
	warp_timer_resetInterrupts();
	
	//Call User Function
	usr_timerCallback(controlStruct.timerType);
	
	XIntc_Acknowledge(&Intc,XPAR_XPS_INTC_0_WARP_TIMER_PLBW_0_TIMEREXPIRE_INTR);
	return;
}

///@brief This function stops the timer.
///
///Additionally it will return the amount of time remaining before expiration.
///@return Number of clock cycles remaining before expiration.
int warpmac_clearTimer(){
	int time;
	time = warp_timer_timeLeft(0);
	warp_timer_stop(0);
	controlStruct.currBackoff=0;
	return time;
}

///@brief This function sets the timer to countdown for a given number of clock cycles.
///@param clocks Number of 40MHz clock cycles to countdown from
void warpmac_setTimerVal(Xuint32 clocks){
	//Additional factor of two comes into play due to the fact that we moved the core to the 80MHz PLB when it used to be on the 40MHz OPB
	warp_timer_setVal(0, 2*clocks);
}

///@brief This function starts the timer in either a CSMA or non-CSMA mode.
///@param mode #ENABLECSMA if automatic carrier-sense pausing is desired. #DISABLECSMA if received energy
///is to be ignored.
void warpmac_startTimer(unsigned char mode){
	warp_timer_setMode(0,mode);
	warp_timer_start(0);
}

///@brief Generates a uniform random value between [0,(2^N - 1)], where N is a positive integer
///
///Used internally by WARPMAC for random exponential backoffs.
///@param N Window size for draw of uniform random value.
unsigned int randNum(unsigned int N){
	
	if(N<6) return ((unsigned int) rand()) >> (32-(N+4)); //N+6
	else return ((unsigned int) rand()) >> (32-(10)); //N+6
	
}

///@brief Returns a value corresponding to the instantaneous channel condition {busy or idle}
///
///@return 1 if medium is idle, 0 if medium is busy
unsigned int warpmac_carrierSense(){
	
	/* Two reasons for declaring the medium busy:
	 1: The Rx PHY is actively receiving a packet.
	 This only happens if packet detection and AGC already triggered, and a
	 good/bad pkt interrupt will soon be asserted
	 2: The packet detector indicates the average RSSI has exceeded the carrier sense
	 threshold sometime in the last DIFS period
	 */
	return ofdm_pktDetector_mimo_ReadReg_pktDet_idleDifs(PKTDET_BASEADDR) && !(0x80000000 & ofdm_txrx_mimo_ReadReg_Rx_packet_done(OFDM_BASEADDR));
}

///@brief Function is responsible for high-level MAC timing
///
///This function is used by user code, and in turn calls the other timer functions.
///It is capable of initiating either a deterministic timeout, or a random backoff.
///@param type #TIMEOUT for deterministic countdown, #BACKOFF for random exponential
void warpmac_setTimer(int type){
	unsigned int setTime;
	int myRandNum;
	int retval;
	switch(type) {
		case TIMEOUT:
			controlStruct.timerType=TIMEOUT;
			setTime = controlStruct.timeout*40;
			warpmac_setTimerVal(setTime);
			warpmac_startTimer(DISABLECSMA);
			return;
			break;
		case BACKOFF:
			controlStruct.timerType=BACKOFF;
			myRandNum = randNum(controlStruct.currBackoff);
			setTime = myRandNum*controlStruct.slotTime*40;
			if(controlStruct.currBackoff < controlStruct.maxBackoff){
				controlStruct.currBackoff = controlStruct.currBackoff + 1;
			}
			/* Set the reset value of the timer (seconds * 40,000,000) */
			if(setTime != 0){
				warpmac_setTimerVal(setTime);
				warpmac_startTimer(ENABLECSMA);
				return;
			}
			if(setTime == 0){
				usr_timerCallback(controlStruct.timerType);
			}
			break;
			
	}
}

///@brief Pushes the given Macframe over ethernet - must come after warpmac_prepEmacForXmit
///@param packet Pointer to user's Macframe
///@return #XST_FAILURE or #XST_SUCCESS
int warpmac_startEmacXmit(Macframe* packet){
	u32 RegValue;
	/*
	 * Wait until the DMA transfer is done by checking the Status register
	 */
	do {
		RegValue = XDmaCentral_GetStatus(&DmaCentralInst);
	}
	while ((RegValue & XDMC_DMASR_BUSY_MASK) == XDMC_DMASR_BUSY_MASK);
	
	
	/*
	 * If Bus error occurs, return Failure.
	 */
	if (RegValue &  (XDMC_DMASR_BUS_ERROR_MASK)) {
		//	warpmac_incrementLEDLow();
		return XST_FAILURE;
	}
	
	//EMAC should have a complete packet; start the wired Tx
	u32 Register;
	//Copy back into emac;
	u32 BaseAddress;
	
	/*
	 * Determine the expected TX buffer address
	 */
	BaseAddress = XEmacLite_mNextTransmitAddr((XEmacLite *)&EmacLiteInstance);
	
	/*
	 * Determine if the expected buffer address is empty
	 */
	Register = XEmacLite_mGetTxStatus(BaseAddress);
	
	/*
	 * If the expected buffer is available, fill it with the provided data
	 * Align if necessary.
	 */
	if (XEmacLite_mIsTxDone(BaseAddress)==TRUE) {
		
		/*
		 * Switch to next buffer if configured
		 */
		if (EmacLiteInstance.EmacLiteConfig.TxPingPong != 0) {
			EmacLiteInstance.NextTxBufferToUse ^= XEL_BUFFER_OFFSET;
		}
		
		unsigned int length;
		length = (packet->header.length)-4;
		/*
		 * The frame is in the buffer, now send it
		 */
		XEmacLite_mWriteReg(BaseAddress, XEL_TPLR_OFFSET,
							(length & (XEL_TPLR_LENGTH_MASK_HI |
									   XEL_TPLR_LENGTH_MASK_LO)));
		
		/*
		 * Update the Tx Status Register to indicate that there is a
		 * frame to send.
		 * If the interrupts are enabled then set the
		 * XEL_TSR_XMIT_ACTIVE_MASK flag which is used by the interrupt
		 * handler to call the callback function provided by the user
		 * to indicate that the frame has been transmitted.
		 */
		Register = XEmacLite_mGetTxStatus(BaseAddress);
		Register |= XEL_TSR_XMIT_BUSY_MASK;
		if ((Register & XEL_TSR_XMIT_IE_MASK) != 0) {
			Register |= XEL_TSR_XMIT_ACTIVE_MASK;
		}
		XEmacLite_mSetTxStatus(BaseAddress, Register);
	}
	return XST_SUCCESS;
}

///@brief Starts a DMA transfer from the RX PHY to the EMAC for Ethernet transmission
///@param packet Pointer to user's Macframe
void warpmac_prepEmacForXmit(Macframe* packet){
	unsigned int length;
	length = (packet->header.length)-4;
	u32 BaseAddress,RegValue;
	BaseAddress = XEmacLite_mNextTransmitAddr(&EmacLiteInstance);
	
	
	RegValue = XDmaCentral_GetStatus(&DmaCentralInst);
	
	/*
	 * If Bus error occurs, return Failure.
	 */
	if (RegValue &  (XDMC_DMASR_BUS_ERROR_MASK)) {
		ErrorTrap("Bus Error\r\n");
	}
	XDmaCentral_Transfer(&DmaCentralInst, (u8 *)(warpphy_getBuffAddr(controlStruct.rxBuffIndex)+NUM_HEADER_BYTES), (u8 *)BaseAddress, length);
	
}

///@brief Attaches user callback to up button
///@param handler pointer to user's callback
void warpmac_setUpButtonCallback(void(*handler)()){
	usr_upbutton = handler;
}

///@brief Attaches user callback to left button
///@param handler pointer to user's callback
void warpmac_setLeftButtonCallback(void(*handler)()){
	usr_leftbutton = handler;
}

///@brief Attaches user callback to right button
///@param handler pointer to user's callback
void warpmac_setRightButtonCallback(void(*handler)()){
	usr_rightbutton = handler;
}

///@brief Attaches user callback to middle button
///@param handler pointer to user's callback
void warpmac_setMiddleButtonCallback(void(*handler)()){
	usr_middlebutton = handler;
}

///@brief Attaches user callback to timer
///@param handler pointer to user's callback
void warpmac_setTimerCallback(void(*handler)()){
	usr_timerCallback = handler;
}


///@brief Attaches user callback to the Emac
///@param handler pointer to user's callback
void warpmac_setEmacCallback(void(*handler)()){
	usr_emacCallback = handler;
}


///@brief Attaches user callback to PHY's good header interrupt
///@param handler pointer to user's callback
void warpmac_setGoodHeaderCallback(void(*handler)()){
	usr_goodHeaderCallback = handler;
}

///@brief Attaches user callback to PHY's bad packet interrupt
///@param handler pointer to user's callback
void warpmac_setBadHeaderCallback(void(*handler)()){
	usr_badHeaderCallback = handler;
}


///@brief Alternates the bottom two LEDs on the WARP board.
void warpmac_incrementLEDLow(){
	
	
	//Update the global variable we use to track which LED/segments are currently lit
	// The xps_gpio core doesn't allow outputs to be read from code, so we have to track this internally
	LEDHEX_Outputs = (USERIO_MAP_LEDS( (ledStatesLow[ledStatesIndexLow]|ledStatesHigh[ledStatesIndexHigh])) | USERIO_MAP_DISPR(rightHex) | USERIO_MAP_DISPL(leftHex));
	
	
	XGpio_DiscreteSet(&GPIO_UserIO, USERIO_CHAN_OUTPUTS, LEDHEX_Outputs);
	ledStatesIndexLow = (ledStatesIndexLow+1)%2;
}


///@brief Alternates the top two LEDs on the WARP board.
void warpmac_incrementLEDHigh(){
	
	//Update the global variable we use to track which LED/segments are currently lit
	// The xps_gpio core doesn't allow outputs to be read from code, so we have to track this internally
	LEDHEX_Outputs = (USERIO_MAP_LEDS( (ledStatesLow[ledStatesIndexLow]|ledStatesHigh[ledStatesIndexHigh])) | USERIO_MAP_DISPR(rightHex) | USERIO_MAP_DISPL(leftHex));
	
	XGpio_DiscreteSet(&GPIO_UserIO, USERIO_CHAN_OUTPUTS, LEDHEX_Outputs);
	
	ledStatesIndexHigh = (ledStatesIndexHigh+1)%2;
}

///@brief Sets the base rate modulation order
///
///The base rate symbols (i.e. the header) must have an agreed upon rate between the transmitter
///and receiver. This function sets that rate.
///@param rate #BPSK, #QPSK, #QAM_16, or #QAM_64
void warpmac_setBaseRate(unsigned char rate){
	warpphy_set_modulation(rate,0xF,0);
}


///@brief Loads a header into the PHY, but does not immediately transmit
///
///This function performs the conversion from the packet structure to the byte array
///but does not send the packet over the air. This is used to "preload" and ACK
///into the PHY while the data packet is still being received. This extra 
///pipelining saves on turn-around time.
///@param packet Pointer to user's Macframe
///@param buffer Packet buffer to send from
void warpmac_prepPhyForXmit(Macframe* packet,unsigned char buffer){
	int i;
	unsigned int numPayloadBytes;
	unsigned int numFullRate;
	unsigned int numBaseRate = 2;
	unsigned int numTrainingSyms = 2;
	unsigned int bytesPerSymbol;
	int txResult;
	
	
	if(packet->header.length == 0) { //All-header packet, like an ACK, has only base-rate symbols
		//All-header packet, like an ACK, has only base-rate symbols
		numFullRate = 0;
		
		//Update the PHY header length to consist of just the header
		packet->header.length = NUM_HEADER_BYTES;
	}
	else
	{
		if(warpmac_sisoMode)
		{
			//Extract just antenna A modulation and multiply by 6 (48 subcarriers / 8 bytes/bit)
			//FIXME: someday the "6" here should be a variable determined by the actual subcarrier masks
			bytesPerSymbol = 6 * (0xF & packet->header.fullRate);
			//xil_printf("SISO bytesPerSymbol: %d\r\n", bytesPerSymbol);

			//In SISO mode, numFullRate must be doubled to accomodate the PHY's control system
			// Basic calculation is 2*ceil(numBytes / bytesPerSymbol)
			// Where numBytes is payload length + 4 checksum bytes inserted by PHY
			// In place of ceil(), we do (numBytes + (bytesPerSymbol-1)) / bytesPerSymbol
			numFullRate = ( (packet->header.length + 4 + bytesPerSymbol - 1) / bytesPerSymbol ) * 2;
			//xil_printf("SISO numFullRate: %d\r\n", numFullRate);
		}
		else//mimoMode
		{
			//Average bytes per symbol across both antennas
			bytesPerSymbol = ( (6 * (0xF & packet->header.fullRate)) + (6 * (0xF & (packet->header.fullRate>>4)))) / 2;

			//In MIMO mode, this value is written as-is (no doubling like SISO mode)
			//For unfortunate control reasons, numFullRate must be even
			numFullRate = ( (packet->header.length + 4 + bytesPerSymbol - 1) / bytesPerSymbol );
			numFullRate = numFullRate + (numFullRate%2);
			//xil_printf("MIMO numFullRate: %d\r\n", numFullRate);
		}

		//Update the header's numBytes field to include the header bytes
		packet->header.length = NUM_HEADER_BYTES + 4 + packet->header.length;
	}

	//numSyms register has three fields:
	// bits[7:0] = number of training symbol periods (fixed)
	// bits[15:8] = number of base rate data symbols (fixed)
	// bits[23:16] = number of full rate data symbols (varries with each pkt)
	warpphy_setNumSyms(numTrainingSyms + (numBaseRate<<8) + (numFullRate<<16));
	
	//Copy the header to the PHY packet buffer, filling exactly NUM_HEADER_BYTES bytes
	// The rest of the packet (its payload) will be copied by DMA later
	memcpy( (void *) warpphy_getBuffAddr(buffer), (void *) packet, (size_t) NUM_HEADER_BYTES);
	
	//Revert the packet payload length so the user code doesn't see the MAC/PHY tweaking
	if(packet->header.length == NUM_HEADER_BYTES){
		packet->header.length = 0;
	}
	else{
		packet->header.length = packet->header.length - (NUM_HEADER_BYTES + 4);
	}
	
	
	return;
}

///@brief Sends the current txBuffer's content.
///
///This function sends an existing Macframe over the air. This existing Macframe comes from the warpmac_prepPhyForXmit 
///function.
void warpmac_startPhyXmit(unsigned char buffer){
	
	int txResult;
	
	//Hold the packet detecter in reset to prevent any interfering receptions
	ofdm_pktDetector_mimo_WriteReg_pktDet_reset(PKTDET_BASEADDR, 1);

	//Transmit the packet; WARPPHY disables the radio Rx, enables Tx, sends the packet, then re-enables radio Rx
	// TXBLOCK will cause this call to block until the transmission is finished
	warpphy_setBuffs(buffer, controlStruct.rxBuffIndex);
	txResult = warpphy_pktTx(TXNOBLOCK);
	
	if(txResult != 0)
	{
		ErrorTrap("WARPMAC tried to Tx, but PHY was busy\r\n");
	}
	
	//Re-enable the packet detector so future Rx packets will be processed
	//ofdm_pktDetector_mimo_WriteReg_pktDet_reset(PKTDET_BASEADDR, 0);
	
	return;
}

///@brief Blocks on PHY transmission
///
///This function waits for the PHY to finish transmitting, then re-enables wireless reception and sets packet buffers back
///to their default values.
void warpmac_finishPhyXmit(){
	warpphy_waitForTx();
	warpphy_setBuffs(controlStruct.txBuffIndex, controlStruct.rxBuffIndex);
}




///@brief Tells the PHY which piece of memory to receive to
///
///Also, it updates the global struct element Maccontrol#rxBuffIndex to keep track of that information*/
void warpmac_setRxBuffer(Macframe* packet, unsigned int rxBuff){
	rxPacket=packet;
	controlStruct.rxBuffIndex = rxBuff;
	warpphy_setBuffs(controlStruct.txBuffIndex, controlStruct.rxBuffIndex);
	return;
}

///@brief Tells the PHY which piece of memory to send from
///
///Also, it updates the global struct element Maccontrol#txBuffIndex to keep track of that information*/
void warpmac_setTxBuffer(unsigned int txBuff){
	controlStruct.txBuffIndex = txBuff;
	warpphy_setBuffs(controlStruct.txBuffIndex, controlStruct.rxBuffIndex);
	return;
}

///@brief Returns which buffer PHY is set to receive to
///
///@return index of PHY's buffer
int warpmac_getRxBuffer(){
	return (controlStruct.rxBuffIndex);
}


///@brief Returns which buffer PHY is set to send from
///
///@return index of PHY's buffer
int warpmac_getTxBuffer(){
	return (controlStruct.txBuffIndex);
}


///@brief Increments the resend counter for the given Macframe
///
///Also, it returns whether or not the counter has wrapped around the maximum
///number of retransmits
///@param packet Pointer to user's Macframe
///@return 0 if maximum number of retransmits has been reached, and 1 otherwise
int warpmac_incrementResend(Macframe* packet){
	int status = 1;
	packet->header.currReSend = ( packet->header.currReSend+1)%((controlStruct.maxReSend)+1); //increment currReSend and wrap around max
	if( packet->header.currReSend == 0) { //if reSend wraps around, the packet will be dropped
		status = 0;
		controlStruct.currBackoff=0;
	}
	return status;
}

///@brief Returns a value corresponding to whether the packet was destined for this particular node.
///
///@param packet Pointer to user's Macframe
///@return 1 if destination of packet was this node, 0 otherwise
int warpmac_addressedToMe(Macframe* packet){
	int i,sum;
	unsigned char tempBoolean;
	sum = 0;
	for(i=0;i<6;i++){
		sum = sum+(packet->header.destAddr[i] == controlStruct.selfAddr[i]);
		//xil_printf("%d ",packet->header.destAddr[i]);
		//xil_printf("%d\r\n",controlStruct.selfAddr[i]);
	}
	
	tempBoolean = 0;
	if(sum==6) tempBoolean = 1;
	sum = 0;
	return tempBoolean;
}

///@brief Returns a value corresponding to whether the packet was sourced from a given node.
///
///@param packet Pointer to user's Macframe
///@param addr Pointer to char array containing MAC address to compare against
///@return 1 if destination of packet was sourced from the given address, 0 otherwise
int warpmac_addressedFromThem(Macframe* packet,unsigned char* addr){
	int i,sum;
	unsigned char tempBoolean;
	sum = 0;
	for(i=0;i<6;i++){
		sum = sum+(packet->header.srcAddr[i] == addr[i]);
	}
	tempBoolean = 0;
	if(sum==6) tempBoolean = 1;
	sum = 0;
	
	return tempBoolean;
}

///@brief Returns a value corresponding to whether the packet was destined for a given node.
///
///@param packet Pointer to user's Macframe
///@param addr Pointer to char array containing MAC address to compare against
///@return 1 if destination of packet was sourced from the given address, 0 otherwise
int warpmac_addressedToThem(Macframe* packet,unsigned char* addr){
	int i,sum;
	unsigned char tempBoolean;
	sum = 0;
	for(i=0;i<6;i++){
		sum = sum+(packet->header.destAddr[i] == addr[i]);
	}
	tempBoolean = 0;
	if(sum==6) tempBoolean = 1;
	sum = 0;
	return tempBoolean;
}

///@brief Sets the maximum number of resends
///
///@param c Integer maximum number of resends
void warpmac_setMaxResend(unsigned int c){
	controlStruct.maxReSend=c;
}

///@brief Sets the maximum contention window.
///
///@param c Maximum contention window
void warpmac_setMaxCW(unsigned int c){
	controlStruct.maxBackoff=c;
}

///@brief Sets wireless MAC address of this node.
///
///@param addr Pointer to char array containing MAC address
void warpmac_setMacAddr(unsigned char* addr){
	controlStruct.selfAddr[5] = addr[5];
	controlStruct.selfAddr[4] = addr[4];
	controlStruct.selfAddr[3] = addr[3];
	controlStruct.selfAddr[2] = addr[2];
	controlStruct.selfAddr[1] = addr[1];
	controlStruct.selfAddr[0] = addr[0];
}

///@brief Sets the amount of time the node is willing to wait for an acknowledgement.
///
///@param time Timeout duration (in microseconds)
void warpmac_setTimeout(unsigned int time){
	controlStruct.timeout = time;
}

///@brief Sets the smallest backoff window.
///
///@param time Slot time duration (in microseconds)
void warpmac_setSlotTime(unsigned int time){
	controlStruct.slotTime = time;
}

///@brief Returns a value corresponding to whether or not the node is in timeout.
///
///@return 1 if in #TIMEOUT, 0 otherwise
int warpmac_inTimeout(){
	return controlStruct.timerType==TIMEOUT;
}

///@brief Reads the value from the user dip switches for use as node identification.
///
///@return Value currently set on dip switches of the WARP board
int warpmac_getMyId(){
	
	warpmac_rightHex(dipswState);
	return dipswState;
}

///@brief Lowers carrier sense threshold such that CSMA state is enabled
void warpmac_enableCSMA(){
	ofdm_pktDetector_mimo_WriteReg_csma_avgThresh(PKTDET_BASEADDR, 9000);
	return;
}

///@brief Raises carrier sense threshold such that CSMA state is disabled
void warpmac_disableCSMA(){
	ofdm_pktDetector_mimo_WriteReg_csma_avgThresh(PKTDET_BASEADDR, 16000);
	return;
}

///@brief Maps character to the seven segment display
///
///@param x Input character
///@return Corresponding value that can be written to the GPIO connected to the Hex displays
unsigned char sevenSegmentMap(unsigned char x){
	switch(x)
	{
		case(0x0) : return 0x007E;
		case(0x1) : return 0x0030;
		case(0x2) : return 0x006D;
		case(0x3) : return 0x0079;
		case(0x4) : return 0x0033;
		case(0x5) : return 0x005B;
		case(0x6) : return 0x005F;
		case(0x7) : return 0x0070;
		case(0x8) : return 0x007F;
		case(0x9) : return 0x007B;
			
		case(0xA) : return 0x0077;
		case(0xB) : return 0x007F;
		case(0xC) : return 0x004E;
		case(0xD) : return 0x007E;
		case(0xE) : return 0x004F;
		case(0xF) : return 0x0047;
		default : return 0x0000;
	}
}

///@brief Displays the input character on the left hex display
///
///@param x Character to display
void warpmac_leftHex(unsigned char x){
	
	
	leftHex = sevenSegmentMap(x);
	
	//Update the global variable we use to track which LED/segments are currently lit
	// The xps_gpio core doesn't allow outputs to be read from code, so we have to track this internally
	LEDHEX_Outputs = (USERIO_MAP_LEDS( (ledStatesLow[ledStatesIndexLow]|ledStatesHigh[ledStatesIndexHigh])) | USERIO_MAP_DISPR(rightHex) | USERIO_MAP_DISPL(leftHex));
	
	
	XGpio_DiscreteSet(&GPIO_UserIO, USERIO_CHAN_OUTPUTS, LEDHEX_Outputs);
	
	return;
}

///@brief Displays the input character on the right hex display
///
///@param x Character to display
void warpmac_rightHex(unsigned char x){
	
	rightHex = sevenSegmentMap(x);	
	//Update the global variable we use to track which LED/segments are currently lit
	// The xps_gpio core doesn't allow outputs to be read from code, so we have to track this internally
	LEDHEX_Outputs = (USERIO_MAP_LEDS( (ledStatesLow[ledStatesIndexLow]|ledStatesHigh[ledStatesIndexHigh])) | USERIO_MAP_DISPR(rightHex) | USERIO_MAP_DISPL(leftHex));
	XGpio_DiscreteSet(&GPIO_UserIO, USERIO_CHAN_OUTPUTS, LEDHEX_Outputs);
	return;
}


///@brief Raises a signal that is routed out to the debug header on the board
///
///@param val 4-bit input that toggles
void warpmac_setDebugGPIO(unsigned char val){
	XGpio_mSetDataReg(XPAR_DEBUGOUTPUTS_BASEADDR, 1, val);
	
	return;
}

///@brief Enabled the Ethernet
void warpmac_enableEthernet(){
	controlStruct.enableEthernetInt = 1;
//	XIntc_Enable(&Intc, XPAR_XPS_INTC_0_ETHERNET_MAC_IP2INTC_IRPT_INTR);
	return;
}




///@brief Disables the Ethernet
void warpmac_disableEthernet(){
	controlStruct.enableEthernetInt = 0;
//	XIntc_Disable(&Intc, XPAR_XPS_INTC_0_ETHERNET_MAC_IP2INTC_IRPT_INTR);
	return;
}

///@brief Polls the Ethernet
///
///Function checks the status of the ping and pong Emac buffers
///and calls the effective-ISR when either is filled.
void warpmac_pollEthernet(){
	if(controlStruct.enableEthernetInt){
		
		unsigned int RxPktBaseAddressPing = XPAR_ETHERNET_MAC_BASEADDR;
		unsigned int RxPktBaseAddressPong = XPAR_ETHERNET_MAC_BASEADDR+XEL_BUFFER_OFFSET;
		
		unsigned int pingStatus = XEmacLite_mGetRxStatus(RxPktBaseAddressPing);
		unsigned int pongStatus = XEmacLite_mGetRxStatus(RxPktBaseAddressPong);
		
		
		if((pingStatus&1)||(pongStatus&1)){
		
			XIntc_Disable(&Intc, XPAR_XPS_INTC_0_OFDM_TXRX_MIMO_PLBW_0_RX_INT_BADPKT_INTR);
			XIntc_Disable(&Intc, XPAR_XPS_INTC_0_OFDM_TXRX_MIMO_PLBW_0_RX_INT_BADHEADER_INTR);
			XIntc_Disable(&Intc, XPAR_XPS_INTC_0_OFDM_TXRX_MIMO_PLBW_0_RX_INT_GOODHEADER_INTR);
			XIntc_Disable(&Intc, XPAR_XPS_INTC_0_USER_IO_IP2INTC_IRPT_INTR);
			XIntc_Disable(&Intc, XPAR_XPS_INTC_0_WARP_TIMER_PLBW_0_TIMEREXPIRE_INTR);
		
			emacRx_int_handler(&EmacLiteInstance);
		
			XIntc_Enable(&Intc, XPAR_XPS_INTC_0_OFDM_TXRX_MIMO_PLBW_0_RX_INT_BADPKT_INTR);
			XIntc_Enable(&Intc, XPAR_XPS_INTC_0_OFDM_TXRX_MIMO_PLBW_0_RX_INT_BADHEADER_INTR);
			XIntc_Enable(&Intc, XPAR_XPS_INTC_0_OFDM_TXRX_MIMO_PLBW_0_RX_INT_GOODHEADER_INTR);
			XIntc_Enable(&Intc, XPAR_XPS_INTC_0_USER_IO_IP2INTC_IRPT_INTR);
			XIntc_Enable(&Intc, XPAR_XPS_INTC_0_WARP_TIMER_PLBW_0_TIMEREXPIRE_INTR);
			
		}
		
		
	}
	return;
}

void warpmac_enableMimoMode(){
	
	//Update WARPMAC's global variable
	warpmac_sisoMode = 0;

	//Call WARPPHY's function to configure PHY MIMO mode
	warpphy_enableMimoMode();

	return;
}

void warpmac_enableSisoMode(){
	
	//Update WARPMAC's global variable
	warpmac_sisoMode = 1;

	//Call WARPPHY's function to configure PHY MIMO mode
	warpphy_enableSisoMode();

	return;
}

void warpmac_enableMisoMode(){
	warpmac_sisoMode = 0;
	warpphy_enableMisoMode();
}

void warpmac_disableMisoMode(){
	warpmac_sisoMode = 1;
	warpphy_disableMisoMode();
}

//void warpmac_uartRecvHandler(void *CallBackRef, unsigned int EventData)
void warpmac_uartRecvHandler(void *CallBackRef)
{

	XUartLite *InstancePtr = CallBackRef;
	
	unsigned int uartIntStatus;
	unsigned char receivedByte;
	
	uartIntStatus = XUartLite_mReadReg(InstancePtr->RegBaseAddress, XUL_STATUS_REG_OFFSET);

	if( (uartIntStatus & (XUL_SR_RX_FIFO_FULL |	XUL_SR_RX_FIFO_VALID_DATA)) != 0)
	{
		receivedByte = XUartLite_RecvByte(InstancePtr->RegBaseAddress);
		
		//Call the user callback, passing the byte that was received
		usr_uartRecvCallback(receivedByte);
	}

	return;
}

///@brief Attaches user callback to UART receive interrupt
///@param handler pointer to user's callback
void warpmac_setUartRecvCallback(void(*handler)()){
	usr_uartRecvCallback = handler;
}

