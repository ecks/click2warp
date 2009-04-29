/*! \file csmaMac.c
 \brief Carrier-Sensing Random Access MAC.
 
 @version 10
 @author Chris Hunter
 
 The csmaMac is a modified ALOHA MAC that
 serves as an example for novel MAC
 development. Nodes transmit whenever
 they have information to transmit, and only
 move on to the next packet once the original
 transmit is acknowledged (ACKed). If no ACK
 is received, a collision is inferred and the
 packet is re-transmitted.
 
 By default, the MAC also implements carrier-
 sensing multiple-access with collision-
 avoidance (CSMA-CA). This functionality is
 built into hardware peripherals in the project
 so very little software state is affected.
 
 In its current state, the project acts as
 a true ethernet MAC-level wireless bridge.
 Any ethernet activity that appears on one
 WARP will be sent to another via the custom
 wireless link.
 */


#include "xtime_l.h"
#include "warpmac.h"
#include "warpphy.h"
#include "csmaMac.h"
#include "xparameters.h"
#include "string.h"
#include "errno.h"
#include "stdlib.h"
#include "stdio.h"



///Routing table with agreed upon mapping between dipswitches and physical addresses
typedef struct {
	unsigned char IPaddr[4];
	unsigned char MACaddr[6];
	unsigned char Next_HOP[6];
	unsigned char num_hops;
	XTime timestamp;
} route;

///Instance of the routing table
route routeTable[5];

///Byte array containing physical MAC address of this node
unsigned char myAddr[6];
///Index to the routing table that identifies this node
unsigned char myID;
unsigned char myIP[4];
///Full rate modulation selection; QPSK by default
unsigned int pktFullRate;
unsigned int not_yet_set=1;
///Buffer for holding a packet-to-xmit across multiple retransmissions
Macframe txBuffer;
///Buffer to hold received packet
Macframe rxBuffer;

///Current antenna selection
unsigned char currentAnt = 0;
///Current 802.11 channel
unsigned char chan = 15;

unsigned char broadcast_address[6]={255,255,255,255,255,255};
unsigned char broadcast_address2[4]={10,0,0,255};
///Data packet with payload meant for Ethernet transmission
#define DATAPACKET 1
///Acknowledgement packet meant for halting retransmissions
#define ACKPACKET 0
///Advertisment packet meant for advertising routes
#define ADVPACKET 2

#define ROUTE_EXPIRATION 82

///@brief Callback for the depression of the left push button
///
///This button selects the left radio (slot 2) for Tx and Rx
/* antenna switching version
void left(){
	currentAnt = 0;
	warpphy_setSISOAntenna(0);
	warpmac_leftHex(0);
}
*/
void printRouteTable(){
	int i;
	XTime time;
	XTime_GetTime(&time);
	xil_printf("Route Table:\r\n");
	for(i=0;i<5;i++){
		if(routeTable[i].timestamp+(XTime)ROUTE_EXPIRATION*160000000>=time){
			xil_printf("%d.%d.%d.%d ",routeTable[i].IPaddr[0],routeTable[i].IPaddr[1],routeTable[i].IPaddr[2],routeTable[i].IPaddr[3]);
			xil_printf(" %x.%x.%x.%x.%x.%x ",routeTable[i].MACaddr[0],routeTable[i].MACaddr[1],routeTable[i].MACaddr[2],routeTable[i].MACaddr[3],routeTable[i].MACaddr[4],routeTable[i].MACaddr[5]);
			xil_printf(" VIA :%x.%x.%x.%x.%x.%x ", routeTable[i].Next_HOP[0],routeTable[i].Next_HOP[1],routeTable[i].Next_HOP[2],routeTable[i].Next_HOP[3],routeTable[i].Next_HOP[4],routeTable[i].Next_HOP[5]);
			xil_printf(" Num Hops = %d\r\n",routeTable[i].num_hops);
		}
	}
	return;
}
void left(){
	
/*	switch(pktFullRate)
	{
		case HDR_FULLRAATE_BPSK:
			pktFullRate = HDR_FULLRAATE_BPSK;
			xil_printf("Tx: BPSK\r\n");
		break;
		case HDR_FULLRAATE_QPSK:
			pktFullRate = HDR_FULLRAATE_BPSK;
			xil_printf("Tx: BPSK\r\n");
		break;
		case HDR_FULLRAATE_QAM_16:
			pktFullRate = HDR_FULLRAATE_QPSK;
			xil_printf("Tx: QPSK\r\n");
		break;
		case HDR_FULLRAATE_QAM_64:
			pktFullRate = HDR_FULLRAATE_QAM_16;
			xil_printf("Tx: 16-QAM\r\n");
		break;
		default:
			pktFullRate = HDR_FULLRAATE_QPSK;
			xil_printf("Tx: QPSK\r\n");
		break;
	}*/
	printRouteTable();
}

///@brief Callback for the depression of the right push button
///
///This button selects the right radio (slot 3) for Tx and Rx
/* Antenna switching version
void right(){
	currentAnt = 1;
	warpphy_setSISOAntenna(1);
	warpmac_leftHex(1);
}
*/

void right(){
/*	switch(pktFullRate)
	{
		case HDR_FULLRAATE_BPSK:
			pktFullRate = HDR_FULLRAATE_QPSK;
			xil_printf("Tx: QPSK\r\n");
		break;
		case HDR_FULLRAATE_QPSK:
			pktFullRate = HDR_FULLRAATE_QAM_16;
			xil_printf("Tx: 16-QAM\r\n");
		break;
		case HDR_FULLRAATE_QAM_16:
			pktFullRate = HDR_FULLRAATE_QAM_64;
			xil_printf("Tx: 64-QAM\r\n");
		break;
		case HDR_FULLRAATE_QAM_64:
			pktFullRate = HDR_FULLRAATE_QAM_64;
			xil_printf("Tx: 64-QAM\r\n");
		break;
		default:
			pktFullRate = HDR_FULLRAATE_QPSK;
			xil_printf("Tx: QPSK\r\n");
		break;
	}*/
}

///@brief Callback for the depression of the up push button
///
///This button increments the 2.4GHz channel being used
void up(){
	chan = chan+1;
	warpphy_setChannel(GHZ_2, chan);
	warpmac_leftHex(chan);
}

///@brief Callback for the depression of the middle push button
///
///This button decrements the 2.4GHz channel being used
void middle(){
	chan = chan-1;
	warpphy_setChannel(GHZ_2, chan);
	warpmac_leftHex(chan);
}

///@brief Callback for the expiration of timers
///
///This function is responsible for handling #TIMEOUT and #BACKOFF.
///The job responsibilities of this function are to:
///-increase the contention window upon the expiration of a #TIMEOUT
///-initiate a #BACKOFF timer upon the expiration of a #TIMEOUT
///-retransmit a packet upon the expiration of a #BACKOFF
///@param timerType #TIMEOUT or #BACKOFF
void timer_callback(unsigned char timerType){
	int status;
	switch(timerType){
		case TIMEOUT:
	//		xil_printf("T\r\n");
			status = warpmac_incrementResend(&txBuffer);
			if(status == 0){
				//The packet has been resent too many times, so we are going to re-enable Ethernet and overwrite
				warpmac_enableEthernet();
		//		xil_printf("D\r\n");
				return;
			}
			
			//Start a backoff timer
			warpmac_setTimer(BACKOFF);
		//	xil_printf("T");
		break;
		case BACKOFF:
		//	xil_printf("B\r\n");
			//Copy the header over to packet buffer 1
			warpmac_prepPhyForXmit(&txBuffer,1);
			//Send packet buffer 1
			warpmac_startPhyXmit(1);
			//Wait for it to finish and enable the receiver
			warpmac_finishPhyXmit();
			//Start a timeout timer
			if(!warpmac_addressedToThem(&txBuffer,broadcast_address))
				warpmac_setTimer(TIMEOUT);
			else
				warpmac_enableEthernet();
		break;
	}
}


///@brief Callback for the reception of Ethernet packets
///
///This function is called by the ethernet MAC drivers
///when a packet is available to send. This function fills
///the Macframe transmit buffer with the packet and sends
///it over the OFDM link
///@param length Length, in bytes, of received Ethernet frame
void emacRx_callback(Xuint32 length){
	warpmac_disableEthernet();
	txBuffer.header.currReSend = 0;
	txBuffer.header.length = length;
	txBuffer.header.pktType = DATAPACKET;
	
	//Set the modulation scheme for the packet's full-rate symbols
	txBuffer.header.fullRate = pktFullRate;
	
	//Copy in the packet's destination MAC address
	//Hard-coded as this node's partner node
//	memcpy(txBuffer.header.destAddr,routeTable[(myID+1)%2].addr,6);
	//unsigned char addr[6];
	//memcpy(addr,(void *)warpphy_getBuffAddr(1)+NUM_HEADER_BYTES,6);
//	xil_printf("Ethernet Addr:%d.%d.%d.%d.%d.%d\r\n",addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
	
	unsigned short type = *(unsigned short *)(warpphy_getBuffAddr(1)+NUM_HEADER_BYTES+12);
	
//	xil_printf("Type %d\r\n",type);
	switch(type){
		case 2054:
			service_ARP();
			warpmac_enableEthernet();
			break;
		case 2048:
			service_IP();
			break;
		default:
			warpmac_enableEthernet();//general_service();
	}
/*	if(type==2054){
		service_ARP();
		warpmac_enableEthernet();
		return;
	
			
	}
	else {
	if(type==2048){
	
		service_IP();
	//	warpmac_enableEthernet();
		return;
		
			
	}
		else{
	   	general_service();
	//		warpmac_enableEthernet();
			return;
		}
	}*/
}
/*	memcpy(txBuffer.header.destAddr,(void *)warpphy_getBuffAddr(1)+NUM_HEADER_BYTES,6);
	
	
	memcpy(txBuffer.header.srcAddr,(unsigned char *)(warpphy_getBuffAddr(1)+6+NUM_HEADER_BYTES),6);
//	xil_printf("My Ethernet Addr:%d.%d.%d.%d.%d.%d\r\n",myAddr[0],myAddr[1],myAddr[2],myAddr[3],myAddr[4],myAddr[5]);
	if(!warpmac_addressedToThem(&txBuffer,broadcast_address)){
	//If the medium is idle
	if(warpmac_carrierSense()){
		//Copy the header over to packet buffer 1
		warpmac_prepPhyForXmit(&txBuffer,1);
		//Send packet buffer 1
		warpmac_startPhyXmit(1);
		//Wait for it to finish and enable the receiver
		warpmac_finishPhyXmit();
		//Start a timeout timer
		warpmac_setTimer(TIMEOUT);
	}
	else{
		//Start a backoff timer
		warpmac_setTimer(BACKOFF);
	}
	}
	else{
		if(warpmac_carrierSense()){
			warpmac_prepPhyForXmit(&txBuffer,1);
			warpmac_startPhyXmit(1);
			warpmac_finishPhyXmit();
		//	usleep(6);
			warpmac_enableEthernet();
			}
		else{
			warpmac_setTimer(BACKOFF);
		}
	}
	memcpy((unsigned char *)myAddr,(unsigned char *)(warpphy_getBuffAddr(1)+6+NUM_HEADER_BYTES),6);
	warpmac_setMacAddr(myAddr) ;
}
*/

void general_service(){
//	xil_printf("G\r\n");
	memcpy(txBuffer.header.destAddr,(void *)warpphy_getBuffAddr(1)+NUM_HEADER_BYTES,6);
	memcpy(txBuffer.header.srcAddr,(unsigned char *)(warpphy_getBuffAddr(1)+6+NUM_HEADER_BYTES),6);
	if(!warpmac_addressedToThem(&txBuffer,broadcast_address)){
		warpmac_enableEthernet();
	//	xil_printf("Unicast MAC PCKT?\r\n");
//		xil_printf("T %d.%d.%d.%d.%d.%d\r\n",txBuffer.header.destAddr[0],txBuffer.header.destAddr[1],txBuffer.header.destAddr[2],txBuffer.header.destAddr[3],txBuffer.header.destAddr[4],txBuffer.header.destAddr[5]);
//		xil_printf("F %d.%d.%d.%d.%d.%d\r\n",txBuffer.header.srcAddr[0],txBuffer.header.srcAddr[1],txBuffer.header.srcAddr[2],txBuffer.header.srcAddr[3],txBuffer.header.srcAddr[4],txBuffer.header.srcAddr[5]);
//		xil_printf("T %x\r\n",*(unsigned short *)(warpphy_getBuffAddr(1)+NUM_HEADER_BYTES+12));
//		xil_printf("L %d\r\n",txBuffer.header.length);
		
		return;
	}
	else{
		warpmac_enableEthernet();
	/*	if(warpmac_carrierSense()){
			warpmac_prepPhyForXmit(&txBuffer,1);
			warpmac_startPhyXmit(1);
			warpmac_finishPhyXmit();
		//	warpphy_clearTxInterrupts();
		//	usleep(6);
			warpmac_enableEthernet();
			return;
			}
		else{
			warpmac_setTimer(BACKOFF);
			return;
		}*/
	}
	return;
}


void service_IP(){
//	xil_printf("IP\r\n");
	memcpy(txBuffer.header.destAddr,(void *)warpphy_getBuffAddr(1)+NUM_HEADER_BYTES,6);
	memcpy(txBuffer.header.srcAddr,(unsigned char *)(warpphy_getBuffAddr(1)+6+NUM_HEADER_BYTES),6);
	if(!warpmac_addressedToThem(&txBuffer,broadcast_address)){
		unsigned char destIP[4];
		memcpy(destIP,(void *)(warpphy_getBuffAddr(1)+NUM_HEADER_BYTES+30),4);
		int route=find_route(destIP);
//		xil_printf("Dest IP: %d.%d.%d.%d\r\n",destIP[0],destIP[1],destIP[2],destIP[3]);
		if(route==-1){
			warpmac_enableEthernet();
//			xil_printf("Unknown\r\n");
		}
		else{
			txBuffer.header.reserved1=destIP[0];
			txBuffer.header.reserved2=destIP[1];
			txBuffer.header.reserved3=destIP[2];
			txBuffer.header.reserved4=destIP[3];
			txBuffer.header.currReSend=0;
			memcpy(txBuffer.header.destAddr,routeTable[route].Next_HOP,6);
			if(warpmac_carrierSense()){
				warpmac_prepPhyForXmit(&txBuffer,1);
				warpmac_startPhyXmit(1);
				warpmac_finishPhyXmit();
				warpmac_setTimer(TIMEOUT);
			}
			else{
				warpmac_setTimer(BACKOFF);
			}
		}
	}
	else{
		if(warpmac_carrierSense()){
			warpmac_prepPhyForXmit(&txBuffer,1);
			warpmac_startPhyXmit(1);
			warpmac_finishPhyXmit();
		//	usleep(6);
			warpmac_enableEthernet();
			}
		else{
			warpmac_setTimer(BACKOFF);
		}
		
	}
	return;
}
			
			
			
			
void service_ARP(){
//	xil_printf("ARP\r\n");
	unsigned char targ_IP[4];
	memcpy(targ_IP,(void *)(warpphy_getBuffAddr(1)+NUM_HEADER_BYTES+38),4);
	if(addressed_ToMe_IP(targ_IP)){
		memcpy((unsigned char *)myAddr,(unsigned char *)(warpphy_getBuffAddr(1)+6+NUM_HEADER_BYTES),6);
		warpmac_setMacAddr(myAddr) ;
		not_yet_set=0;
		xil_printf("Set\r\n");
	}
	else{
		int pos;
//		xil_printf("not for me\r\n");
		pos=find_route(targ_IP);
		unsigned short op=*(unsigned short *) (warpphy_getBuffAddr(1)+20+NUM_HEADER_BYTES);
	//	xil_printf("OP= %d/r/n",op);
//		xil_printf("Position: %d\r\n",pos);
		if(pos!=-1&&op==1){
			memcpy((void *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES,(void *)myAddr, 6);
			memcpy((void *)warpphy_getBuffAddr(0)+6+NUM_HEADER_BYTES,(void *) routeTable[pos].MACaddr,6);
			unsigned short type=2054;
			memcpy((void *)warpphy_getBuffAddr(0)+12+NUM_HEADER_BYTES,&type,2);
			memcpy((void *)warpphy_getBuffAddr(0)+14+NUM_HEADER_BYTES,(void *)(warpphy_getBuffAddr(1)+NUM_HEADER_BYTES+14),6);
			unsigned short op2=2;
			memcpy((void *)warpphy_getBuffAddr(0)+20+NUM_HEADER_BYTES,&op2,2);
			memcpy((void *)warpphy_getBuffAddr(0)+22+NUM_HEADER_BYTES,(void *) (routeTable[pos].MACaddr),6);
			memcpy((void *)warpphy_getBuffAddr(0)+28+NUM_HEADER_BYTES,(void *)(routeTable[pos].IPaddr),4);
			memcpy((void *)warpphy_getBuffAddr(0)+32+NUM_HEADER_BYTES,(void *)myAddr, 6);
			memcpy((void *)warpphy_getBuffAddr(0)+38+NUM_HEADER_BYTES,(void *)myIP,4);
			Macframe pack;
			pack.header.length=42;
			warpmac_prepEmacForXmit(&pack);
			warpmac_startEmacXmit(&pack);
	//		xil_printf("MAC %d.%d.%d.%d.%d.%d\r\n",routeTable[pos].MACaddr[0],routeTable[pos].MACaddr[1],routeTable[pos].MACaddr[2],routeTable[pos].MACaddr[3],routeTable[pos].MACaddr[4],routeTable[pos].MACaddr[5]);
		}
	}
	return;
}

//Returns true if addr is my IP, false otherwise
int addressed_ToMe_IP(unsigned char *addr){
	int i,sum=0;
	for(i=0;i<4;i++){
		sum+=(myIP[i]==addr[i]);
	}
	if(sum==4)
		return 1;
	else 
		return 0;
}

int addressedToBR(Macframe *packet){
	int i,sum1=0,sum2=0;
	for(i=0;i<6;i++){
		if(i>=4){
			sum1+=(packet->header.destAddr[i]==broadcast_address[i]);		
		}
		else{
			sum1+=(packet->header.destAddr[i]==broadcast_address[i]);
			sum2+=(packet->header.destAddr[i]==broadcast_address2[i]);
		}
	}
	if((sum1==6)||(sum2==4)){
		return 1;
	}
	else{
		return 0;
	}
}
//Retrns the RouteTable position of (IP) addr
int find_route(unsigned char *addr){
	int i,sum=0;
	for(i=0;i<3;i++){
		sum+=(myIP[i]==addr[i]);
	}
	XTime time;
	XTime_GetTime(&time);
	if((sum==3)&&(addr[3]<=5)){
		if((XTime)routeTable[addr[3]-1].timestamp+(XTime)ROUTE_EXPIRATION*160000000>=time)
			return addr[3]-1;
	}
	return -1;
}

int find_route_MAC(unsigned char *macAddr){
	int i,j;
	for(i=0;i<5;i++){
		int sum=0;
		for(j=0;j<6;j++){
			sum+=((unsigned char)routeTable[i].MACaddr[j]==(unsigned char) macAddr[j]);
		}
		if(sum==6){
			return i;
		}
	}
	return -1;
}
///@brief Callback for the reception of bad wireless headers
///
///@param packet Pointer to received Macframe
void phyRx_badHeader_callback() {
//	warpmac_incrementLEDLow();

}


void phyRx_goodHeader_callback(Macframe * packet){
	unsigned char state=INCOMPLETE;
	unsigned char final_dest[4]={packet->header.reserved1,packet->header.reserved2,packet->header.reserved3,packet->header.reserved4};
	Macframe ackPacket;
	if(warpmac_addressedToMe(packet)){
		switch(packet->header.pktType){
			case DATAPACKET:
				ackPacket.header.length = 0;
				ackPacket.header.pktType = ACKPACKET;
				ackPacket.header.fullRate = pktFullRate;
				memcpy(ackPacket.header.srcAddr,(unsigned char *)myAddr,6);
				memcpy(ackPacket.header.destAddr,packet->header.srcAddr,6);
				warpmac_prepPhyForXmit(&ackPacket,2);
				if(addressed_ToMe_IP(final_dest)){
					while(state==INCOMPLETE){
						//Blocks until the PHY reports the received packet as either good or bad
						state = warpphy_pollRxStatus();
					}
					if(state==GOODPACKET){
							//Send packet buffer 2 containing the ACK
					//	usleep(6);
						
						
						warpmac_startPhyXmit(2);
						warpmac_prepEmacForXmit(packet);
						warpmac_startEmacXmit(packet);
						warpmac_finishPhyXmit();
						
						
		//				xil_printf("m");
					}
		//			else{
		//				xil_printf("B");
		//			}
//					xil_printf("m\r\n");
				}
				else{
					
					warpmac_prepPhyForXmit(&ackPacket,2);
			//		if(route!=-1){
						
						//txBuffer.header.length=packet->header.length;
					//	memcpy((void *)warpphy_getBuffAddr(1),(void *)warpphy_getBuffAddr(0),packet->header.length);
			//		}
					int route=find_route_MAC(ackPacket.header.destAddr);
					while(state==INCOMPLETE){
					//Blocks until the PHY reports the received packet as either good or bad
						state = warpphy_pollRxStatus();
					}
				//	if(equal_MACs(routeTable[route].Next_HOP,packet->header.srcAddr)!=6){
						if(state==GOODPACKET){
							memcpy(packet->header.destAddr,routeTable[packet->header.reserved4-1].Next_HOP,6);
							
							warpmac_startPhyXmit(2);
							memcpy(packet->header.srcAddr,myAddr,6);
							memcpy((void *)warpphy_getBuffAddr(1)+NUM_HEADER_BYTES,(void *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES,packet->header.length);
							memcpy(&txBuffer,packet,NUM_HEADER_BYTES);
							warpmac_finishPhyXmit();
							if(route!=-1){
							//	
								
								txBuffer.header.currReSend=0;
								if(warpmac_carrierSense()){
							   	warpmac_prepPhyForXmit(&txBuffer,1); //Or use 3 or 2???? Maybe 1 directly?Shall I close the receiver?!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
									warpmac_startPhyXmit(1);
									warpmac_finishPhyXmit();
									warpmac_setTimer(TIMEOUT);
						 //		warpmac_setTimer(TIMEOUT);
								//
									
											
								}
								else{
									warpmac_setTimer(BACKOFF);
								//	mimo_ofdmRx_enable();
								}
			//				xil_printf("F");
							}
						}
			//		}
				}
				break;
			case ACKPACKET:
				if(warpmac_inTimeout()){
					warpmac_clearTimer(TIMEOUT);
					warpmac_enableEthernet();
				}
				break;
		}
	}
	if(addressedToBR(packet)){
		switch(packet->header.pktType){
			case DATAPACKET:
				while(state==INCOMPLETE){
					state = warpphy_pollRxStatus();
				}
				if(state==GOODPACKET){
					warpmac_prepEmacForXmit(packet);
					warpmac_startEmacXmit(packet);
				}
				break;
			case ADVPACKET:
//				xil_printf("ADV\r\n");
				if(packet->header.length==0){
					//No other entries, move on
					checkAdvertisment(packet,0);
				}
				else{
					while(state==INCOMPLETE){
						state = warpphy_pollRxStatus();
					}
					if(state==GOODPACKET){
						checkAdvertisment(packet,packet->header.length/11);
					}
				//	else{
				//		checkAdvertisment(packet,0);
				//	}
				}
				break;
		}
	}
	return;
}
/*///@brief Callback for the reception of good wireless headers
///
///This function then polls the PHY to determine if the entire packet passes checksum
///thereby triggering the transmission of the ACK and the transmission of the received
///data over Ethernet.
///@param packet Pointer to received Macframe
void phyRx_goodHeader_callback(Macframe* packet){
	Macframe ackPacket;
	unsigned char state=INCOMPLETE;
	unsigned char final_dest[4]={packet->header.reserved1,packet->header.reserved2,packet->header.reserved3,packet->header.reserved4};
//	xil_printf("Good Hd: src_addr=%d.%d.%d.%d.%d.%d\r\n",packet->header.srcAddr[0],packet->header.srcAddr[1],packet->header.srcAddr[2],packet->header.srcAddr[3],packet->header.srcAddr[4],packet->header.srcAddr[5]);
//	xil_printf("Good Hd: dest_addr=%d.%d.%d.%d.%d.%d\r\n",packet->header.destAddr[0],packet->header.destAddr[1],packet->header.destAddr[2],packet->header.destAddr[3],packet->header.destAddr[4],packet->header.destAddr[5]);
	//If the packet is addressed to this node
	if(warpmac_addressedToMe(packet)){
		if(addressed_ToMe_IP(final_dest)){
			switch(packet->header.pktType){
				//If received packet is data
				case DATAPACKET:
					//Acknowledge the reception of the data
			
					ackPacket.header.length = 0;
					ackPacket.header.pktType = ACKPACKET;
					ackPacket.header.fullRate = pktFullRate;
					memcpy(ackPacket.header.srcAddr,(unsigned char *)myAddr,6);
					memcpy(ackPacket.header.destAddr,packet->header.srcAddr,6);
					int route=find_route_MAC(ackPacket.header.destAddr);
					if(route!=-1){
						ackPacket.header.reserved1=routeTable[route].IPaddr[0];
						ackPacket.header.reserved2=routeTable[route].IPaddr[1];
						ackPacket.header.reserved3=routeTable[route].IPaddr[2];
						ackPacket.header.reserved4=routeTable[route].IPaddr[3];
				}
	//			xil_printf("Make ack for Mac UN %x.%x.%x.%x.%x.%x\r\n",ackPacket.header.destAddr[0],ackPacket.header.destAddr[1],ackPacket.header.destAddr[2],ackPacket.header.destAddr[3],ackPacket.header.destAddr[4],ackPacket.header.destAddr[5]);
	//			xil_printf("Make ack for IP UN %d.%d.%d.%d, route %d\r\n",ackPacket.header.reserved1,ackPacket.header.reserved2,ackPacket.header.reserved3,ackPacket.header.reserved4,route);
			
					//Copy the header over to packet buffer 2
					warpmac_prepPhyForXmit(&ackPacket,2);
				
					//At this point, we have pre-loaded the PHY transmitter with the ACK in hoping that
					//the packet passes checksum. Now we wait for the state of the received to packet
					//to move from INCOMPLETE to either GOODPACKET or BADPACKET
					while(state==INCOMPLETE){
						//Blocks until the PHY reports the received packet as either good or bad
						state = warpphy_pollRxStatus();
					}
				
					if(state==GOODPACKET){
						//Send packet buffer 2 containing the ACK
						warpmac_startPhyXmit(2);
			//			warpmac_incrementLEDHigh();
						warpmac_rightHex(packet->header.currReSend);
						//Starts the DMA transfer of the payload into the EMAC
						warpmac_prepEmacForXmit(packet);
						//Blocks until the PHY is finished sending and enables the receiver
						warpmac_finishPhyXmit();
						//Waits until the DMA transfer is complete, then starts the EMAC
						warpmac_startEmacXmit(packet);
					}
				
				/*	if(state==BADPACKET){
				//		warpmac_incrementLEDLow();
					}*/
	/*				xil_printf("Fm\r\n");
					return;
				break;
				
				case ACKPACKET:
					//Clear the TIMEOUT and enable Ethernet
				//	xil_printf("aCK\r\n");
					if(warpmac_inTimeout()){
				//		warpmac_incrementLEDHigh();
						warpmac_clearTimer(TIMEOUT);
						warpmac_enableEthernet();
					}
				return;
				break;
			}
		}
		else{
			if(packet->header.pktType==DATAPACKET){
				ackPacket.header.length = 0;
				ackPacket.header.pktType = ACKPACKET;
				ackPacket.header.fullRate = pktFullRate;
				memcpy(ackPacket.header.srcAddr,(unsigned char *)myAddr,6);
				memcpy(ackPacket.header.destAddr,packet->header.srcAddr,6);
				int route=find_route_MAC(ackPacket.header.destAddr);
				if(route!=-1){
					ackPacket.header.reserved1=routeTable[route].IPaddr[0];
					ackPacket.header.reserved2=routeTable[route].IPaddr[1];
					ackPacket.header.reserved3=routeTable[route].IPaddr[2];
					ackPacket.header.reserved4=routeTable[route].IPaddr[3];
				}
				//	xil_printf("Make ack for Mac MH %x.%x.%x.%x.%x.%x\r\n",ackPacket.header.destAddr[0],ackPacket.header.destAddr[1],ackPacket.header.destAddr[2],ackPacket.header.destAddr[3],ackPacket.header.destAddr[4],ackPacket.header.destAddr[5]);
				//	xil_printf("Make ack for IP MH %d.%d.%d.%d, route %d\r\n",ackPacket.header.reserved1,ackPacket.header.reserved2,ackPacket.header.reserved3,ackPacket.header.reserved4,route);
				//Copy the header over to packet buffer 2
				warpmac_prepPhyForXmit(&ackPacket,2);
				//At this point, we have pre-loaded the PHY transmitter with the ACK in hoping that
				//the packet passes checksum. Now we wait for the state of the received to packet
				//to move from INCOMPLETE to either GOODPACKET or BADPACKET
				while(state==INCOMPLETE){
					//Blocks until the PHY reports the received packet as either good or bad
					state = warpphy_pollRxStatus();
				}
				if(state==GOODPACKET){
					//Send packet buffer 2 containing the ACK
					warpmac_startPhyXmit(2);
		//			warpmac_incrementLEDHigh();
			//		warpmac_rightHex(packet->header.currReSend);
					//Starts the DMA transfer of the payload into the EMAC
					//warpmac_prepEmacForXmit(packet);
					//Blocks until the PHY is finished sending and enables the receiver
					warpmac_finishPhyXmit();
					//Waits until the DMA transfer is complete, then starts the EMAC
					//warpmac_startEmacXmit(packet);
			//		xil_printf("Not fd - fd= %d.%d.%d.%d\r\n",packet->header.reserved1,packet->header.reserved2,packet->header.reserved3,packet->header.reserved4);
			//		xil_printf("Dest MAc %x.%x.%x.%x.%x.%x\r\n",packet->header.destAddr[0],packet->header.destAddr[1],packet->header.destAddr[2],packet->header.destAddr[3],packet->header.destAddr[4],packet->header.destAddr[5]);
			//		xil_printf("PKtype = %d\r\n",packet->header.pktType);
					int route=find_route(final_dest);
			//		xil_printf("Route %d\r\n",route);
					if(route!=-1){
		//				xil_printf("sd\r\n");
						memcpy(packet->header.destAddr,routeTable[packet->header.reserved4-1].Next_HOP,6);
						int rt=find_route_MAC(packet->header.srcAddr);
						memcpy(packet->header.srcAddr,myAddr,6);
						memcpy(&txBuffer,packet,NUM_HEADER_BYTES);
						memcpy((void *)warpphy_getBuffAddr(1),(void *)warpphy_getBuffAddr(0),packet->header.length);
						if(warpmac_carrierSense()){
							warpmac_prepPhyForXmit(&txBuffer,1);
							warpmac_startPhyXmit(1);
							warpmac_finishPhyXmit();
							warpmac_setTimer(TIMEOUT);
						}
						else{
							warpmac_setTimer(BACKOFF);
						}
				//		unsigned char destIP[4];
				//		memcpy(destIP,(void *)(warpphy_getBuffAddr(1)+NUM_HEADER_BYTES+30),4);
						
						xil_printf("Fw to %d from %d\r\n",final_dest[3],rt+1);
						return;
				//		xil_printf("F");
					}
				}
			/*	if(state==BADPACKET){
		//			warpmac_incrementLEDLow();
				}*/
/*			}
			else{
				xil_printf("smthg is wrong\r\n");
			}
		}
	}
	else {
		if(addressedToBR(packet)){
			switch(packet->header.pktType){
				case DATAPACKET:
					//	xil_printf("Broadcast_PKT\r\n");
					while(state==INCOMPLETE){
						//Blocks until the PHY reports the received packet as either good or bad
						state = warpphy_pollRxStatus();
					}
					if(state==GOODPACKET){
						//	xil_printf("Good_Pkt");
						//Send packet buffer 2 containing the ACK
						//warpmac_startPhyXmit(2);
						//warpmac_incrementLEDHigh();
						//warpmac_rightHex(packet->header.currReSend);
						//Starts the DMA transfer of the payload into the EMAC
						warpmac_prepEmacForXmit(packet);
						//Blocks until the PHY is finished sending and enables the receiver
						//warpmac_finishPhyXmit();
						//Waits until the DMA transfer is complete, then starts the EMAC
						warpmac_startEmacXmit(packet);
					}
				//	if(state==BADPACKET){
						//	xil_printf("Bad_Pkt");
				//		warpmac_incrementLEDLow();
				//	}
					return;
				break;
				case ADVPACKET:
					if(packet->header.length==0){
						//No other entries, move on
						checkAdvertisment(packet,0);
					}
					else{
						while(state==INCOMPLETE){
							//Blocks until the PHY reports the received packet as either good or bad
							state = warpphy_pollRxStatus();
						}
						if(state==GOODPACKET){
							checkAdvertisment(packet,packet->header.length/11);
						}
						else{
							checkAdvertisment(packet,0);
						}
					}
					return;
				break;
			}
		}
	}
	return;
}*/

int equal_MACs(unsigned char *addr1,unsigned char *addr2){
	int i,sum=0;
	for(i=0;i<6;i++){
		sum+=(addr1[i]==addr2[i]);
	}
	return sum;
}


void checkAdvertisment(Macframe *packet, unsigned int num_entries){
	warpmac_enableEthernet();
	XTime time;
	XTime_GetTime(&time);
//	xil_printf("CHK from %d, %d \r\n",packet->header.reserved4,num_entries);
	
	int j;
/*	for(j=0;j<3;j++){
		if(j!=myID){
		xil_printf("IP: %d.%d.%d.%d\t",routeTable[j].IPaddr[0],routeTable[j].IPaddr[1],routeTable[j].IPaddr[2],routeTable[j].IPaddr[3]);
		xil_printf("MAC: %x.%x.%x.%x.%x.%x\t",routeTable[j].MACaddr[0],routeTable[j].MACaddr[1],routeTable[j].MACaddr[2],routeTable[j].MACaddr[3],routeTable[j].MACaddr[4],routeTable[j].MACaddr[5]);
		xil_printf("N-H: %x.%x.%x.%x.%x.%x\t",routeTable[j].Next_HOP[0],routeTable[j].Next_HOP[1],routeTable[j].Next_HOP[2],routeTable[j].Next_HOP[3],routeTable[j].Next_HOP[4],routeTable[j].Next_HOP[5]);
		xil_printf("Num-H: %d\r\n",(routeTable[j].num_hops));
		}
	}*/
	routeTable[packet->header.reserved4-1].timestamp=time;
	memcpy(routeTable[packet->header.reserved4-1].MACaddr,packet->header.srcAddr,6);
	memcpy(routeTable[packet->header.reserved4-1].Next_HOP,packet->header.srcAddr,6);
	routeTable[packet->header.reserved4-1].num_hops=(unsigned char)1;
	if(num_entries>0){
		int i;
		for(i=0;i<num_entries;i++){
			if(!addressed_ToMe_IP((void *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i)){
				unsigned char ps;
				memcpy(&ps,(unsigned char *)(warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+3),1);
				ps--;
				unsigned char hops;
				memcpy(&hops,(unsigned char *)(warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+10),1);
				
		//		xil_printf("For %d, hops adv %d\r\n",(unsigned char)ps,(unsigned char)hops);
				if((routeTable[ps].timestamp+(XTime)ROUTE_EXPIRATION*160000000<time)||(routeTable[ps].num_hops-(unsigned char)1>=hops)){
					memcpy(routeTable[ps].MACaddr,(void *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+4,6);
					memcpy(routeTable[ps].Next_HOP,packet->header.srcAddr,6);
					routeTable[ps].timestamp=time	;
					routeTable[ps].num_hops=hops+(unsigned char)1;
			/*		if((routeTable[ps].timestamp+(XTime)ROUTE_EXPIRATION*160000000<time)){
						xil_printf("Up bcse of EXP\r\n");
					}
					else{
						xil_printf("UP bcse of HOPs\r\n");
					}*/
					//		xil_printf("MH! To %x.%x.%x.%x.%x.%x via %x.%x.%x.%x.%x.%x\r\n",routeTable[*(unsigned char *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+3].MACaddr[0],routeTable[*(unsigned char *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+3].MACaddr[1],routeTable[*(unsigned char *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+3].MACaddr[2],routeTable[*(unsigned char *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+3].MACaddr[3],routeTable[*(unsigned char *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+3].MACaddr[4],routeTable[*(unsigned char *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+3].MACaddr[5],routeTable[*(unsigned char *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+3].Next_HOP[0],routeTable[*(unsigned char *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+3].Next_HOP[1],routeTable[*(unsigned char *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+3].Next_HOP[2],routeTable[*(unsigned char *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+3].Next_HOP[3],routeTable[*(unsigned char *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+3].Next_HOP[4],routeTable[*(unsigned char *)warpphy_getBuffAddr(0)+NUM_HEADER_BYTES+11*i+3].Next_HOP[5]);
				}
				
			}
		}
	}
//	xil_printf("After:\r\n");
/*	for(j=0;j<3;j++){
		if(j!=myID){
		xil_printf("IP: %d.%d.%d.%d\t",routeTable[j].IPaddr[0],routeTable[j].IPaddr[1],routeTable[j].IPaddr[2],routeTable[j].IPaddr[3]);
		xil_printf("MAC: %x.%x.%x.%x.%x.%x\t",routeTable[j].MACaddr[0],routeTable[j].MACaddr[1],routeTable[j].MACaddr[2],routeTable[j].MACaddr[3],routeTable[j].MACaddr[4],routeTable[j].MACaddr[5]);
		xil_printf("N-H: %x.%x.%x.%x.%x.%x\t",routeTable[j].Next_HOP[0],routeTable[j].Next_HOP[1],routeTable[j].Next_HOP[2],routeTable[j].Next_HOP[3],routeTable[j].Next_HOP[4],routeTable[j].Next_HOP[5]);
		xil_printf("Num-H: %d\r\n",(unsigned char)routeTable[j].num_hops);
		}
	}*/
	return;
}

void sendAdvertisment(XTime tcur){
//	xil_printf("A - A %lld\r\n",tcur);
//	warpmac_clearTimer(TIMEOUT);
//	warpmac_clearTimer(BACKOFF);
	//MacFrame advertisment;
	warpmac_disableEthernet();
	
	
	/*if(warpmac_inTimeout()){
		//	warpmac_incrementLEDHigh();
		warpmac_clearTimer(TIMEOUT);
	//	warpmac_enableEthernet();
	}*/
	
	int num_routes=0;
	int i;
	for(i=0;i<5;i++){
		XTime tm=routeTable[i].timestamp;
		if ((tm+(XTime)ROUTE_EXPIRATION*160000000>=tcur)){
			if(i!=myID){
				memcpy((unsigned char *)warpphy_getBuffAddr(2)+NUM_HEADER_BYTES+num_routes*11,routeTable[i].IPaddr,4);
				memcpy((unsigned char *)warpphy_getBuffAddr(2)+NUM_HEADER_BYTES+num_routes*11+4,routeTable[i].MACaddr,6);
				memcpy((unsigned char *)warpphy_getBuffAddr(2)+NUM_HEADER_BYTES+num_routes*11+10,&routeTable[i].num_hops,1);
	//			xil_printf("ADV for %d.%d.%d.%d\r\n",routeTable[i].IPaddr[0],routeTable[i].IPaddr[1],routeTable[i].IPaddr[2],routeTable[i].IPaddr[3]);
				num_routes++;
	//			xil_printf("%lld vs %lld\r\n",(XTime)tcur,*(XTime *)routeTable[i].timestamp);
			}
		}
	}
	txBuffer.header.pktType=ADVPACKET;
	txBuffer.header.currReSend = 0;
	memcpy(txBuffer.header.srcAddr,(unsigned char *)myAddr,6);
	memcpy(txBuffer.header.destAddr,broadcast_address,6);
//	xil_printf("NR %d\r\n",num_routes);
	txBuffer.header.length=11*num_routes;
	txBuffer.header.reserved1=myIP[0];
	txBuffer.header.reserved2=myIP[1];
	txBuffer.header.reserved3=myIP[2];
	txBuffer.header.reserved4=myIP[3];
//	
	
	if(warpmac_carrierSense()){
		warpmac_prepPhyForXmit(&txBuffer,2);
		warpmac_startPhyXmit(2);
		warpmac_finishPhyXmit();
		warpmac_enableEthernet();
		
	}
	else{
		memcpy((unsigned char *)(warpphy_getBuffAddr(1)+NUM_HEADER_BYTES),(unsigned char *)(warpphy_getBuffAddr(2)+NUM_HEADER_BYTES),txBuffer.header.length);
		warpmac_setTimer(BACKOFF);
	}
//	warpphy_clearTxInterrupts();
	//xil_printf("I send out %d number of entries\r\n",num_routes);
	
	return;
}


///@brief Main function
///
///This function configures MAC parameters, enables the underlying frameworks, and then loops forever.
int main(){		
	
	pktFullRate = HDR_FULLRAATE_QPSK;
	
	//Initialize the framework
	warpmac_init();
//	warpmac_setMaxResend(8);
	warpmac_setMaxResend(5);
	warpmac_setMaxCW(5);
//	warpmac_setTimeout(160);
	warpmac_setTimeout(280);
	warpmac_setSlotTime(9);
	
	
	
	//Read Dip Switch value from FPGA board.
	//This value will be used as an index into the routing table for other nodes
	myID = warpmac_getMyId();
	
	//Create an arbitrary address for this node
//	unsigned char tmpAddr[6] = {0x16,0x24,0x63,0x53,0xe2,0xc2};
	unsigned char tmp[4]={10,0,0,1+myID};
	memcpy((void *)myIP,(void *)tmp,4);
//	xil_printf("My ID %d\r\n",myID);
	//memcpy((unsigned char *)myAddr,(unsigned char *)tmpAddr,6);
	
	//Fill an arbitrary routing table so that nodes know each others' addresses
	unsigned char i;
	unsigned char zero_MAC[6]={0,0,0,0,0,0};
//	XTime tcur;
//	XTime_GetTime(&tcur);
	for(i=0;i<5;i++){
		memcpy((void *)routeTable[i].IPaddr,(void *)myIP,3);
		routeTable[i].IPaddr[3]=1+i;
		memcpy(routeTable[i].MACaddr,(void *)zero_MAC,6);
		memcpy(routeTable[i].Next_HOP,(void *)zero_MAC,6);
		routeTable[i].num_hops=(unsigned char)50;
		routeTable[i].timestamp=(XTime)0;
	}
	//For test purposes - needs 2 b removed
//	memcpy(routeTable[1].MACaddr,(void *)tmpAddr,6);
//	memcpy(routeTable[1].Next_HOP,(void *)tmpAddr,6);
	
	/*//Sets the source MAC address
	warpmac_setMacAddr((unsigned char *)(&myAddr));*/
	
	//Rx buffer is where the EMAC will DMA Wireless payloads from
	warpmac_setRxBuffer(&rxBuffer,0);
	//Tx buffer is where the EMAC will DMA Ethernet payloads to
	warpmac_setTxBuffer(1);
	
	//Copy this node's MAC address into the Tx buffer's source address field
	//memcpy(txBuffer.header.srcAddr,(unsigned char *)myAddr,6);
	XTime_SetTime((XTime)(ROUTE_EXPIRATION+1)*160000000);
	//Register callbacks
	warpmac_setBadHeaderCallback((void *)phyRx_badHeader_callback);
	warpmac_setGoodHeaderCallback((void *)phyRx_goodHeader_callback);
	warpmac_setTimerCallback((void *)timer_callback);
	warpmac_setEmacCallback((void *)emacRx_callback);
	warpmac_setUpButtonCallback((void *)up);
	warpmac_setMiddleButtonCallback((void *)middle);
	warpmac_setLeftButtonCallback((void *)left);
	warpmac_setRightButtonCallback((void *)right);
	XTime tcur;
	XTime_GetTime(&tcur);
	XTime tend=(XTime)0;
//	xil_printf("Tcur %lld\r\n",tcur);
//	xil_printf("Tend %lld\r\n",tend);
	unsigned int seconds = 40;
	//Send Advertisment at once
   unsigned char startCount = 0;

	
	
	//Set the default center frequency
	warpphy_setChannel(GHZ_5, chan);
	
	//Enable carrier sensing
	warpmac_enableCSMA();
	
	//Enable Ethernet
	warpmac_enableEthernet();
	
	//Set the modulation scheme use for base rate (header) symbols
	warpmac_setBaseRate(QPSK);
	while(not_yet_set){
		warpmac_pollEthernet();
	}
	
	while(1){
		//Poll the Ethernet
		warpmac_pollEthernet();
		
		XTime_GetTime(&tcur);
		//xil_printf("Tcur2 %lld\r\n",*tcur);
		//xil_printf("Tend %lld\r\n",*tend);
		if(startCount){
			tend=tcur+ ((XTime) seconds) * 160000000;
			startCount=0;
		}
		if((XTime)tcur>(XTime)tend){
		//	warpmac_disableEthernet();
	//		xil_printf("B-S=:%lld\r\n",tcur);
			sendAdvertisment(tcur);
			startCount=1;
		//	warpmac_enableEthernet();
		}
	
		
		
	}
	
	return;
}
