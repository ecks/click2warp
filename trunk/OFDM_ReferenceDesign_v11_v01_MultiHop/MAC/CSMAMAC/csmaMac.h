/*! \file csmaMac.h
\brief Carrier-sensing random access MAC.

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

void up();
void middle();
void right();
void left();
void phyRx_goodHeader_callback(Macframe* packet);
void phyRx_badHeader_callback();
void emacRx_callback(Xuint32 length);
void timer_callback(unsigned char timerType);
void service_ARP();
void service_IP();
void general_service();
void sendAdvertisment(XTime tcur);
void checkAdvertisment(Macframe *packet, unsigned int num_entries);
int addressed_ToMe_IP(unsigned char *addr);
int addressedToBR(Macframe *packet);
int find_route(unsigned char *addr);
int equal_MACs(unsigned char *addr1,unsigned char *addr2);
int find_route_MAC(unsigned char *macAddr);
void printRouteTable();
int main();
