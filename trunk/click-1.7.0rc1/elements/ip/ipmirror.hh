#ifndef CLICK_IPMIRROR_HH
#define CLICK_IPMIRROR_HH
#include <click/element.hh>
CLICK_DECLS

/*
=c

IPMirror

=s ip

swaps IP source and destination

=d

Incoming packets must have their IP header annotations set. Swaps packets'
source and destination IP addresses. Packets containing TCP or UDP
headers---that is, first fragments of packets with protocol 6 or 17---also
have their source and destination ports swapped. TCP packets also have their
seq and ack numbers swapped.

The IP and TCP or UDP checksums are not changed. They don't need to be; these
swap operations do not affect checksums.

*/

class IPMirror : public Element {

 public:

  IPMirror();
  ~IPMirror();

  const char *class_name() const		{ return "IPMirror"; }
  const char *port_count() const		{ return PORTS_1_1; }
  const char *processing() const		{ return AGNOSTIC; }

  Packet *simple_action(Packet *);

};

CLICK_ENDDECLS
#endif // CLICK_IPMIRROR_HH
