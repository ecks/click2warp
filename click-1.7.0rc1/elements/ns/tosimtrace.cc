// Copyright (c) 2004 by the University of Antwerp
// All rights reserved.
//
// Permission to use, copy, modify, and distribute this software and its
// documentation in source and binary forms for non-commercial purposes
// and without fee is hereby granted, provided that the above copyright
// notice appear in all copies and that both the copyright notice and
// this permission notice appear in supporting documentation. and that
// any documentation, advertising materials, and other materials related
// to such distribution and use acknowledge that the software was
// developed by the Polytechnic University of Catalonia, Computer Networking
// Group.  The name of the University may not be used to
// endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.
//
// Other copyrights might apply to parts of this software and are so
// noted when applicable.

#include <click/config.h>
#include <click/confparse.hh>
#include "tosimtrace.hh"
#include <click/packet_anno.hh>
#include <click/string.hh>

CLICK_DECLS

ToSimTrace::ToSimTrace()
                : _packetAnalyzer(0), _offset(0), _checkPaint(false)
{
        additional_info_ = "";
}


ToSimTrace::~ToSimTrace()
{
}


int
ToSimTrace::configure(Vector<String> &conf, ErrorHandler *errh)
{
        if (cp_va_kparse(conf, this, errh,
			 "EVENT", cpkP+cpkM, cpString, &event_,
			 "ADDITIONAL_INFO", 0, cpString, &additional_info_,
			 "ANALYZER", 0, cpElement, &_packetAnalyzer,
			 "OFFSET", 0, cpInteger, &_offset,
			 cpEnd) < 0)
                return -1;
        return 0;
}


void
ToSimTrace::push(int, Packet *packet)
{
        struct timeval now;
        click_gettimeofday(&now);
        simclick_simpacketinfo* pinfo = packet->get_sim_packetinfo();

        // the nsclick interface uses pinfo to store ns2 information that needs to traversed through the click graph. The 'id' field is used to store the ns2 id of the packet. If the packet is generated by click this is only set when the packet arrives at ns2 for the first time. We need to set it here for correct tracing.

        if (pinfo->id < 0) {
	    pinfo->id = router()->sim_get_next_pkt_id();
        }

        char buffer[250];

        String analysis = "";
        if (_packetAnalyzer != 0) {
                analysis = _packetAnalyzer->analyze(packet, _offset);
        }

        sprintf(buffer, "%s %f _%i_ RTR --- %i raw %i [%s %s]", event_.c_str(), Timestamp(now).doubleval(), router()->sim_get_node_id(), pinfo->id, packet->length()-_offset, additional_info_.c_str(), analysis.c_str());

        router()->sim_trace(buffer);

        output(0).push(packet);
}

CLICK_ENDDECLS
ELEMENT_REQUIRES(ns)
EXPORT_ELEMENT(ToSimTrace);