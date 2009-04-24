#include <iostream>

#include <click/config.h>
#include "warpinput.hh"
CLICK_DECLS

WarpInput::WarpInput()
{
}

WarpInput::~WarpInput()
{
}

Packet *
WarpInput::simple_action(Packet *p)
{
  return p;
}


Packet *
WarpInput::make_packet()
{
    WritablePacket *p = Packet::make(WarpInput::staticString->data(), WarpInput::staticString->length());

    int i;  
    char *d = (char *) p->data();
    for (i = 0; i < 5; i += sizeof(int))
        *(int*)(d + i) = click_random();
    for( ; i < 5; i++)
        *(d + i) = click_random();

    p->timestamp_anno().set_now();
    return p;
}

bool
WarpInput::run_task(Task *)
{
  Packet *p = make_packet();
  output(0).push(p);
  return true;
}

void
WarpInput::add_handlers()
{
  add_write_handler("test", test_fnct, (void *)0);

}

int
WarpInput::test_fnct(const String &s, Element *e, void *vparam, ErrorHandler *errh)
{
  staticString = new String(s.data());
  Packet *packet = Packet::make(s.data(), s.length());
  click_chatter("%s", s.c_str());
  return 0;
}

CLICK_ENDDECLS
EXPORT_ELEMENT(WarpInput)
ELEMENT_MT_SAFE(WarpInput)
