#include <iostream>

#include <click/config.h>
#include "warpinput.hh"
CLICK_DECLS

using namespace std;

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


void
WarpInput::add_handlers()
{
  add_write_handler("test", test_fnct, (void *)0);

}

int
WarpInput::test_fnct(const String &s, Element *e, void *vparam, ErrorHandler *errh)
{
//  cout << s* << endl;
  cout << "hello world!" << endl;
  return 0;
}

PushWarpInput::PushWarpInput()
{
}

PushWarpInput::~PushWarpInput()
{
}

void
PushWarpInput::push(int, Packet *p)
{
  output(0).push(p);
}

PullWarpInput::PullWarpInput()
{
}

PullWarpInput::~PullWarpInput()
{
}

Packet *
PullWarpInput::pull(int)
{
  return input(0).pull();
}


CLICK_ENDDECLS
EXPORT_ELEMENT(WarpInput)
EXPORT_ELEMENT(PushWarpInput)
EXPORT_ELEMENT(PullWarpInput)
ELEMENT_MT_SAFE(WarpInput)
ELEMENT_MT_SAFE(PushWarpInput)
ELEMENT_MT_SAFE(PullWarpInput)
