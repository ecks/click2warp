#ifndef CLICK_WARPINPUT_HH
#define CLICK_WARPINPUT_HH
#include <click/element.hh>
CLICK_DECLS

class WarpInput : public Element 
{ 
 public:

  WarpInput();
  ~WarpInput();

  const char *class_name() const	{ return "WarpInput"; }
  const char *port_count() const	{ return PORTS_1_1; }
  const char *processing() const	{ return AGNOSTIC; }

  Packet *simple_action(Packet *);

  void add_handlers();

  static int test_fnct(const String &, Element *, void *, ErrorHandler *);
};

/*
=c
PushNull

=s basictransfer
passes packets unchanged

=d
Responds to each pushed packet by pushing it unchanged out its first output.

=a
Null, PullNull
*/

class PushWarpInput : public Element { public:

  PushWarpInput();
  ~PushWarpInput();

  const char *class_name() const	{ return "PushWarpInput"; }
  const char *port_count() const	{ return PORTS_1_1; }
  const char *processing() const	{ return PUSH; }

  void push(int, Packet *);

};

/*
=c
PullNull

=s basictransfer
passes packets unchanged

=d
Responds to each pull request by pulling a packet from its input and returning
that packet unchanged.

=a
Null, PushNull */

class PullWarpInput : public Element { public:

  PullWarpInput();
  ~PullWarpInput();

  const char *class_name() const	{ return "PullWarpInput"; }
  const char *port_count() const	{ return PORTS_1_1; }
  const char *processing() const	{ return PULL; }

  Packet *pull(int);

};

CLICK_ENDDECLS
#endif
