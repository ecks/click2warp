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

CLICK_ENDDECLS
#endif
