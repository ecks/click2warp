#ifndef CLICK_UNQUEUE_HH
#define CLICK_UNQUEUE_HH
#include <click/element.hh>
#include <click/task.hh>
#include <click/notifier.hh>
CLICK_DECLS

/*
=c

Unqueue([BURST, I<KEYWORDS>])

=s shaping

pull-to-push converter

=d

Pulls packets whenever they are available, then pushes them out
its single output. Pulls a maximum of BURST packets every time
it is scheduled. Default BURST is 1. If BURST
is less than 0, pull until nothing comes back.

Keyword arguments are:

=over 4

=item ACTIVE

If false, does nothing (doesn't pull packets). One possible use
is to set ACTIVE to false in the configuration, and later
change it to true with a handler from DriverManager element.
The default value is true.

=back

=h count read-only

Returns the count of packets that have passed through Unqueue.

=h active read/write

The same as ACTIVE keyword.

=a RatedUnqueue, BandwidthRatedUnqueue
*/

class Unqueue : public Element { public:

  Unqueue();
  ~Unqueue();

  const char *class_name() const		{ return "Unqueue"; }
  const char *port_count() const		{ return PORTS_1_1; }
  const char *processing() const		{ return PULL_TO_PUSH; }

  int configure(Vector<String> &, ErrorHandler *);
  int initialize(ErrorHandler *);
  void add_handlers();

  bool run_task(Task *);

  static String read_param(Element *e, void *);
  static int write_param(const String &, Element *, void *, ErrorHandler *);

 private:

  bool _active;
  int32_t _burst;
  unsigned _count;
  Task _task;
  NotifierSignal _signal;

};

CLICK_ENDDECLS
#endif
