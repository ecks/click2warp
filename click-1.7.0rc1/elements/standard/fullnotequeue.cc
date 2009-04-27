// -*- c-basic-offset: 4 -*-
/*
 * fullnotequeue.{cc,hh} -- queue element that notifies on full
 * Eddie Kohler
 *
 * Copyright (c) 2004-2007 Regents of the University of California
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, subject to the conditions
 * listed in the Click LICENSE file. These conditions include: you must
 * preserve this copyright notice, and you cannot mention the copyright
 * holders in advertising related to the Software without their permission.
 * The Software is provided WITHOUT ANY WARRANTY, EXPRESS OR IMPLIED. This
 * notice is a summary of the Click LICENSE file; the license in that file is
 * legally binding.
 */

#include <click/config.h>
#include "fullnotequeue.hh"
CLICK_DECLS

FullNoteQueue::FullNoteQueue()
{
}

FullNoteQueue::~FullNoteQueue()
{
}

void *
FullNoteQueue::cast(const char *n)
{
    if (strcmp(n, "FullNoteQueue") == 0)
	return (FullNoteQueue *)this;
    else if (strcmp(n, Notifier::FULL_NOTIFIER) == 0)
	return static_cast<Notifier*>(&_full_note);
    else
	return NotifierQueue::cast(n);
}

int
FullNoteQueue::configure(Vector<String> &conf, ErrorHandler *errh)
{
    _full_note.initialize(Notifier::FULL_NOTIFIER, router());
    _full_note.set_active(true, false);
    return NotifierQueue::configure(conf, errh);
}

int
FullNoteQueue::live_reconfigure(Vector<String> &conf, ErrorHandler *errh)
{
    int r = NotifierQueue::live_reconfigure(conf, errh);
    if (r >= 0 && size() < capacity() && _q)
	_full_note.wake();
    return r;
}

void
FullNoteQueue::push(int, Packet *p)
{
    // Code taken from SimpleQueue::push().
    int h = _head, t = _tail, nt = next_i(t);

    if (nt != h)
	push_success(h, t, nt, p);
    else
	push_failure(p);
}

Packet *
FullNoteQueue::pull(int)
{
    // Code taken from SimpleQueue::deq.
    int h = _head, t = _tail, nh = next_i(h);

    if (h != t)
	return pull_success(h, t, nh);
    else
	return pull_failure();
}

CLICK_ENDDECLS
ELEMENT_REQUIRES(NotifierQueue)
EXPORT_ELEMENT(FullNoteQueue FullNoteQueue-FullNoteQueue)