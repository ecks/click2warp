#ifndef CLICK_KERNELHANDLERPROXY_HH
#define CLICK_KERNELHANDLERPROXY_HH
#include "elements/userlevel/handlerproxy.hh"
CLICK_DECLS

/*
=c

KernelHandlerProxy([I<KEYWORDS>])

=s control

proxies kernel module handlers at user level

=d

Provides one proxy handler for each handler in a Linux kernel module Click
configuration. The proxy handler for an element C<e>'s handler C<h> is named
`C<e.h>'. Reading KernelHandlerProxy's C<e.h> handler will return the result
of reading kernel element C<e>'s C<h> handler. Similarly, writing a string to
KernelHandlerProxy's C<e.h> handler will cause the proxy to write that string
to kernel element C<e>'s C<h> handler.

Keyword arguments are:

=over 8

=item VERBOSE

Boolean. If true, print chatter messages when read handlers fail. (Errors with
write handlers are reported to the supplied ErrorHandler, but read handlers
don't take an ErrorHandler argument.) Default is false.

=back

=n

KernelHandlerProxy does not decide ahead of time whether a given handler is
active. Thus, for example, KernelHandlerProxy can report that a handler exists
even when no corresponding element exists in the kernel configuration. Any
error will be reported when the handler is actually called.

=a

SocketHandlerProxy

*/

class KernelHandlerProxy : public HandlerProxy { public:

    KernelHandlerProxy();
    ~KernelHandlerProxy();

    const char* class_name() const	{ return "KernelHandlerProxy"; }
    void* cast(const char*);

    int configure(Vector<String>&, ErrorHandler*);

    int check_handler(const String&, bool write, ErrorHandler* errh);

    void add_handlers();
    int llrpc(unsigned, void*);

  private:

    bool _detailed_error_message;
    bool _verbose;
    bool _dot_h_checked;
    bool _dot_h;

    String handler_name_to_file_name(const String &str);
    static int handler_hook(int, String&, Element*, const Handler*, ErrorHandler*);
    static int star_write_handler(const String&, Element*, void*, ErrorHandler*);
    int complain(ErrorHandler*, const String&, int errcode, const String&);
    int complain_about_open(ErrorHandler*, const String&, int);
    int check_handler_name(const String&, ErrorHandler*);

};

CLICK_ENDDECLS
#endif
