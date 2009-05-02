

sock :: Socket(TCP, 0.0.0.0, 7777, HEADROOM 34) 

FromDevice(eth1) -> Strip(34) -> Print("From Device: ") -> sock
   -> Print("From Socket: ")
   -> IPEncap(8, 150.250.190.119, 150.250.190.219)
   -> Print("From IP: ")
   -> IPPrint("From IP: ")
   -> EtherEncap(0x0800, 00:21:70:B8:FB:82, 00:1C:7E:89:E3:8D)
   -> Print("From Ethernet: ")
   -> Queue 
   -> ToDevice(eth1);
