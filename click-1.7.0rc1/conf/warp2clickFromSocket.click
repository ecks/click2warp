

//sock :: Socket(TCP, 0.0.0.0, 7777) 
//todev :: ToDevice(eth1)

Socket(TCP, 0.0.0.0, 7777)
   -> Print("From Socket: ")
   -> IPEncap(8, 150.250.190.119, 150.250.190.219)
   -> Print("From IP: ")
   -> IPPrint("From IP: ")
   -> EtherEncap(0x0800, 00:21:70:B8:FB:82, 00:1E:EC:CA:F2:F9)
   -> Print("From Ethernet: ")
   -> Queue 
   -> ToDevice(eth0);
