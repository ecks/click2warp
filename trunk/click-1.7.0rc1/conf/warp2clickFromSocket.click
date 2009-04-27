

//sock :: Socket(TCP, 0.0.0.0, 7777) 
//todev :: ToDevice(eth1)

Socket(TCP, 0.0.0.0, 7777)
   -> EtherEncap(0x0800, 00:21:70:B8:FB:82, 00:1E:EC:CA:F2:F9)
   -> Queue 
   -> ToDevice(eth1);
