

sock :: Socket(TCP, 0.0.0.0, 7777, HEADROOM 34) 
class :: Classifier(23/E7, -)
FromDevice(eth1) -> class [0] -> Strip(34) -> Print("From Device: ") -> sock
   -> Print("From Socket: ")
   -> IPEncap(231, 150.250.190.119, 150.250.190.219)
   -> Print("From IP: ")
   -> IPPrint("From IP: ")
   -> EtherEncap(0x0800, 00:21:70:B8:FB:82, 00:1E:EC:CA:F2:F9)
   -> Print("From Ethernet: ")
   -> Queue 
   -> ToDevice(eth1);

class [1] -> Discard;
