

sock :: Socket(TCP, 0.0.0.0, 7777, HEADROOM 34) 
class :: Classifier(12/0801, -)
FromDevice(eth0) -> class [0] -> Strip(34) -> Print("From Device: ") -> sock
   -> Print("From Socket: ")
  -> EtherEncap(0x0801, 00:06:5B:75:72:A2, 00:1E:EC:CA:F2:F9)
   -> Print("From Ethernet: ")
   -> Queue 
   -> ToDevice(eth0);

class [1] -> Discard;
