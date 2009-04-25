import asyncore
import socket, time


class PacketToClick(asyncore.dispatcher):

    def __init__(self, host, port=7777):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect((host, port))
	self.buffer = "31323334"
	self.buffer = self.buffer.decode("hex")

    def writable(self):
        return (len(self.buffer) > 0)

    def handle_write(self):
	sent = self.send(self.buffer)
	self.buffer = self.buffer[sent:]
	if len(self.buffer) == 0:
          self.buffer = raw_input("Type in your packet: ")
	  self.buffer = self.buffer.decode("hex")
	  print self.buffer

    def handle_connect(self):
        pass # connection succeeded

    def handle_expt(self): 
	self.close() # connection failed, shutdown 

    def handle_read(self):
	print self.recv(8192)

    def handle_close(self):
        self.close()

request = PacketToClick("localhost")

asyncore.loop()

