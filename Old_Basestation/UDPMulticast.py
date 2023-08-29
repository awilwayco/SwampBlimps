import socket
import struct
from threading import Thread
import time
import select

class UDPHelper:
    def __init__(self):
        #Setup UDP multicast socket
        self.multicast_group = '239.255.255.250'
        self.multicast_port = 1900

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.multicast_group, self.multicast_port))
        self.sock.setblocking(False)

        self.mreq = struct.pack('4sL', socket.inet_aton(self.multicast_group), socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, self.mreq)
        print("Successfully connected to UDP Multicast")

        self.inputMessages = []
        self.looping = False
        self.verboseOutputReceivedMessages = False
        self.verboseOutputSentMessages = False

    def open(self):
        self.thread = Thread(target=self.loopListen)
        self.thread.start()

    def loopListen(self):
        time.sleep(1)
        print("Listening via UDP Multicast")
        self.looping = True
        while(self.looping):
            time.sleep(0.01)
            self.listen()

    #Checks for message prefix == identifier
    def listen(self):
        #print("waiting to receive message")
        try:
            (data, address) = self.sock.recvfrom(1024)
            #data = message content
            #address = (ip,port)
        except:
            pass
        else:
            inString = data.decode(encoding='utf-8', errors='ignore')
            #print(inString)
            if (inString[0:2] == ":)"):
                inString = inString[2:]
                comma = inString.find(",")
                if(inString[0]=="0" and comma == 1):
                    self.inputMessages.append((inString,address[0]))
                    if(self.verboseOutputReceivedMessages):
                        print("\"",inString,"\"",sep='')

    # Message format: identifier+targetID+,+sourceID+:+flag+:+message
    def send(self, targetID, flag, message):
        message = ":)" + str(targetID) + ",0:" + flag + ":" + str(message)
        outBytes = message.encode(encoding='utf-8',errors='ignore')
        self.sock.sendto(outBytes, (self.multicast_group, self.multicast_port))
        if(self.verboseOutputSentMessages):
            print("Sending: \"",message,"\"",sep='')

    def close(self):
        print("UDPMulticast closing...")
        self.looping = False
        self.thread.join()
        self.sock.close()
        print("UDPMulticast closed.")

    def getInputMessages(self):
        return self.inputMessages