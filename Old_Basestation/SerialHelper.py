import serial
from serial.tools import list_ports
import time
from threading import Thread
from multiprocessing import Process

class SerialHelper:
    def __init__(self):
        portIndex = 0
        self.inputMessages = []
        #List out available ports
        ports = list_ports.comports()
        if(len(ports)>0):
            print("Available Serial Ports:")
            for port in ports:
                print("Port", port[0])
        else:
            while(len(ports)<1):
                print("No Available Serial Ports.")
                time.sleep(1)
                ports = list_ports.comports()

        # Establish connection with first port
        ser = serial.Serial(ports[portIndex][0], 115200, timeout=0.1)
        while not ser.is_open:
            print("Attempting to open port.")
            ser.open()
        print("Serial port is open on ", ports[portIndex][0], ".", sep='')
        ser.flush()
        print("Serial port flushed.")
        self.ser = ser

    #Message format: identifier+targetID+,+sourceID+:+flag+:+message
    def send(self, targetID, flag, message):
        message = str(targetID) + ",0:" + flag + ":" + str(message) + "\n"
        self.ser.write(message.encode('utf-8'))
        #print("Tx:",message,sep='',end='')

    def open(self, frequency):
        self.frequency = frequency
        self.thread = Thread(target=self.loopListen)
        self.thread.start()
        #self.process = Process(target=self.loopListen)
        #self.process.start()

    def close(self):
        print("Closing SerialHelper")
        self.looping = False
        self.thread.join()
        self.ser.close()

    def loopListen(self):
        time.sleep(1)
        delaySec = 1/self.frequency
        print("Reading serial channel with delay:",delaySec,"seconds")
        self.looping = True
        lastTime = time.time()
        while(self.looping):
            currentTime = time.time()
            diff = currentTime - lastTime
            if(diff > delaySec):
                #print("SHDelay:",diff)
                lastTime = currentTime
                self.listen()

    def listen(self):
        ser = self.ser
        while (ser.in_waiting > 0):
            buffer = ""
            while True:
                newByte = ser.read(1)
                if(newByte == b'\r'):
                    ser.read(1)
                    break
                buffer += (newByte.decode(encoding='utf-8',errors='ignore'))
            inString = buffer#.decode(encoding='utf-8',errors='ignore')
            """
            data = ser.read(ser.in_waiting)
            for b in data:
                buffer.append(b)
            """
            #buffer.append(ser.read(ser.in_waiting))
            #print(buffer)
            #inString = buffer
            #inString = ser.readline()#.decode('utf8', errors='ignore').strip()
            print(inString)
            #print("InWaiting:",ser.in_waiting)
            if (inString[0:3] == "Rx:"):
                inString = inString[3:]
                self.inputMessages.append(inString)
            #print(times)
        #print("exiting while loop")

    def getInputMessages(self):
        return self.inputMessages