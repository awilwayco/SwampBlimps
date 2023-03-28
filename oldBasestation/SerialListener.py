import sys
import serial
from serial.tools import list_ports

portIndex = 0
    
#List out available ports
ports = list_ports.comports()
for port in ports:
    print("Port",port[0])

#Establish connection with first port
ser = serial.Serial(ports[portIndex][0],115200,timeout=0.2)
while not ser.is_open:
    print("Attempting to open port.")
    ser.open()
print("Serial port is open on ",ports[portIndex][0],".",sep='')
ser.flush()
print("Serial port flushed.")

"""
test1 = 'abcd'
print(test1)
print(type(test1))

test2 = b'abcd'
print(test2)
print(type(test2))

test3= b'abcd\xff'
print(test3)
print(type(test3))

test4= b'ab\xffcd'
print("index:")
#print(test4.index(100))
print(len(test4))
for p in test4:
    print(p)
print(test4.decode('utf8', errors='ignore'))

sys.exit()
"""
debug = False

while True:
    inString = ""
    if(ser.in_waiting > 0):
        if debug:
            try:
                inString = ser.readline()
                decoded = inString.decode('utf-8')
                print(inString)
            except:
                print("Error occurred, inString: ",inString)
        else:
            inString = ser.readline()
            decoded = inString.decode('utf-8', errors='ignore')
            print(decoded)
