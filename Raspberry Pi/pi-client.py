import serial
port = serial.Serial("COM13", baudrate=115200,timeout = 3.0)

while True:
    port.write("{!r}".format("Say something"))
    rcv = port.read(10)
    port.write("{!r}".format("Say something"))
    port.write("\r\nYou sent:" + repr(rcv))