import serial

# uart2=serial.Serial("/dev/ttyAMA2",115200,timeout=0.5)
uart4=serial.Serial("/dev/ttyAMA4",115200,timeout=0.5)
uart4.write("Hello, world!\n".encode('utf-8'))