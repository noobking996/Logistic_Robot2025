import serial
import time
from subsystems.AGV import myAGV

agv_test=myAGV(0x01,"/dev/ttyAMA4",115200)

interval=0.1
num=0
num_end=20
while(True):
    agv_test.Velocity_Control([0,100,0])
    time.sleep(interval)
    agv_test.Velocity_Control([0,0,0])
    time.sleep(interval)
    num+=1
    print(num)
    if(num>=num_end):
        break