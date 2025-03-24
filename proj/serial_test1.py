import serial
import time
from subsystems.AGV import myAGV
from subsystems.AGV import MOVJ_Drection as md

agv_test=myAGV(0x01,"/dev/ttyAMA2",115200)

# interval=0.1
# num=0
# num_end=20
# while(True):
#     agv_test.Velocity_Control([0,100,0])
#     time.sleep(interval)
#     agv_test.Velocity_Control([0,0,0])
#     time.sleep(interval)
#     num+=1
#     print(num)
#     if(num>=num_end):
#         break

# agv_test.Velocity_Control([0,400,0])
# time.sleep(0.5)
agv_test.MOVJ_control([md.Left_Forward,200,89])
# time.sleep(1)
# agv_test.Velocity_Control([0,400,0])
# time.sleep(0.5)
# agv_test.Velocity_Control([0,0,0])

# agv_test.Position_Control([0,0,900])

agv_test.Angle_Correction(90)