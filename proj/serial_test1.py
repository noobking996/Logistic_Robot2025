import serial
import time
from subsystems.AGV import myAGV
from subsystems.AGV import MOVJ_Drection as md
from subsystems.Manipulator import myManipulator,Servo
from mission import Setup
from logging import DEBUG,INFO,WARNING,ERROR,CRITICAL

def agvTest0():
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


def armTest0():
    Public_Logger=Setup.Logger_Setup("armTest0",[DEBUG,DEBUG,DEBUG])
    myServo=Servo("/dev/ttyAMA4",9600)
    arm=myManipulator([(65,130,130),(71,-20-1.12,0)],Public_Logger,myServo)
    yaw_compensation=-5
    arm.Set_Joint_to_Actuator_Matrix([[[90,430],[90-16.8,500]],
                                        [[(180-90),420],[180-(90+19.2),500]],
                                        [[0,750+yaw_compensation],[60,1000+yaw_compensation]]])
    arm.Set_YawAccRatio(0.2,0.25)
    arm.Set_Claw_Angles((800,980))
    arm.Set_Radial_Offset(50)

    arm.Claw_Cmd(False)


def main():
    # agvTest0()
    armTest0()

if (__name__=="__main__"):
    main()