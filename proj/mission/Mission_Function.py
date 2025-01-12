import numpy as np
import time

from mission.Setup import MissionManager, MissionDef, MissionDef_t
from subsystems import AGV


# 创建agv对象，指定串口
agv=AGV.myAGV(0x01,"/dev/ttyAMA2",115200)


def RawMaterial_2_Processing_Func(self:MissionDef_t):
    """
    @参数：self：MissionDef_t实例
    @作用：原料区->加工区
    """
    # 开始直走，计时
    if(self.Stage_Flag==0):
        self.Change_Stage(1)
        agv.Velocity_Control(self.Para_List[0])
        self.Phase_Start_Time=time.time()
        if(self.Verbose_Flag==True):
            print("Mission({}) 开始直走".format(self.Name))

    # 等待直走完成，开始圆弧转弯
    elif(self.Stage_Flag==1):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[0]):
            self.Change_Stage(2)
            agv.MOVJ_control(AGV.MOVJ_Drection.Left_Forward,self.Para_List[1])
            self.Phase_Start_Time=time.time()
            print("Mission({}) 开始圆弧转弯".format(self.Name))

    # 等待圆弧转弯完成，开始直走
    elif(self.Stage_Flag==2):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[1]):
            self.Change_Stage(3)
            agv.Velocity_Control(self.Para_List[2])
            self.Phase_Start_Time=time.time()
            print("Mission({}) 开始直走".format(self.Name))

    # 等待直走完成，停止
    elif(self.Stage_Flag==3):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[2]):
            self.Change_Stage(4)
            agv.Velocity_Control([0,0,0])
            self.Phase_Start_Time=time.time()
            print("Mission({}) 开始制动".format(self.Name))

    # 等待制动完成，结束
    elif(self.Stage_Flag==4):
        stop_wait_time=0
        if((time.time()-self.Phase_Start_Time)>=stop_wait_time):
            self.Change_Stage(5)
            print("Mission({}) 制动等待完成".format(self.Name))

    else:
        self.End()