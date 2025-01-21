import numpy as np
import time

from mission.Setup import MissionDef, MissionDef_t, Correction_PosDef
from subsystems import AGV


# 创建agv对象，指定串口
agv=AGV.myAGV(0x01,"/dev/ttyAMA2",115200)


def Departure_Func(self:MissionDef_t):

    """
    @功能：启停区出发
    @参数列表元素数: 2 [[斜走速度],[直走速度]]
    @时间列表元素数: 2 [[斜走时间],[直走时间]]
    @参数(在函数体中修改):stop_flag, 是否采用静止扫码方案,若不采用,则参数列表元素数为1
    @参数(在函数体中修改):wait_time, 制动等待时间
    """

    # 是否采用静止扫码方案
    stop_flag=False

    # 等待时间：静止扫码方案中为制动等待时间
    wait_time=0

    # 开始斜走，计时
    if(self.Stage_Flag==0):
        self.Change_Stage(1)
        agv.Velocity_Control(self.Para_List[0])
        self.Phase_Start_Time=time.time()
        if(self.Verbose_Flag==True):
            print("Mission({})开始斜走".format(self.Name))

    # 等待斜走完成，开始直走
    elif(self.Stage_Flag==1):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[0]):
            self.Change_Stage(2)
            agv.Velocity_Control(self.Para_List[1])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                print("Mission({})开始直走".format(self.Name))

    # 方案一：采用直走后停止方案，静止扫码
    if(stop_flag==True):
        # 等待直走完成，停止
        if(self.Stage_Flag==2):
            if((time.time()-self.Phase_Start_Time)>=self.Time_List[1]):
                self.Change_Stage(3)
                agv.Velocity_Control([0,0,0])
                self.Phase_Start_Time=time.time()
                print("Mission({}) 开始制动".format(self.Name))

        # 等待制动完成，结束
        elif(self.Stage_Flag==3):
            if((time.time()-self.Phase_Start_Time)>=wait_time):
                self.Change_Stage(5)
                print("Mission({}) 制动等待完成".format(self.Name))

        elif(self.Stage_Flag==4):
            self.End()

    # 方案二：采用边走边扫码方案
    else:
        # 开始直走后直接结束任务
        if(self.Stage_Flag==2):
            self.End()


def Scan_QRcode_Func(self:MissionDef):
    """
    @功能：扫码
    @参数: scanning_time, 扫码时间
    """
    scanning_time=2.5

    # 开始扫码(扫码功能暂未开发，用计时代替)
    if(self.Stage_Flag==0):
        self.Change_Stage(1)
        self.Phase_Start_Time=time.time()
        if(self.Verbose_Flag==True):
            print("Mission({}) 开始扫码".format(self.Name))

    elif(self.Stage_Flag==1):
        if((time.time()-self.Phase_Start_Time)>=scanning_time):
            self.Change_Stage(2)
            if(self.Verbose_Flag==True):
                print("Mission({}) 扫码完成".format(self.Name))

    else:
        self.End()


def QRcode_2_RawMaterial_Func(self:MissionDef_t):

    """
    @功能: 扫码->原料
    @参数列表元素数: 1 [[直走速度]]
    @时间列表元素数: 1 [直走时间]
    @参数: stop_wait_time, 制动等待时间
    """

    stop_wait_time=0

    # 开始直走，计时
    if(self.Stage_Flag==0):
        self.Change_Stage(1)
        agv.Velocity_Control(self.Para_List[0])
        self.Phase_Start_Time=time.time()
        if(self.Verbose_Flag==True):
            print("Mission({}) 开始直走".format(self.Name))
    
    # 直走结束后，开始制动,计时
    elif(self.Stage_Flag==1):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[0]):
            self.Change_Stage(2)
            agv.Velocity_Control([0,0,0])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                print("Mission({}) 开始制动".format(self.Name))

    elif(self.Stage_Flag==2):
        if((time.time()-self.Phase_Start_Time)>=stop_wait_time):
            self.Change_Stage(3)
            if(self.Verbose_Flag==True):
                print("Mission({}) 制动等待完成".format(self.Name))
    
    else:
        self.End()


def Pos_Correction_Func(self:MissionDef):

    """
    @功能: 位置纠正(原料区、加工区、暂存区)(功能暂未开发,使用延时代替)
    @参数列表元素数: ? [[纠正地点],[...]]
    @参数: correction_time, 纠正时间
    """

    # 纠正时间
    correction_time=2

    if(self.Stage_Flag==0):
        self.Change_Stage(1)
        self.Phase_Start_Time=time.time()
        if(self.Verbose_Flag==True):
            print("Mission({}) 开始位置纠正".format(self.Name))
    
    elif(self.Stage_Flag==1):
        if((time.time()-self.Phase_Start_Time)>=correction_time):
            self.Change_Stage(2)
            if(self.Verbose_Flag==True):
                print("Mission({}) 位置纠正完毕".format(self.Name))
            
    else:
        self.End()


def RawMaterial_Picking_Func(self:MissionDef):
    """
    @功能: 原料区夹取(功能暂未开发,直接跳过)
    """
    self.End()


def RawMaterial_2_Processing_Func(self:MissionDef_t):
    """
    @功能：原料区->加工区
    @参数列表元素数: 5 [[直走速度],[圆弧转弯参数],[直走速度],[圆弧转弯参数],[直走速度]]
    @时间列表元素数: 5 [直走时间,圆弧转弯时间,直走时间,圆弧转弯时间,直走时间]
    @参数: stop_wait_time, 制动等待时间
    """
    stop_wait_time=0

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
            agv.MOVJ_control(self.Para_List[1])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                print("Mission({}) 开始圆弧转弯".format(self.Name))

    # 等待圆弧转弯完成，开始直走
    elif(self.Stage_Flag==2):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[1]):
            self.Change_Stage(3)
            agv.Velocity_Control(self.Para_List[2])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                print("Mission({}) 开始直走".format(self.Name))

    # 等待直走完成，开始圆弧转弯
    elif(self.Stage_Flag==3):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[2]):
            self.Change_Stage(4)
            agv.MOVJ_control(self.Para_List[3])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                print("Mission({}) 开始圆弧转弯".format(self.Name))
        
    # 等待圆弧转弯完成，开始直走
    elif(self.Stage_Flag==4):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[3]):
            self.Change_Stage(5)
            agv.Velocity_Control(self.Para_List[4])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                print("Mission({}) 开始直走".format(self.Name))

    # 等待直走完成，停止
    elif(self.Stage_Flag==5):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[4]):
            self.Change_Stage(6)
            agv.Velocity_Control([0,0,0])
            self.Phase_Start_Time=time.time()
            print("Mission({}) 开始制动".format(self.Name))

    # 等待制动完成，结束
    elif(self.Stage_Flag==6):
        if((time.time()-self.Phase_Start_Time)>=stop_wait_time):
            self.Change_Stage(7)
            print("Mission({}) 制动等待完成".format(self.Name))

    else:
        self.End()


def Processing_PickAndPlace_Func(self:MissionDef):
    """
    @功能: 加工区放置回收(功能暂未开发，直接跳过)
    """
    self.End()


def Three_Section_Turn_Func(self:MissionDef_t):
    """
    @功能: 三段(2直线+1圆弧)转弯,用于加工区->暂存区 或 暂存区->原料区
    @参数列表元素数: 3 [[直走速度],[圆弧转弯参数],[直走速度]]
    @时间列表元素数: 3 [直走时间,圆弧转弯时间,直走时间]
    @参数: stop_wait_time, 制动等待时间
    """
    stop_wait_time=0

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
            agv.MOVJ_control(self.Para_List[1])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                print("Mission({}) 开始圆弧转弯".format(self.Name))

    # 等待圆弧转弯完成，开始直走
    elif(self.Stage_Flag==2):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[1]):
            self.Change_Stage(3)
            agv.Velocity_Control(self.Para_List[2])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
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
        if((time.time()-self.Phase_Start_Time)>=stop_wait_time):
            self.Change_Stage(5)
            print("Mission({}) 制动等待完成".format(self.Name))

    else:
        self.End()


#第一轮专属
def Storage_Place_Func(self:MissionDef):
    """
    @功能: 暂存区放置(功能暂未开发，直接跳过)
    """
    self.End()


# 第二轮专属
def Storage_Stacking_Func(self:MissionDef):
    """
    @功能: 暂存区码垛(功能暂未开发，直接跳过)
    """
    self.End()


def Storage_Go_Home_Func(self:MissionDef_t):
    """
    @功能: 暂存区->启停区
    @参数列表元素数: 4 [[直走速度],[圆弧转弯参数],[直走速度],[斜走速度]]
    @时间列表元素数: 4 [直走时间,圆弧转弯时间,直走时间,斜走时间]
    @参数: stop_wait_time, 制动等待时间
    """
    stop_wait_time=0

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
            agv.MOVJ_control(self.Para_List[1])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                print("Mission({}) 开始圆弧转弯".format(self.Name))

    # 等待圆弧转弯完成，开始直走
    elif(self.Stage_Flag==2):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[1]):
            self.Change_Stage(3)
            agv.Velocity_Control(self.Para_List[2])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                print("Mission({}) 开始直走".format(self.Name))

    # 等待直走完成，开始斜走
    elif(self.Stage_Flag==3):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[2]):
            self.Change_Stage(4)
            agv.Velocity_Control(self.Para_List[3])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                print("Mission({}) 开始斜走".format(self.Name))
        
    # 等待直走完成，停止
    elif(self.Stage_Flag==4):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[3]):
            self.Change_Stage(5)
            agv.Velocity_Control([0,0,0])
            self.Phase_Start_Time=time.time()
            print("Mission({}) 开始制动".format(self.Name))

    # 等待制动完成，结束
    elif(self.Stage_Flag==5):
        if((time.time()-self.Phase_Start_Time)>=stop_wait_time):
            self.Change_Stage(6)
            print("Mission({}) 制动等待完成".format(self.Name))

    else:
        self.End()


def Home_Pos_Correction_Func(self:MissionDef):
    """
    @功能: 启停区位置纠正(功能暂未开发，直接跳过)
    """
    self.End()