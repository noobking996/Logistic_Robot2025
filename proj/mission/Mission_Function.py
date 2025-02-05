import numpy as np
import time
from typing import List,Union,Callable,Any
from logging import Logger,DEBUG,INFO,WARNING,ERROR,CRITICAL


from mission.Setup import MissionDef, MissionDef_t, MissionManager ,Correction_PosDef
from subsystems import AGV
from subsystems.Computer_Vision import Video_Stream

# from cv2 import VideoCapture, VideoWriter
import cv2 as cv
from enum import Enum
import pyzbar as pz
from subsystems.Keyboard import Keyboard_Enum as kb


# 创建agv对象，指定串口
agv=AGV.myAGV(0x01,"/dev/ttyAMA2",115200)

# 摄像头捕获图像
frame_captured=None

# 键盘按键状态
key=None


def Frame_Capture_Func(self:MissionDef):
    global frame_captured
    video:Video_Stream=self.Para_List[0][0]
    retval,frame_captured=video.Read_Frame()
    if(retval==False):
        raise Exception("Frame_Capture:帧捕获失败")

def Frame_Capture_Trigger(self:MissionManager):
    # 开始后直接触发
    trigger_condition=(self.Stage_Flag>=self.Para_List[0][0])
    P_Mission=self.Permanent_Mission_LIst[0]
    error_flag=P_Mission.Run_Triggered_By(trigger_condition)
    return error_flag

def Frame_Capture_Callback(self:MissionDef):
    video:Video_Stream=self.Para_List[0][0]
    video.Release_VideoCapture()
    

def Frame_Mark_Display_Func(self:MissionDef):
    """
    @功能:标记帧
    @两种模式:
    1. standby阶段:在帧上标记十字,显示standby,并刷新帧显示
    2. 出发后:在帧上标记十字,录制时间,并刷新帧显示
    """
    global frame_captured
    global key
    video:Video_Stream=self.Para_List[0][0]
    # 标记十字
    video.Mark_Cross(frame_captured)
    # 标记时间
    annote_text=None
    if(self.Stage_Flag==0):
        # 显示standby字样
        annote_text="Standing By"
    elif(self.Stage_Flag==1):
        # 显示录制时间
        current_time=round((time.time()-self.Phase_Start_Time),1)
        annote_text="{}s".format(current_time)
    video.Mark_Text(frame_captured,annote_text)
    # 更新窗口
    video.Update_Window(frame_captured)
    # 查询按键状态
    key=(cv.pollKey() & 0xFF)
    if(key==ord('q')):
        raise Exception("Frame_Mark_Save: Keyboard Interrupt")

def Frame_Mark_Display_Callback(self:MissionDef):
    cv.destroyAllWindows()

def Frame_Save_Func(self:MissionDef):
    video:Video_Stream=self.Para_List[0][0]
    video.Save_Frame(frame_captured)

def Frame_Save_Callback(self:MissionDef):
    video:Video_Stream=self.Para_List[0][0]
    video.Release_VideoWriter()

def Frame_Mark_Save_Trigger(self:MissionManager):
    # 开始后直接触发mark+dsplay
    trigger_condition=(self.Stage_Flag>=self.Para_List[0][1])
    P_Mission=self.Permanent_Mission_LIst[1]
    error_flag=P_Mission.Run_Triggered_By(trigger_condition)
    if(error_flag==True):
        return True
    # 出发后,显示时间并开始录像
    trigger_condition=(self.Stage_Flag>=self.Para_List[0][2])
    if(trigger_condition==True):
        if(P_Mission.Stage_Flag==0):
            P_Mission.Phase_Start_Time=time.time()
            P_Mission.Change_Stage(1)
    # save
    P_Mission=self.Permanent_Mission_LIst[2]
    error_flag=P_Mission.Run_Triggered_By(trigger_condition)
    return error_flag


def Standby_Func(self:MissionDef):
    """
    @功能：待机
    @参数列表元素数: 1 [[待机时间]]
    """
    wait_time=self.Para_List[0][0]
    if(self.Stage_Flag==0):
        self.Change_Stage(1)
        if(self.Verbose_Flag==True):
            self.Output("Mission({}) 准备出发...".format(self.Name))
    elif(self.Stage_Flag==1):
        if(time.time()-self.Start_Time>=wait_time):
            self.Change_Stage(2)
    elif(self.Stage_Flag==2):
        self.End()


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
            self.Output("Mission({})开始斜走".format(self.Name))

    # 等待斜走完成，开始直走
    elif(self.Stage_Flag==1):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[0]):
            self.Change_Stage(2)
            agv.Velocity_Control(self.Para_List[1])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                self.Output("Mission({})开始直走".format(self.Name))

    # 方案一：采用直走后停止方案，静止扫码
    if(stop_flag==True):
        # 等待直走完成，停止
        if(self.Stage_Flag==2):
            if((time.time()-self.Phase_Start_Time)>=self.Time_List[1]):
                self.Change_Stage(3)
                agv.Velocity_Control([0,0,0])
                self.Phase_Start_Time=time.time()
                self.Output("Mission({}) 开始制动".format(self.Name))

        # 等待制动完成，结束
        elif(self.Stage_Flag==3):
            if((time.time()-self.Phase_Start_Time)>=wait_time):
                self.Change_Stage(5)
                self.Output("Mission({}) 制动等待完成".format(self.Name))

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
    # True则扫描二维码,否则用延时代替
    scan_flag=self.Para_List[0][0]
    if(scan_flag==True):
        if(self.Stage_Flag==0):
            self.Change_Stage(1)
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 开始扫码".format(self.Name))
            elif(self.Stage_Flag==1):
                frame_th=cv.inRange(frame_captured,np.array([0,0,0]),np.array([255,255,255]))
                self.Change_Stage(2)
    else:
        scanning_time=self.Para_List[0][1]
        # 开始扫码(扫码功能暂未开发，用计时代替)
        if(self.Stage_Flag==0):
            self.Change_Stage(1)
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 开始扫码".format(self.Name))
        elif(self.Stage_Flag==1):
            if((time.time()-self.Phase_Start_Time)>=scanning_time):
                self.Change_Stage(2)
                if(self.Verbose_Flag==True):
                    self.Output("Mission({}) 扫码完成".format(self.Name))
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
            self.Output("Mission({}) 开始直走".format(self.Name))
    
    # 直走结束后，开始制动,计时
    elif(self.Stage_Flag==1):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[0]):
            self.Change_Stage(2)
            agv.Velocity_Control([0,0,0])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 开始制动".format(self.Name))

    elif(self.Stage_Flag==2):
        if((time.time()-self.Phase_Start_Time)>=stop_wait_time):
            self.Change_Stage(3)
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 制动等待完成".format(self.Name))
    
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
            self.Output("Mission({}) 开始位置纠正".format(self.Name))
    
    elif(self.Stage_Flag==1):
        if((time.time()-self.Phase_Start_Time)>=correction_time):
            self.Change_Stage(2)
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 位置纠正完毕".format(self.Name))
            
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
            self.Output("Mission({}) 开始直走".format(self.Name))

    # 等待直走完成，开始圆弧转弯
    elif(self.Stage_Flag==1):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[0]):
            self.Change_Stage(2)
            agv.MOVJ_control(self.Para_List[1])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 开始圆弧转弯".format(self.Name))

    # 等待圆弧转弯完成，开始直走
    elif(self.Stage_Flag==2):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[1]):
            self.Change_Stage(3)
            agv.Velocity_Control(self.Para_List[2])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 开始直走".format(self.Name))

    # 等待直走完成，开始圆弧转弯
    elif(self.Stage_Flag==3):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[2]):
            self.Change_Stage(4)
            agv.MOVJ_control(self.Para_List[3])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 开始圆弧转弯".format(self.Name))
        
    # 等待圆弧转弯完成，开始直走
    elif(self.Stage_Flag==4):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[3]):
            self.Change_Stage(5)
            agv.Velocity_Control(self.Para_List[4])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 开始直走".format(self.Name))

    # 等待直走完成，停止
    elif(self.Stage_Flag==5):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[4]):
            self.Change_Stage(6)
            agv.Velocity_Control([0,0,0])
            self.Phase_Start_Time=time.time()
            self.Output("Mission({}) 开始制动".format(self.Name))

    # 等待制动完成，结束
    elif(self.Stage_Flag==6):
        if((time.time()-self.Phase_Start_Time)>=stop_wait_time):
            self.Change_Stage(7)
            self.Output("Mission({}) 制动等待完成".format(self.Name))

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
            self.Output("Mission({}) 开始直走".format(self.Name))

    # 等待直走完成，开始圆弧转弯
    elif(self.Stage_Flag==1):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[0]):
            self.Change_Stage(2)
            agv.MOVJ_control(self.Para_List[1])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 开始圆弧转弯".format(self.Name))

    # 等待圆弧转弯完成，开始直走
    elif(self.Stage_Flag==2):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[1]):
            self.Change_Stage(3)
            agv.Velocity_Control(self.Para_List[2])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 开始直走".format(self.Name))

    # 等待直走完成，停止
    elif(self.Stage_Flag==3):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[2]):
            self.Change_Stage(4)
            agv.Velocity_Control([0,0,0])
            self.Phase_Start_Time=time.time()
            self.Output("Mission({}) 开始制动".format(self.Name))

    # 等待制动完成，结束
    elif(self.Stage_Flag==4):
        if((time.time()-self.Phase_Start_Time)>=stop_wait_time):
            self.Change_Stage(5)
            self.Output("Mission({}) 制动等待完成".format(self.Name))

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
            self.Output("Mission({}) 开始直走".format(self.Name))

    # 等待直走完成，开始圆弧转弯
    elif(self.Stage_Flag==1):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[0]):
            self.Change_Stage(2)
            agv.MOVJ_control(self.Para_List[1])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 开始圆弧转弯".format(self.Name))

    # 等待圆弧转弯完成，开始直走
    elif(self.Stage_Flag==2):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[1]):
            self.Change_Stage(3)
            agv.Velocity_Control(self.Para_List[2])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 开始直走".format(self.Name))

    # 等待直走完成，开始斜走
    elif(self.Stage_Flag==3):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[2]):
            self.Change_Stage(4)
            agv.Velocity_Control(self.Para_List[3])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 开始斜走".format(self.Name))
        
    # 等待直走完成，停止
    elif(self.Stage_Flag==4):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[3]):
            self.Change_Stage(5)
            agv.Velocity_Control([0,0,0])
            self.Phase_Start_Time=time.time()
            self.Output("Mission({}) 开始制动".format(self.Name))

    # 等待制动完成，结束
    elif(self.Stage_Flag==5):
        if((time.time()-self.Phase_Start_Time)>=stop_wait_time):
            self.Change_Stage(6)
            self.Output("Mission({}) 制动等待完成".format(self.Name))

    else:
        self.End()


def Home_Pos_Correction_Func(self:MissionDef):
    """
    @功能: 启停区位置纠正(功能暂未开发，直接跳过)
    """
    self.End()


def Thresholding_Test_Func(self:MissionDef):
    """
    @功能: 二值化阈值测试
    @操作指南:
    1. 点击'shift'键切换调参通道,分别为b、g、r通道,以及灰度模式;
    2. 点击'<-/->'键切换高、低阈值;
    3. 点击'up/down'键调整阈值高低
    4. 点击'Tab'键切换调整精确度
    5. 点击'q'键结束测试
    @ 参数列表: 
    1. 初始高、低阈值数组列表[b,g,r];
    2. [阈值状态位(true:高阈值 false:低阈值), 精确度状态位(true:粗调 false:精调)]
    """
    
    threshold_status:List[bool]=self.Para_List[3]
    th_HighOrLow=threshold_status[0]
    # 阈值索引,由th_HighOrLow控制,决定调整高/低阈值
    threshold_indexs=None
    if(th_HighOrLow==True):
        threshold_indexs=(1,0)
    else:
        threshold_indexs=(0,1)
    th_CoarseOrPrecise=threshold_status[1]
    delta_Coarse=10
    delta_Precise=1
    # 精度索引,由th_CoarseOrPrecise控制,决定调整值大小
    threshold_delta=None
    if(th_CoarseOrPrecise==True):
        threshold_delta=delta_Coarse
    else:
        threshold_delta=delta_Precise

    # b_th:List[np.uint8]=self.Para_List[0]
    # g_th:List[np.uint8]=self.Para_List[1]
    # r_th:List[np.uint8]=self.Para_List[2]
    channal_th:List=self.Para_List[self.Stage_Flag-1]

    global key
    if(key==kb.SHIFT.value):
        #点击该键,stage_flag++,在[1,4]区间内切换
        current_stage=self.Stage_Flag
        if(current_stage==4):
            current_stage=1
            channal_th_0:List=self.Para_List[0]
            self.Output("Mission({}) B通道,当前阈值:({},{})"
                            .format(self.Name,channal_th_0[0],channal_th_0[1]),INFO)
        else:
            current_stage+=1
            if(current_stage==2):
                self.Output("Mission({}) G通道,当前阈值:({},{})"
                        .format(self.Name,channal_th[0],channal_th[1]),INFO)
            elif(current_stage==3):
                self.Output("Mission({}) R通道,当前阈值:({},{})"
                        .format(self.Name,channal_th[0],channal_th[1]),INFO)
            elif(current_stage==4):
                self.Output("Mission({}) 灰度模式".format(self.Name),INFO)
        self.Change_Stage(current_stage)
    elif(key==kb.LEFT_ARROW.value):
        self.Output("Mission({}) 调整低阈值".format(self.Name),INFO)
        if(th_HighOrLow==True):
            th_HighOrLow=False
            self.Para_List[3][0]=False
    elif(key==kb.RIGHT_ARROW.value):
        self.Output("Mission({}) 调整高阈值".format(self.Name),INFO)
        if(th_HighOrLow==False):
            th_HighOrLow=True
            self.Para_List[3][0]=True
    elif(key==kb.UP_ARROW.value):
        if(self.Stage_Flag<4):
            th_modified=channal_th[threshold_indexs[0]]+threshold_delta
            if(threshold_indexs[0]==1):
                # 高阈值不可大于255
                if(th_modified>255):
                    th_modified=255
            else:
                # 低阈值必须低于高阈值
                if(th_modified>=channal_th[threshold_indexs[1]]):
                    th_modified=channal_th[threshold_indexs[1]]-1
            channal_th[threshold_indexs[0]]=th_modified
            self.Para_List[self.Stage_Flag-1][threshold_indexs[0]]=th_modified
            self.Output("Mission({}) 调整当前通道阈值为({},{})"
                        .format(self.Name,channal_th[0],channal_th[1]),INFO)
    elif(key==kb.DOWN_ARROW.value):
        if(self.Stage_Flag<4):
            th_modified=channal_th[threshold_indexs[0]]-threshold_delta
            if(threshold_indexs[0]==0):
                # 低阈值不可小于0
                if(th_modified<0):
                    th_modified=0
            else:
                # 高阈值必须高于低阈值
                if(th_modified<=channal_th[threshold_indexs[1]]):
                    th_modified=channal_th[threshold_indexs[1]]+1
            channal_th[threshold_indexs[0]]=th_modified
            self.Para_List[self.Stage_Flag-1][threshold_indexs[0]]=th_modified
            self.Output("Mission({}) 调整当前通道阈值为({},{})"
                        .format(self.Name,channal_th[0],channal_th[1]),INFO)
    elif(key==kb.ESC.value):
        self.Change_Stage(5)
    elif(key==kb.TAB.value):
        if(th_CoarseOrPrecise==True):
            th_CoarseOrPrecise=False
            self.Para_List[3][1]=False
            self.Output("Mission({}) 精调模式".format(self.Name),INFO)
        else:
            th_CoarseOrPrecise=True
            self.Para_List[3][1]=True
            self.Output("Mission({}) 粗调模式".format(self.Name),INFO)

    global frame_captured
    th_3channals=np.zeros((2,3),dtype=np.uint8)
    for i in range(2):
        for j in range(3):
            th_3channals[i][j]=self.Para_List[j][i]
    if(self.Stage_Flag==0):
        self.Change_Stage(1)
        channal_th_0:List=self.Para_List[0]
        self.Output("Mission({}) B通道,当前阈值:({},{})"
                        .format(self.Name,channal_th_0[0],channal_th_0[1]),INFO)
    elif(self.Stage_Flag<5):
        if(self.Stage_Flag<4):
            # 根据三通道阈值对帧图像进行二值化
            frame_captured=cv.inRange(frame_captured,th_3channals[0],th_3channals[1])
        elif(self.Stage_Flag==4):
            # 以灰度模式显示二值化图像
            frame_captured=cv.cvtColor(frame_captured,cv.COLOR_BGR2GRAY)
        frame_captured=cv.cvtColor(frame_captured,cv.COLOR_GRAY2BGR)
    elif(self.Stage_Flag==5):
       self.Output("Mission({}) 测试结束,阈值:{}".format(self.Name,th_3channals),INFO)
       self.End()