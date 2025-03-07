import numpy as np
import time
from typing import List,Union,Callable,Any,Tuple
from logging import Logger,DEBUG,INFO,WARNING,ERROR,CRITICAL

from mission.Setup import MissionDef,MissionDef_t,MissionManager,myObject
from mission.Setup import Correction_PosDef as CP
from subsystems import AGV,Manipulator
from subsystems.Computer_Vision import Video_Stream
from subsystems.Buttom import myButtom

import mission.Math_Tools as MT
import mission.Reusable_Module as RM

# from cv2 import VideoCapture, VideoWriter
import cv2 as cv
from enum import Enum
from subsystems.Keyboard import Keyboard_Enum as kb
import pyzbar.pyzbar as pz
from pyzbar.pyzbar import Decoded


# 创建agv对象，指定串口
agv=AGV.myAGV(0x01,"/dev/ttyAMA2",115200)

# 创建舵机和机械臂对象
myServo=Manipulator.Servo("/dev/ttyAMA4",9600)
arm:Manipulator.myManipulator=None

# 摄像头捕获图像
frame_captured=None

# 键盘按键状态
key=None

# 车载启动按键
buttom=myButtom(14,0.1)

# 任务顺序码列表
rgb_order_list=[]

# 任务码显示窗口
show_missionCode=True
missionCode_window="mission_code"

# 原料盘对象
material_plate:myObject=None

# 加工/暂存区色环对象(用于位置纠正)
green_ring:myObject=None

# 加工/暂存区边缘线对象(用于角度纠正)
edge_line:myObject=None

# 物块列表,表中的顺序决定物块的编号
stuff_list=[]
def Stuff_List_Init(stuffs:Tuple[myObject]):
    for stuff in stuffs:
        stuff_list.append(stuff)

# 均值滤波器
myfilter=MT.Average_Filter(5)

# 物料夹取任务计数器
stuff_index_counter=RM.Module_Stage_Counter()
# 轮次计数器
round_counter=RM.Module_Stage_Counter()

def Frame_Capture_Func(self:MissionDef):
    global frame_captured
    video:Video_Stream=self.Video
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
    self.Video.Release_VideoCapture()
    

def Frame_Mark_Display_Func(self:MissionDef):
    """
    @功能:标记帧
    @两种模式:
    1. standby阶段:在帧上标记十字,显示standby,并刷新帧显示
    2. 出发后:在帧上标记十字,录制时间,并刷新帧显示
    """
    global frame_captured
    global key
    video:Video_Stream=self.Video
    # 标记十字
    video.Mark_Cross(frame_captured)
    # 标记时间
    annote_text=None
    # 左下角文字位置
    # 左上角是时间显示,右下角添加缩略图,剩余右上角可以标记
    # 受到屏幕尺寸限制,上边缘的文字实时不可见
    buttom_left_pos=(3,470)
    if(self.Stage_Flag==0):
        # 显示standby字样
        annote_text="Standing By"
        video.Mark_Text(frame_captured,annote_text,buttom_left_pos,1,2)
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
    video:Video_Stream=self.Video
    video.Save_Frame(frame_captured)

def Frame_Save_Callback(self:MissionDef):
    self.Video.Release_VideoWriter()

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
    @参数列表元素数: 3 [[待机时间(s)],[机械臂复位位置,复位时间],[yaw轴转动时间]];
        若待机时间为0,则点击ENTER键结束待机
    """
    wait_time=self.Para_List[0][0]
    global key
    if(self.Stage_Flag==0):
        # 复位机械臂(为了更新关节角和防止动作干涉)
        arm_pos,arm_reset_time=self.Para_List[1]
        busy_flag=arm.Goto_Target_Pos(arm_pos,arm_reset_time)
        if(busy_flag==False):
            self.Change_Stage(1)
            arm.Claw_Cmd(False,False)
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 准备出发...".format(self.Name))
    elif(self.Stage_Flag==1):
        if(wait_time!=0):
            if(time.time()-self.Start_Time>=wait_time):
                self.Change_Stage(2)
        else:
            if((key==kb.ENTER.value)or(buttom.Poll()==True)):
                self.Change_Stage(2)
    elif(self.Stage_Flag==2):
        # 进入扫码姿态
        busy_flag=arm.Run_Preset_Action(arm.ActionGroup.SCAN_QRCODE)
        if(busy_flag==False):
            self.Change_Stage(3)
    elif(self.Stage_Flag==3):
        # yaw轴转至扫码角度
        arm_rotation_time=self.Para_List[2][0]
        busy_flag=arm.Goto_Target_Pos((0,-100,0),arm_rotation_time,arm.Ctrl_Mode.YAW_ROTATION)
        if(busy_flag==False):
            self.Change_Stage(4)
    elif(self.Stage_Flag==4):
        self.End()


def Departure_Func(self:MissionDef_t):
    """
    * 启停区出发
        * 包含横走,制动,直走,直走2(移动扫码)/制动(静止扫码)
    ### 参数列表\n
    * para_list:[[横走速度],[直走速度],[直走速度2]]
        * 若直走速度2==[0,0,0],则采用静止扫码方案
    * time_list:[横走时间,直走时间,制动等待时间]
    """
    # 开始斜走，计时
    if(self.Stage_Flag==0):
        self.Change_Stage()
        agv.Velocity_Control(self.Para_List[0])
        self.Phase_Start_Time=time.time()
        if(self.Verbose_Flag==True):
            self.Output("Mission({})开始横走".format(self.Name))
    # 等待横走完成，开始制动
    elif(self.Stage_Flag==1):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[0]):
                self.Change_Stage()
                agv.Velocity_Control([0,0,0])
                self.Phase_Start_Time=time.time()
                self.Output("Mission({}) 开始制动".format(self.Name))
    # 等待制动完成,开始直走
    elif(self.Stage_Flag==2):
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[2]):
            self.Change_Stage()
            agv.Velocity_Control(self.Para_List[1])
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                self.Output("Mission({})制动完成,开始直走".format(self.Name))
    # 等待直走完成，停止
    if(self.Stage_Flag==3):
        # 若直走时间不为0,采用直走后停止方案，静止扫码
        if((time.time()-self.Phase_Start_Time)>=self.Time_List[1]):
            self.Change_Stage()
            agv.Velocity_Control(self.Para_List[2])
            self.Phase_Start_Time=time.time()
            self.Output("Mission({}) 开始直走/制动".format(self.Name))
    # 等待制动完成，结束
    elif(self.Stage_Flag==4):
        if(self.Para_List[2]==[0,0,0]):
            self.End()
        else:
            if((time.time()-self.Phase_Start_Time)>=self.Time_List[2]):
                self.End()
                self.Output("Mission({}) 制动完成".format(self.Name))


def Scan_QRcode_Func(self:MissionDef):
    """
    @功能：扫码
    @参数: scanning_time, 扫码时间
    @ 参数列表: [[scan_flag,scanning_time]],
    1. scan_flag为True则扫描二维码,否则用延时代替;
    2. scanning_time为扫码时间,扫码超时会引发TimeoutError,若赋值0,则不会触发超时
    """
    global frame_captured
    global rgb_order_list
    # True则扫描二维码,否则用延时代替
    scan_flag=self.Para_List[0][0]
    scanning_time=self.Para_List[0][1]
    if(scan_flag==True):
        if(self.Stage_Flag==0):
            self.Change_Stage(1)
            self.Phase_Start_Time=time.time()
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 开始扫码".format(self.Name))
        elif(self.Stage_Flag==1):
            if(scanning_time!=0):
                if((time.time()-self.Phase_Start_Time)>=scanning_time):
                    raise TimeoutError("Mission({}) 扫码超时".format(self.Name))
            # 检测图像中的内容码,若识别出二维码，则进行内容格式检查，若格式无误，则记录内容
            frame_processed=cv.cvtColor(frame_captured,cv.COLOR_BGR2GRAY)
            code_list:List[Decoded]=pz.decode(frame_processed)
            frame_processed=cv.cvtColor(frame_processed,cv.COLOR_GRAY2BGR)
            frame_processed=self.Video.Make_Thumbnails(frame_processed)
            self.Video.Paste_Img(frame_captured,frame_processed)
            if(len(code_list)>0):
                if(self.Verbose_Flag==True):
                    self.Output("Mission({}) 检测到可识别码".format(self.Name),INFO)
                for code in code_list:
                    c,r,width,height=code.rect
                    cv.rectangle(frame_captured,(c,r),(c+width,r+height),(0,0,255),2)
                    code_type:str=code.type
                    # 检查码类型
                    if(code_type=="QRCODE"):
                        code_data:bytes=code.data
                        data_decoded=code_data.decode()
                        if(self.Verbose_Flag==True):
                            self.Output("Mission({}) code_data: {}"
                                        .format(self.Name,data_decoded),INFO)
                        # 检查格式是否为???+???
                        if(data_decoded[3]=='+'):
                            str_list=data_decoded.split('+')
                            if(len(str_list)==2):
                                error_flag=False
                                num_list=[]
                                # 检查???是否为123的组合
                                for string in str_list:
                                    if((string.isdigit()==False)or(len(string)!=3)):
                                        error_flag=True
                                        break
                                    else:
                                        for char in string:
                                            num=np.uint8(char)
                                            if(num>0 and num<4):
                                                num_list.append(num)
                                            else:
                                                error_flag=True
                                                break
                                if(error_flag==False):
                                    # 检查任务码互斥性
                                    if(num_list[0]!=num_list[1] and num_list[0]!=num_list[2]):
                                        if(num_list[3]!=num_list[4] and num_list[3]!=num_list[5]):
                                            rgb_order_list=[num_list[:3],num_list[3:]]
                                            self.Output("Mission({}) 装载任务码: {}"
                                                        .format(self.Name,rgb_order_list),INFO)
                                            self.Change_Stage(2)
                                            if(show_missionCode==True):
                                                cv.namedWindow(missionCode_window)
                                                RM.Show_MissionCode(missionCode_window,
                                                                    data_decoded)
                # 若检测到内容码，但内容未记录,说明不是二维码或者格式错误,输出警告信息
                if(len(rgb_order_list)==0):
                    self.Output("Mission({}) 内容码类型/格式错误".format(self.Name),WARNING)
        elif(self.Stage_Flag==2):
            self.End()
    else:
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

    stop_wait_time=1

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
    ## 功能:
    位置纠正(原料区、加工区、暂存区)\n
    ## 参数列表:\n
    [\n
    0. [纠正位置]
    1. [动作就位时间/ms:t0,t1...],\n
    2. [(高)纠正参数:adjInterval, stop_th, v_adj, vc_th, detail_params],\n
    3. [低纠正参数:adjInterval, stop_th, v_adj, detail_params],\n
    4. [角度纠正参数:adjInterval,stop_th,omg_adj,angle_compensation,detail_params,y_offset],\n
    * detail_params[tuple] = circle:(minR,maxR) || line_canny:(th_l,th_h)
        * 默认值:(100,200) || (90,180)
    * y_offset为角度纠正时机械臂y轴位置偏移(为了确保场地边缘在视野中),以场外方向为正\n
    5. [是否滤波:filter_flag_xy, filter_flag_angle]\n
    6. [纠正时机械臂y轴补偿:y_compensation_h, y_compensation_l]\n
    ]
    * stop_th[tuple] = thy, thx
    * v_adj[tuple] = v_adj_y, v_adj_x
    """
    global frame_captured
    correction_pos:CP=self.Para_List[0][0]
    if(correction_pos==None):
        # 纠正时间
        correction_time=self.Para_List[1][0]
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
    else:
        # 若在原料区,则检测物料盘;若在加工/暂存区,则检测绿色环
        target_object=None
        lin_flag=False
        grey_flag=False
        if(correction_pos==CP.Material):
            target_object=material_plate
            grey_flag=True
        else:
            target_object=green_ring
            lin_flag=True
        if(self.Stage_Flag==0):
            # 机械臂就位
            busy_flag=True
            action_time=self.Para_List[1][0]
            if(correction_pos==CP.Material):
                x,y,z=material_plate.Get_Material_Pos()
                z+=120
                busy_flag=arm.Goto_Target_Pos((x,y,z),action_time)
            elif(correction_pos==CP.Processing or correction_pos==CP.Storage):
                # 获得角度纠正y轴位置偏移(为了确保场地边缘在视野中)
                y_offset=self.Para_List[4][5]
                x,y,z=green_ring.Get_Processing_Pos()
                y-=y_offset
                busy_flag=arm.Goto_Target_Pos((x,y,z),action_time)
            if(busy_flag==False):
                self.Change_Stage(99)
                if(self.Verbose_Flag==True):
                    correction_name=correction_pos.value[1]
                    self.Output("Mission({}) 开始{}".format(self.Name,correction_name))
                self.Phase_Start_Time=time.time()
        elif(self.Stage_Flag==99):
            if(correction_pos==CP.Material):
                RM.Monitor_andAbandon(frame_captured,target_object,lin_flag,self)
            else:
                self.Change_Stage()
                self.Phase_Start_Time=time.time()
        elif(self.Stage_Flag==100):
            # 若纠正位置为加工/暂存区,则进行角度纠正
            # 若纠正位置为原料区,则检测原料盘圆形物块阴影并判断是否静止
            if(correction_pos==CP.Material):
                vc_th=self.Para_List[2][3]
                RM.Circle_Detect_Stable(self,frame_captured,target_object,vc_th,1,False,True)
                if(time.time()-self.Phase_Start_Time>=20):
                    raise TimeoutError("Mission({}) 原料区纠正超时".format(self.Name))
            else:
                adjInterval,stop_th,omg_adj,angle_compensation,detail_params,_=self.Para_List[4]
                line_list,frame_processed=edge_line.Detect(frame_captured,False,detail_params,
                                                           True)
                # 角度取样(+滤波)
                # 暂无多物体识别能力,只要第一个坐标,其它全部当成噪声放弃
                miss_flag=False
                if(len(line_list)!=0):
                    pt1,pt2=line_list[0]
                    cv.line(frame_captured,pt1,pt2,(0,0,0),2)
                    vector_delta=pt2-pt1
                    angle=np.arctan2(vector_delta[1],vector_delta[0])
                    angle=np.degrees(angle)
                    vector_delta[1]=0
                    pt2=pt1+vector_delta
                    cv.line(frame_captured,pt1,pt2,(0,0,255),2)
                    filter_flag=self.Para_List[5][1]
                    if(filter_flag==True):
                        angle=myfilter.Get_Filtered_Value(angle)
                else:
                    miss_flag=True
                frame_processed=cv.cvtColor(frame_processed,cv.COLOR_GRAY2BGR)
                frame_processed=self.Video.Make_Thumbnails(frame_processed)
                self.Video.Paste_Img(frame_captured,frame_processed)
                # 按照规定时间间隔:判断角度,调整转动角速度
                current_time=time.time()
                if(current_time-self.Phase_Start_Time>=adjInterval):
                    if(miss_flag==True):
                        self.Output("Mission({}) 未检测到场地边缘".format(self.Name),WARNING)
                        agv.Velocity_Control([0,0,0])
                    else:
                        if(len(line_list)>1):
                            self.Output("Mission({}) 发现多个边缘".format(self.Name),WARNING)
                        self.Output("Mission({}) (point,angle)({},{})"
                                    .format(self.Name,pt1,angle))
                        delta=angle+angle_compensation
                        omg=0
                        if(delta>stop_th):
                            omg=-omg_adj
                        elif(delta<-stop_th):
                            omg=omg_adj
                        else:
                            self.Change_Stage(200)
                            # 复位滤波器,准备进行高纠
                            myfilter.Reset()
                            self.Output("Mission({}) 角度纠正完毕".format(self.Name))
                        agv.Velocity_Control([0,0,omg])
                    self.Phase_Start_Time=time.time()
        elif(self.Stage_Flag==101):
            # 原料区专用:超时错误处理
            RM.RawMaterial_ErrorHandler(self,agv,1)
        elif(self.Stage_Flag==200):
            # 加工\暂存区专属,调整机械臂姿态,进行高纠
            action_time=self.Para_List[1][0]
            y_compensation=self.Para_List[6][0]
            x,y,z=green_ring.Get_Processing_Pos()
            y+=y_compensation
            busy_flag=arm.Goto_Target_Pos((x,y,z),action_time)
            if(busy_flag==False):
                self.Change_Stage(1)
                self.Output("Mission({}) 开始高纠".format(self.Name))
        elif(self.Stage_Flag in [1,3]):
            adjInterval=None
            stop_th=None
            v_adj=None
            detail_params=None
            if(self.Stage_Flag==1):
                # 原料区纠正||加工/暂存区高纠正参数
                adjInterval,stop_th,v_adj,_,detail_params=self.Para_List[2]
            else:
                # 加工/暂存区低纠正参数
                adjInterval,stop_th,v_adj,detail_params=self.Para_List[3]
            thy,thx=stop_th
            v_adj_y,v_adj_x=v_adj
            # 若在原料区,则检测物料盘;若在加工/暂存区,则检测绿色环
            center_list,frame_processed=target_object.Detect(frame_captured,lin_flag,
                                                             detail_params,grey_flag)
            frame_processed=cv.cvtColor(frame_processed,cv.COLOR_GRAY2BGR)
            frame_processed=self.Video.Make_Thumbnails(frame_processed)
            self.Video.Paste_Img(frame_captured,frame_processed)
            # 位置取样(+滤波)
            # 暂无多物体识别能力,只要第一个坐标,其它全部当成噪声放弃
            miss_flag=False
            c=None;r=None
            if(len(center_list)!=0):
                center:np.ndarray=center_list[0]
                filter_flag=self.Para_List[5][0]
                c,r=center
                org=self.Video.Frame_Shape_Half
                org=np.array(org[::-1])
                cv.line(frame_captured,center.astype(int),org,(0,0,255),2)
                if(filter_flag==True):
                    center=myfilter.Get_Filtered_Value(center)
            else:
                miss_flag=True
            # 按照规定时间间隔:判断距离,调整速度
            current_time=time.time()
            if(current_time-self.Phase_Start_Time>=adjInterval):
                if(miss_flag==True):
                    self.Output("Mission({}) 未检测到目标对象".format(self.Name),WARNING)
                    agv.Velocity_Control([0,0,0])
                else:
                    if(len(center_list)>1):
                        self.Output("Mission({}) 发现多个目标对象".format(self.Name),WARNING)
                    RM.Correction_xyResp(self,agv,(c,r,thy,thx,v_adj_y,v_adj_x))
                self.Phase_Start_Time=time.time()
        elif(self.Stage_Flag==2):
            if(correction_pos==CP.Material):
                # 复位滤波器,准备后续任务
                myfilter.Reset()
                self.End()
                stuff_index=rgb_order_list[round_counter.Get()][stuff_index_counter.Get()]-1
                self.Output("Mission({}) 开始扫描物块{}"
                            .format(self.Name,stuff_index+1),INFO)
            else:
                # 原料/加工区低纠正就位
                x,y,z=green_ring.Get_Processing_Pos()
                z-=50
                # 试验中物块在机械臂系下的y值会大于纠正位置,纠正时需要补偿
                y_compensation=self.Para_List[6][1]
                y+=y_compensation
                action_time=self.Para_List[1][1]
                busy_flag=arm.Goto_Target_Pos((x,y,z),action_time)
                if(busy_flag==False):
                    # 复位滤波器,准备进行低纠
                    myfilter.Reset()
                    self.Change_Stage()
                    self.Output("Mission({}) 低纠姿态就位".format(self.Name))
                    self.Phase_Start_Time=time.time()
        elif(self.Stage_Flag==4):
            # 等待2秒,评估结果
            assumption_time=0
            if(time.time()-self.Phase_Start_Time>=assumption_time):
                self.Change_Stage()
                self.Output("Mission({}) 评估完成".format(self.Name))
        elif(self.Stage_Flag==5):
            busy_flag=arm.Run_Preset_Action(arm.ActionGroup.HOLD_STUFF)
            if(busy_flag==False):
                # 复位滤波器,准备后续任务
                myfilter.Reset()
                self.End()
                self.Output("Mission({}) 纠正结束,机械臂复位".format(self.Name))


def RawMaterial_Picking_Func(self:MissionDef):
    """
    * 功能: 原料区夹取
    ## 参数列表:\n    
    [\n
    0. [material_height_offset,claw_y_offset,stuff_height_offset],\n
    1. [vc_th],\n
    2. [t0_lookdown,t1_inplace,t2_turn,t3_readyPut,...],\n
    3. [progression_speed,put_speed]\n
    ]\n
    """
    global frame_captured
    global stuff_list
    global stuff_index_counter
    stuff_index=rgb_order_list[round_counter.Get()][stuff_index_counter.Get()]-1
    current_stuff:myObject=stuff_list[stuff_index]
    if(self.Stage_Flag in [4,9]):
        # 获取任务参数0
        material_height_offset=self.Para_List[0][0]
        t0=self.Para_List[2][0]
        # 移动到物块上方
        x,y,z=current_stuff.Get_Material_Pos()
        z+=material_height_offset
        busy_flag=arm.Goto_Target_Pos((x,y,z),t0)
        if(busy_flag==False):
            self.Change_Stage()
            if(self.Verbose_Flag==True):
                self.Output("Mission({}) 机械臂就位,开始扫描物块{}"
                            .format(self.Name,stuff_index+1),INFO)
    elif(self.Stage_Flag in [0,5,10]):
        # 获取任务参数1
        vc_th=self.Para_List[1][0]
        # 检测圆形
        RM.Circle_Detect_Stable(self,frame_captured,current_stuff,vc_th)
    elif(self.Stage_Flag in [1,6,11]):
        RM.Material_FetchStuff(self,arm,current_stuff)
    elif(self.Stage_Flag in [2,7,12]):
        RM.StuffPlate_PutOn(self,arm,current_stuff)
    elif(self.Stage_Flag in [3,8,13]):
        t2=self.Para_List[2][2]
        pos=current_stuff.Get_Material_Pos()
        busy_flag=arm.Goto_Target_Pos(pos,t2,arm.Ctrl_Mode.YAW_ROTATION)
        if(busy_flag==False):
            # 每次将物料放上车后的回转是完成物料夹取的标志,索引自增,进行下一个物料的夹取
            # 最后一次回转后标志所有夹取结束,索引重置
            if(self.Stage_Flag!=13):
                stuff_index_counter.Increment()
            else:
                stuff_index_counter.Reset()
            self.Change_Stage()
            self.Output("Mission({}) 已朝向原料盘".format(self.Name))
    elif(self.Stage_Flag==14):
        # 完成所有夹取任务后,复位物料索引计数器
        stuff_index_counter.Reset()
        self.End()


def RawMaterial_2_Processing_Func(self:MissionDef_t):
    """
    @功能：原料区->加工区
    @参数列表元素数: 5 [[直走速度],[圆弧转弯参数],[直走速度],[圆弧转弯参数],[直走速度]]
    @时间列表元素数: 5 [直走时间,圆弧转弯时间,直走时间,圆弧转弯时间,直走时间]
    @参数: stop_wait_time, 制动等待时间
    """
    stop_wait_time=0.5

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
    * 加工区放置回收
    ### 参数列表:\n
    [\n
    0. [物块相关参数: t_turn_R, t_turn_G, t_turn_B]\n
    1. [车取参数: t_aim, speed_down ,speed_up, height_offset]\n
        * 若t_aim==0,则不进行对准动作,直接到位,到位时间为t_down=speed_down\n
    2. [加放参数: t_aim, speed_down, speed_up, height_offset]\n
        * 若speed_up==0,则不进行中间动作,直接复位至HOLD姿态\n
    3. [加取参数: t_first_turn, t_aim, speed_down]\n
        * t_first_turn是首次回收的转动时间,此时机械臂末端转动前后都在加工区\n
        * height_offset参数与加放共用\n
    4. [车放参数: t_aim, speed_down, height_offset]\n
    5. [加放独立位置补偿(RGB)(round 1): (x_comp, y_comp, z_comp),(...),(...)]\n
    6. [加放独立位置补偿(RGB)(round 2): (x_comp, y_comp, z_comp),(...),(...)]
    ]
    """
    stuff_index=rgb_order_list[round_counter.Get()][stuff_index_counter.Get()]-1
    current_stuff:myObject=stuff_list[stuff_index]
    if(self.Stage_Flag in [0,3,6]):
        self.Change_Stage()
        self.Output("Mission({}) 放置物块{}".format(self.Name,stuff_index+1),INFO)
    elif(self.Stage_Flag in [1,4,7]):
        # 车取
        RM.StuffPlate_Fetch(self,arm,current_stuff,stuff_index)
    elif(self.Stage_Flag in [2,5,8]):
        # 加放
        comp=None
        if(round_counter.Get()==0):
            comp=5
        else:
            comp=6
        cplt_flag=RM.Processing_PutOn(self,arm,current_stuff,stuff_index,False,comp)
        # 每次完成放置后,索引计数器自增或重置
        if(cplt_flag==True):
            if(self.Stage_Flag!=9):
                stuff_index_counter.Increment()
                self.Output("Mission({}) 索引自增".format(self.Name))
            else:
                stuff_index_counter.Reset()
                self.Output("Mission({}) 开始回收".format(self.Name))
    elif(self.Stage_Flag in [9,12,15]):
        self.Change_Stage()
        self.Output("Mission({}) 回收物块{}".format(self.Name,stuff_index+1),INFO)
    elif(self.Stage_Flag in [10,13,16]):
        # 加取
        comp=None
        if(round_counter.Get()==0):
            comp=5
        else:
            comp=6            
        RM.Processing_Fetch(self,arm,current_stuff,stuff_index,comp)
    elif(self.Stage_Flag in [11,14,17]):
        # 车放
        cplt_flag=RM.StuffPlate_PutOn(self,arm,current_stuff,False,stuff_index)
        # 除最后一次,每次完成回收后,索引计数器自增
        if(cplt_flag==True):
            if(self.Stage_Flag!=18):
                stuff_index_counter.Increment()
    elif(self.Stage_Flag==18):
        t_turn=self.Para_List[0][stuff_index]
        busy_flag=arm.Goto_Target_Pos((0,-200,0),t_turn,arm.Ctrl_Mode.YAW_ROTATION)
        if(busy_flag==False):
            self.End()
            # 回收结束后重置索引计数器
            stuff_index_counter.Reset()
            self.Output("Mission({}) 回收完毕,机械臂复位".format(self.Name))


def Three_Section_Turn_Func(self:MissionDef_t):
    """
    @功能: 三段(2直线+1圆弧)转弯,用于加工区->暂存区 或 暂存区->原料区
    @参数列表元素数: 3 [[直走速度],[圆弧转弯参数],[直走速度]]
    @时间列表元素数: 3 [直走时间,圆弧转弯时间,直走时间]
    @参数: stop_wait_time, 制动等待时间
    """
    stop_wait_time=0.5

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


def Storage_Place_Func(self:MissionDef):
    """
    * 暂存区放置 & 暂存区码垛
    ### 参数列表:\n
    [\n
    0. [物块相关参数: t_turn_R, t_turn_G, t_turn_B]\n
    1. [车取参数: t_aim, speed_down ,speed_up, height_offset]\n
        * 若t_aim==0,则不进行对准动作,直接到位,到位时间为t_down=speed_down\n
    2. [加放参数: t_aim, speed_down, speed_up, height_offset]\n
        * 若speed_up==0,则不进行中间动作,直接复位至HOLD姿态\n
    3. [码垛标志: stacking_flag]\n
    4. [加方独立位置补偿(RGB): (x_comp, y_comp, z_comp),(...),(...)]
    ]
    """
    stuff_index=rgb_order_list[round_counter.Get()][stuff_index_counter.Get()]-1
    current_stuff:myObject=stuff_list[stuff_index]
    if(self.Stage_Flag in [0,3,6]):
        self.Change_Stage()
        self.Output("Mission({}) 放置物块{}".format(self.Name,stuff_index+1))
    elif(self.Stage_Flag in [1,4,7]):
        # 车取
        RM.StuffPlate_Fetch(self,arm,current_stuff,stuff_index)
    elif(self.Stage_Flag in [2,5,8]):
        # 加放
        stacking_flag=self.Para_List[3][0]
        cplt_flag=RM.Processing_PutOn(self,arm,current_stuff,stuff_index,stacking_flag,4)
        # 每次完成放置后,索引计数器自增或重置
        if(cplt_flag==True):
            if(self.Stage_Flag!=9):
                stuff_index_counter.Increment()
            else:
                stuff_index_counter.Reset()
    elif(self.Stage_Flag==9):
        t_turn=self.Para_List[0][stuff_index]
        busy_flag=arm.Goto_Target_Pos((0,-200,0),t_turn,arm.Ctrl_Mode.YAW_ROTATION)
        if(busy_flag==False):
            self.End()
            # 回收结束后重置索引计数器
            stuff_index_counter.Reset()
            # 暂存区放置完成后第一轮结束,进入第二轮
            # 注意此处第二轮暂存区码垛结束后也会自增,可能有bug
            round_counter.Increment()
            self.Output("Mission({}) 回收完毕,机械臂复位".format(self.Name))


# 第二轮专属
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
    6. 点击'enter'键保存当前帧为图片
    @ 参数列表: 
    1. 初始高、低阈值数组列表[b,g,r];
    2. [阈值状态位(true:高阈值 false:低阈值), 精确度状态位(true:粗调 false:精调)]
    """
    
    # 对局部变量赋值列表中的元素似乎不能共享内存
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

    global frame_captured
    global key

    # 按键读取+触发操作
    if(key==kb.SHIFT.value):
        #点击该键,stage_flag++,在[1,4]区间内切换
        current_stage=self.Stage_Flag
        if(current_stage==4):
            current_stage=1
        else:
            current_stage+=1
        self.Change_Stage(current_stage)
    elif(key==kb.LEFT_ARROW.value):
        self.Output("Mission({}) 调整低阈值".format(self.Name),DEBUG)
        if(th_HighOrLow==True):
            th_HighOrLow=False
            self.Para_List[3][0]=False
    elif(key==kb.RIGHT_ARROW.value):
        self.Output("Mission({}) 调整高阈值".format(self.Name),DEBUG)
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
                        .format(self.Name,channal_th[0],channal_th[1]),DEBUG)
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
                        .format(self.Name,channal_th[0],channal_th[1]),DEBUG)
    elif(key==kb.ESC.value):
        # 结束测试
        self.Change_Stage(5)
    elif(key==kb.TAB.value):
        if(th_CoarseOrPrecise==True):
            th_CoarseOrPrecise=False
            self.Para_List[3][1]=False
            self.Output("Mission({}) 精调模式".format(self.Name),DEBUG)
        else:
            th_CoarseOrPrecise=True
            self.Para_List[3][1]=True
            self.Output("Mission({}) 粗调模式".format(self.Name),DEBUG)
    elif(key==kb.ENTER.value):
        img_code=self.Para_List[4][0]
        self.Para_List[4][0]+=1
        img_name="frame_captured_{}.png".format(img_code)
        cv.imwrite("proj/assets/images/{}".format(img_name),frame_captured)
        self.Output("Mission({}) 已保存当前帧为图片{}".format(self.Name,img_name),INFO)

    # 图像显示状态机
    video=self.Video
    th_3channals=np.zeros((2,3),dtype=np.uint8)
    for i in range(2):
        for j in range(3):
            # 阈值格式变换(3*2->2*3)
            th_3channals[i][j]=self.Para_List[j][i]
    if(self.Stage_Flag==0):
        self.Change_Stage(1)
        channal_th_0:List=self.Para_List[0]
        self.Output("Mission({}) B通道,当前阈值:({},{})"
                        .format(self.Name,channal_th_0[0],channal_th_0[1]),INFO)
    elif(self.Stage_Flag<5):
        if(self.Stage_Flag<4):
            # 根据三通道阈值对帧图像进行二值化
            frame_processed=cv.inRange(frame_captured,th_3channals[0],th_3channals[1])
        elif(self.Stage_Flag==4):
            # 以灰度模式显示二值化图像
            frame_processed=cv.cvtColor(frame_captured,cv.COLOR_BGR2GRAY)
        frame_processed=cv.cvtColor(frame_processed,cv.COLOR_GRAY2BGR)
        # 制作frame_processed缩略图,并粘贴到frame_captured中
        frame_processed=video.Make_Thumbnails(frame_processed)
        video.Paste_Img(frame_captured,frame_processed)
        annote_pos=[3,300]
        annote_font=cv.FONT_HERSHEY_SIMPLEX
        annote_font_size=0.8
        annote_font_color=(0,50,0)
        annote_font_thickness=2
        front_mark_list=[' ',' ',' ',' ']
        front_mark_list[self.Stage_Flag-1]='*'
        annote_text=["{}B:({},{})"
                     .format(front_mark_list[0],self.Para_List[0][0],self.Para_List[0][1]),
                    "{}G:({},{})"
                    .format(front_mark_list[1],self.Para_List[1][0],self.Para_List[1][1]),
                    "{}R:({},{})"
                    .format(front_mark_list[2],self.Para_List[2][0],self.Para_List[2][1]),
                    "{}Greyscale Mode"
                    .format(front_mark_list[3])]
        text_interval=30
        for i in range(4):
            # 阈值显示
                annote_pos[1]+=text_interval
                pos=np.array(annote_pos)
                cv.putText(frame_captured,annote_text[i],pos,annote_font,
                    annote_font_size,annote_font_color,
                    annote_font_thickness)
        if(th_HighOrLow==True):
            cv.putText(frame_captured,"H",(100,300),annote_font,annote_font_size,
                       annote_font_color,annote_font_thickness)
        else:
            cv.putText(frame_captured,"L",(50,300),annote_font,annote_font_size,
                       annote_font_color,annote_font_thickness)
        if(th_CoarseOrPrecise==True):
            cv.putText(frame_captured,"C",(3,300),annote_font,annote_font_size,
                       annote_font_color,annote_font_thickness)
        else:
            cv.putText(frame_captured,"P",(3,300),annote_font,annote_font_size,
                       annote_font_color,annote_font_thickness)
    elif(self.Stage_Flag==5):
       self.Output("Mission({}) 测试结束,阈值:{}".format(self.Name,th_3channals),INFO)
       self.End()