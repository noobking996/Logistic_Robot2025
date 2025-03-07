import numpy as np
import cv2 as cv
import math
import time
from typing import Tuple
from logging import Logger,DEBUG,INFO,WARNING,ERROR,CRITICAL
from mission.Setup import MissionDef
from subsystems.AGV import myAGV
from subsystems.Manipulator import myManipulator
from mission.Setup import myObject

class Module_Stage_Counter:
    def __init__(self):
        self.stage_count = np.uint8(0)
    def Get(self):
        return self.stage_count
    def Increment(self):
        self.stage_count=np.uint8(self.stage_count+1)
    def Reset(self):
        self.stage_count=np.uint8(0)

cnt=Module_Stage_Counter()

def Pos_Compensation(pos:Tuple[float,float,float],self:MissionDef,compensation_index:int=None,
                     stuff_index=None):
    """
    * 独立位置补偿(目前用于加放)
    @param pos: 物块原放置位置
    @param self: 任务实例
    @param compensation_index: 位置补偿列表在任务参数列表中的索引值
    @param stuff_index: 物块在任务参数列表中的索引值
    @returns: 补偿后的位置
    """
    x,y,z=pos
    x_comp,y_comp,z_comp=(0,0,0)
    if(compensation_index!=None and stuff_index!=None):
        x_comp,y_comp,z_comp=self.Para_List[compensation_index][stuff_index]
    else:
        raise Warning("Mission({}) 位置补偿参数位置缺失,补偿为0".format(self.Name))
    x0,y0,z0=x,y,z
    x+=x_comp
    y+=y_comp
    z+=z_comp
    # self.Output("Mission({}) 独立位置补偿:({},{},{})->({},{},{})".format(self.Name,x0,y0,z0,x,y,z))
    return x,y,z

def Show_MissionCode(winname:str,mission_code:str):
    """
    * 显示任务码
    """
    img=np.zeros((400,730),np.uint8)
    img=cv.putText(img,mission_code,(0,250),cv.FONT_ITALIC,5,(255,255,255),10,cv.LINE_AA)
    cv.moveWindow(winname,65,0)
    cv.imshow(winname,img)

def Monitor_andAbandon(frame_captured:np.ndarray,target_object:myObject,lin_flag,self:MissionDef):
    """
    * 持续监视目标,等到其消失后,进入位置纠正阶段
    * 为纠正保留足够的时间
    """
    if(cnt.Get()==0):
        cnt.Increment()
        self.Output("Mission({}) 开始等待".format(self.Name))
    elif(cnt.Get()==1):
        # 额外等待0.5s
        if(time.time()-self.Phase_Start_Time>=0.5):
            cnt.Increment()
            self.Output("Mission({}) 等待结束,开始监视".format(self.Name))
    elif(cnt.Get()==2):
        circie_list,frame_processed=target_object.Detect(frame_captured,lin_flag,
                                                                    None,True)
        frame_processed=cv.cvtColor(frame_processed,cv.COLOR_GRAY2BGR)
        frame_processed=self.Video.Make_Thumbnails(frame_processed)
        self.Video.Paste_Img(frame_captured,frame_processed)
        num_circle=len(circie_list)
        # 如果一开始就发现目标,则持续监视,等到其消失为止才进入原料区纠正
        # 此举是为了为纠正保留足够的时间
        if(num_circle==0):
            self.Change_Stage()
            cnt.Reset()
            self.Output("Mission({}) 目标消失,开始纠正".format(self.Name))
            self.Phase_Start_Time=time.time()

def Circle_Detect_Stable(self:MissionDef,frame_captured:np.ndarray,current_stuff:myObject,
                         vc_th,next_satge:np.uint8=None,lin_flag=True,grey_flag=False):
    """
    * 检测圆形物体,并判断是否静止\n
    * 注意:该函数自带任务状态转换\n
    @param self: Mission实例
    @param frame_captured: 当前帧
    @param current_stuff: 当前物体实例
    @param vc_th: 静止阈值
    @param next_satge: 判断静止后下一任务阶段号码,默认为None
    @param lin_flag: 是否采用林算法二值化,默认为True
    """
    circie_list,frame_processed=current_stuff.Detect(frame_captured,lin_flag,None,grey_flag)
    frame_processed=cv.cvtColor(frame_processed,cv.COLOR_GRAY2BGR)
    frame_processed=self.Video.Make_Thumbnails(frame_processed)
    self.Video.Paste_Img(frame_captured,frame_processed)
    num_circle=len(circie_list)
    if(num_circle!=0):
        if(num_circle>1):
            self.Output("Mission({}) 检测到{}个圆".format(self.Name,num_circle),WARNING)
        circle=circie_list[0]
        c,r=circle
        vc,vr=current_stuff.Velocity
        self.Output("Mission({}),circle_params,{},{},{},{}".format(self.Name,c,r,vc,vr))
        if(vc<vc_th and vc>-vc_th):
            self.Change_Stage(next_satge)
            self.Output("Mission({}) 目标静止,开始行动".format(self.Name),INFO)
    else:
        current_stuff.Clear_Velocity()

def RawMaterial_ErrorHandler(self:MissionDef,agv:myAGV,move_time:float,next_stage=100):
    """
    * 原料区超时错误时(一般第二轮),调用该函数,像x方向走一段距离
    """
    if(cnt.Get()==0):
        cnt.Increment()
        self.Output("Mission({}) 原料区超时,开始强制位置纠正".format(self.Name),WARNING)
        agv.Velocity_Control([200,0,0])
        self.Phase_Start_Time=time.time()
    elif(cnt.Get()==1):
        if(time.time()-self.Phase_Start_Time>=move_time):
            cnt.Increment()
            agv.Velocity_Control([0,0,0])
            self.Output("Mission({}) 强制位置纠正完毕,开始制动".format(self.Name),WARNING)
            self.Phase_Start_Time=time.time()
    elif(cnt.Get()==2):
        # 制动0.3s
        if(time.time()-self.Phase_Start_Time>=0.3):
            cnt.Reset()
            self.Change_Stage(next_stage)
            self.Output("Mission({}) 制动完毕,开始检测圆".format(self.Name),WARNING)
            self.Phase_Start_Time=time.time()

def Correction_xyResp(self:MissionDef,agv:myAGV,adj_params:Tuple)->bool:
    """
    * xy方向分别纠正\n
    * 注意:该函数自带任务状态转换\n
    @param self: Mission实例
    @param agv: AGV实例
    @param adj_params: 纠正参数(c,r,thy,thx,v_adj_y,v_adj_x)
    @returns: 
    complete_flag: 完成标志
    """
    complete_flag=False
    c,r,thy,thx,v_adj_y,v_adj_x=adj_params
     # 场地横向纠正({agv}y)
    if(cnt.Get()==0):
        vy=0
        delta=self.Video.Frame_Shape_Half[1]-c
        if(delta>thy):
            vy=v_adj_y
        elif(delta<-thy):
            vy=-v_adj_y
        else:
            cnt.Increment()
            self.Output("Mission({}) 横向纠正完毕".format(self.Name))
        agv.Velocity_Control([0,vy,0])
        self.Output("Mission({}),delta_y,{},vy,{}".format(self.Name,delta,vy))
    # 场地纵向纠正({agv}x)
    elif(cnt.Get()==1):
        vx=0
        delta=self.Video.Frame_Shape_Half[0]-r
        if(delta>thx):
            vx=v_adj_x
        elif(delta<-thx):
            vx=-v_adj_x
        else:
            cnt.Reset()
            self.Change_Stage()
            complete_flag=True
            self.Output("Mission({}) 纵向纠正完毕".format(self.Name))
        agv.Velocity_Control([vx,0,0])
        self.Output("Mission({}),delta_x,{},vx,{}".format(self.Name,delta,vx))
    return complete_flag


# 回撤目标位置
x_retreat=None;y_retreat=None
# 是否进行计算
calculate_flag=True
def EndActuator_Retreat(arm:myManipulator,current_pos:Tuple,speed:np.uint16,
                        self:MissionDef)->bool:
    """
    * 机械臂末端回撤,防止放置完成后夹爪与物块相撞\n
        * 建议在复用模块中调用,通过返回值判断回撤完成后自增模块计数器
    ## returns
    complete_flag: 完成标志
    """
    global x_retreat,y_retreat,calculate_flag
    complete_flag=False
    x,y,z=current_pos
    if(calculate_flag==True):
        theta3=math.radians(arm.Current_JointAngles[2])
        delta_r=arm.Radial_Offset
        delta_x=delta_r*np.cos(theta3)
        delta_y=delta_r*np.sin(theta3)
        x_retreat=x-delta_x
        y_retreat=y-delta_y
        # 计算完成,反转标志,开始回撤
        calculate_flag=False
        self.Output("Mission({}),retreat_pos,{},{}".format(self.Name,x_retreat,y_retreat))
    else:
        z+=40
        busy_flag=arm.Goto_Target_Pos((x_retreat,y_retreat,z),50,arm.Ctrl_Mode.LINEAR,speed)
        if(busy_flag==False):
            complete_flag=True
            calculate_flag=True
    return complete_flag


def Get_Radial_Offset_Pos(arm:myManipulator,target_pos:Tuple,delta_r:float)->bool:
    """
    * 根据目标位置和给定的径向偏移量,计算偏移后的位置\n
        * 用于加工区夹取,计算就位位置\n
    """
    x,y,z=target_pos
    theta3=math.radians(arm.Current_JointAngles[2])
    delta_x=delta_r*np.cos(theta3)
    delta_y=delta_r*np.sin(theta3)
    x1=x-delta_x
    y1=y-delta_y        
    return x1,y1,z

def Material_FetchStuff(self:MissionDef,arm:myManipulator,current_stuff:myObject,
                        stuff_index=None):
    """
    * 从原料区夹取物料
    """
    if(cnt.Get()==0):
        # 获取任务参数2
        t1=self.Para_List[2][1]
        # 获取任务参数0
        y_offset=self.Para_List[0][1]
        x,y,z=current_stuff.Get_Material_Pos()
        y+=y_offset
        busy_flag=arm.Goto_Target_Pos((x,y,z),t1)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 准备前进夹取".format(self.Name))
    elif(cnt.Get()==1):
        # 获取任务参数3
        progression_speed=self.Para_List[3][0]
        x,y,z=current_stuff.Get_Material_Pos()
        y_offset=self.Para_List[0][1]
        y-=y_offset
        busy_flag=arm.Goto_Target_Pos((x,y,z),50,arm.Ctrl_Mode.LINEAR,progression_speed)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 到达夹取位置".format(self.Name))
    elif(cnt.Get()==2):
        busy_flag=arm.Claw_Cmd(True)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 机械臂夹取".format(self.Name))
    elif(cnt.Get()==3):
        busy_flag=arm.Run_Preset_Action(arm.ActionGroup.HOLD_STUFF)
        if(busy_flag==False):
            cnt.Reset()
            self.Change_Stage()
            self.Output("Mission({}) 提起物块".format(self.Name))

def StuffPlate_PutOn(self:MissionDef,arm:myManipulator,current_stuff:myObject,legacy_flag=True,
                     stuff_index=None)->bool:
    """
    * 将物块放到车载物料盘
    @param legacy_flag: 是否采用老版本原料区夹取的参数设置模式,默认为True
    @param stuff_index: 新参数模式下的物料索引号,用于选取yaw轴转动时间
    """
    complete_flag=False
    t_turn,t_aim,speed_down,height_offset=(None,None,None,None)
    if(legacy_flag==False):
        t_turn=self.Para_List[0][stuff_index]
        t_aim,speed_down,height_offset=self.Para_List[4]
    if(cnt.Get()==0):
        # 获取任务参数2
        if(legacy_flag==True):
            t2=self.Para_List[2][2]
            t_turn=t2
        pos=current_stuff.Get_StuffPlate_Pos()
        busy_flag=arm.Goto_Target_Pos(pos,t_turn,arm.Ctrl_Mode.YAW_ROTATION)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 已朝向物料盘".format(self.Name))
    elif(cnt.Get()==1):
        if(legacy_flag==True):
            # 获取任务参数2
            t3=self.Para_List[2][3]
            t_aim=t3
            # 获取任务参数0
            stuff_height_offset=self.Para_List[0][2]
            height_offset=stuff_height_offset
        x,y,z=current_stuff.Get_StuffPlate_Pos()
        z+=height_offset
        busy_flag=arm.Goto_Target_Pos((x,y,z),t_aim)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 到达物料盘上方".format(self.Name))
    elif(cnt.Get()==2):
        if(legacy_flag==True):
            put_speed=self.Para_List[3][1]
            speed_down=put_speed
        pos=current_stuff.Get_StuffPlate_Pos()
        busy_flag=arm.Goto_Target_Pos(pos,50,arm.Ctrl_Mode.LINEAR,speed_down)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 放入物料盘".format(self.Name))
    elif(cnt.Get()==3):
        busy_flag=arm.Claw_Cmd(False)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 物块已释放".format(self.Name))
    elif(cnt.Get()==4):
        # if(legacy_flag==True):
        #     put_speed=self.Para_List[3][1]
        #     stuff_height_offset=self.Para_List[0][2]
        #     speed_down=put_speed
        #     height_offset=stuff_height_offset
        # x,y,z=current_stuff.Get_StuffPlate_Pos()
        # z+=height_offset
        # busy_flag=arm.Goto_Target_Pos((x,y,z),50,arm.Ctrl_Mode.LINEAR,2*speed_down)
        # if(busy_flag==False):
        #     cnt.Increment()
        #     self.Output("Mission({}) 垂直回升".format(self.Name))
        cnt.Increment()
    elif(cnt.Get()==5):
        busy_flag=arm.Run_Preset_Action(arm.ActionGroup.HOLD_STUFF)
        if(busy_flag==False):
            cnt.Reset()
            complete_flag=True
            self.Change_Stage()
            self.Output("Mission({}) 姿态收缩".format(self.Name))
    return complete_flag


def StuffPlate_Fetch(self:MissionDef,arm:myManipulator,current_stuff:myObject,
                     stuff_index=None):
    """
    * 取出车上的物料
    """
    # 参数获取
    t_aim,speed_down,speed_up,height_offset=self.Para_List[1]
    # 决定是否进行对准动作
    aim_flag=False
    if(t_aim!=0):
        aim_flag=True
    if(cnt.Get()==0):
        pos=current_stuff.Get_StuffPlate_Pos()
        t_turn=self.Para_List[0][stuff_index]
        busy_flag=arm.Goto_Target_Pos(pos,t_turn,arm.Ctrl_Mode.YAW_ROTATION)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 已朝向物料盘".format(self.Name))
    elif(cnt.Get()==1):
        # 计算就位位置,并存储
        pos=current_stuff.Get_StuffPlate_Pos()
        pos=Get_Radial_Offset_Pos(arm,pos,40)
        arm.Store_Intermediate_Point(pos)
        cnt.Increment()
    elif(cnt.Get()==2):
        x,y,z=arm.Get_Intermediat_Point()
        z+=height_offset
        if(aim_flag==True):
            busy_flag=arm.Goto_Target_Pos((x,y,z),t_aim)
            if(busy_flag==False):
                cnt.Increment()
                self.Output("Mission({}) 已就位".format(self.Name))
        else:
            cnt.Increment()
    elif(cnt.Get()==3):
        pos=current_stuff.Get_StuffPlate_Pos()
        busy_flag=None
        if(aim_flag==True):
            busy_flag=arm.Goto_Target_Pos(pos,50,arm.Ctrl_Mode.LINEAR,speed_down)
        else:
            t_down=speed_down
            busy_flag=arm.Goto_Target_Pos(pos,t_down)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 已到达物料位置".format(self.Name))
    elif(cnt.Get()==4):
        busy_flag=arm.Claw_Cmd(True)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 机械臂夹取".format(self.Name))
    elif(cnt.Get()==5):
        x,y,z=current_stuff.Get_StuffPlate_Pos()
        z+=height_offset
        busy_flag=arm.Goto_Target_Pos((x,y,z),50,arm.Ctrl_Mode.LINEAR,speed_up)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 物料已提起".format(self.Name))
    elif(cnt.Get()==6):
        busy_flag=arm.Run_Preset_Action(arm.ActionGroup.HOLD_STUFF)
        if(busy_flag==False):
            cnt.Reset()
            self.Change_Stage()
            self.Output("Mission({}) 姿态收缩,夹取完毕".format(self.Name))


def Processing_PutOn(self:MissionDef,arm:myManipulator,current_stuff:myObject,
                     stuff_index=None,stacking_flag=False,compensation_index=None)->bool:
    """
    * 将物块放到加工区/暂存区
    @param self: MissionDef实例\n
    @param arm: 机械臂实例\n
    @param current_stuff: 要放置的物块\n
    @param stuff_index: 物料索引号,用于选取yaw轴转动时间和独立位置补偿\n
    @param stucking_flag: 是否码垛,默认为False\n
    @param compensation_index: 位置补偿列表在任务参数列表中的索引值\n
    ## Returns
    complete_flag: 完成放置标志
    """
    complete_flag=False
    # 获取任务参数
    t_aim,speed_down,speed_up,height_offset=self.Para_List[2]
    pos=current_stuff.Get_Processing_Pos()
    pos=Pos_Compensation(pos,self,compensation_index,stuff_index)
    up_flag=False
    if(speed_up!=0):
        up_flag=True
    if(cnt.Get()==0):
        turn_time=self.Para_List[0][stuff_index]
        busy_flag=arm.Goto_Target_Pos(pos,turn_time,arm.Ctrl_Mode.YAW_ROTATION)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 已朝向加工区圆环".format(self.Name))
    elif(cnt.Get()==1):
        x,y,z=pos
        if(stacking_flag==True):
            z+=current_stuff.Height
        z+=height_offset
        busy_flag=arm.Goto_Target_Pos((x,y,z),t_aim)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 已对准圆环".format(self.Name))
    elif(cnt.Get()==2):
        x,y,z=pos
        if(stacking_flag==True):
            z+=current_stuff.Height
        busy_flag=arm.Goto_Target_Pos((x,y,z),50,arm.Ctrl_Mode.LINEAR,speed_down)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 物块到达地面".format(self.Name))
    elif(cnt.Get()==3):
        busy_flag=arm.Claw_Cmd(False)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 物块已释放".format(self.Name))
    elif(cnt.Get()==4):
        if(up_flag==True):
            retreat_cpltFlag=EndActuator_Retreat(arm,pos,speed_up,self)
            if(retreat_cpltFlag==True):
                cnt.Increment()
                self.Output("Mission({}) 夹爪已回撤".format(self.Name))
        else:
            cnt.Increment()
    elif(cnt.Get()==5):
        busy_flag=arm.Run_Preset_Action(arm.ActionGroup.HOLD_STUFF)
        if(busy_flag==False):
            cnt.Reset()
            complete_flag=True
            self.Change_Stage()
            self.Output("Mission({}) 姿态收缩,放置完毕".format(self.Name))
    return complete_flag


def Processing_Fetch(self:MissionDef,arm:myManipulator,current_stuff:myObject,
                     stuff_index=None,compensation_index=None):
    """
    * 从加工区夹取物块
    @param self: MissionDef实例\n
    @param arm: 机械臂实例\n
    @param current_stuff: 要取回的物块\n
    @param stuff_index: 物料索引号,用于选取yaw轴转动时间和独立位置补偿\n
    @param compensation_index: 位置补偿列表在任务参数列表中的索引值
    """
    # 获取任务参数
    t_first_turn,t_aim,speed_down=self.Para_List[3]
    pos=current_stuff.Get_Processing_Pos()
    pos=Pos_Compensation(pos,self,compensation_index,stuff_index)
    # 与加工区放置共用一个height_offset
    height_offset=self.Para_List[2][3]
    if(cnt.Get()==0):
        t_turn=None
        if(self.Stage_Flag==10):
            t_turn=t_first_turn
        else:
            t_turn=self.Para_List[0][stuff_index]
        busy_flag=arm.Goto_Target_Pos(pos,t_turn,arm.Ctrl_Mode.YAW_ROTATION)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 已朝向加工区物料".format(self.Name))
    elif(cnt.Get()==1):
        # 计算径向偏移后的就位位置坐标,并存储
        pos_offset=Get_Radial_Offset_Pos(arm,pos,60)
        arm.Store_Intermediate_Point(pos_offset)
        cnt.Increment()
    elif(cnt.Get()==2):
        x,y,z=arm.Get_Intermediat_Point()
        # z+=height_offset
        busy_flag=arm.Goto_Target_Pos((x,y,z),t_aim)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 已对准物料".format(self.Name))
    elif(cnt.Get()==3):
        busy_flag=arm.Goto_Target_Pos(pos,50,arm.Ctrl_Mode.LINEAR,speed_down)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 已到达夹取位置".format(self.Name))
    elif(cnt.Get()==4):
        busy_flag=arm.Claw_Cmd(True)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 机械臂夹取".format(self.Name))
    elif(cnt.Get()==5):
        busy_flag=arm.Run_Preset_Action(arm.ActionGroup.HOLD_STUFF)
        if(busy_flag==False):
            cnt.Reset()
            self.Change_Stage()
            self.Output("Mission({}) 姿态收缩,夹取完毕".format(self.Name))
