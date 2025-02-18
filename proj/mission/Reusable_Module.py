import numpy as np
from typing import Tuple
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

def Correction_xyResp(self:MissionDef,agv:myAGV,adj_params:Tuple):
    """
    * xy方向分别纠正\n
    * 注意:该函数自带任务状态转换\n
    @param self: Mission实例
    @param agv: AGV实例
    @param adj_params: 纠正参数(c,r,thy,thx,v_adj_y,v_adj_x)
    """
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
            self.Output("Mission({}) 纵向纠正完毕".format(self.Name))
        agv.Velocity_Control([vx,0,0])


def Material_FetchStuff(self:MissionDef,arm:myManipulator,current_stuff:myObject):
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

def StuffPlate_PutOn(self:MissionDef,arm:myManipulator,current_stuff:myObject):
    if(cnt.Get()==0):
        # 获取任务参数2
        t2=self.Para_List[2][2]
        pos=current_stuff.Get_StuffPlate_Pos()
        busy_flag=arm.Goto_Target_Pos(pos,t2,arm.Ctrl_Mode.YAW_ROTATION)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 已朝向物料盘".format(self.Name))
    elif(cnt.Get()==1):
        # 获取任务参数2
        t3=self.Para_List[2][3]
        # 获取任务参数0
        stuff_height_offset=self.Para_List[0][2]
        x,y,z=current_stuff.Get_StuffPlate_Pos()
        z+=stuff_height_offset
        busy_flag=arm.Goto_Target_Pos((x,y,z),t3)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 到达物料盘上方".format(self.Name))
    elif(cnt.Get()==2):
        put_speed=self.Para_List[3][1]
        pos=current_stuff.Get_StuffPlate_Pos()
        busy_flag=arm.Goto_Target_Pos(pos,50,arm.Ctrl_Mode.LINEAR,put_speed)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 放入物料盘".format(self.Name))
    elif(cnt.Get()==3):
        busy_flag=arm.Claw_Cmd(False)
        if(busy_flag==False):
            cnt.Increment()
            self.Output("Mission({}) 物块已释放".format(self.Name))
    elif(cnt.Get()==4):
        # 可以取消垂直回升阶段,直接收缩
        # put_speed=self.Para_List[3][1]
        # t3=self.Para_List[2][3]
        # stuff_height_offset=self.Para_List[0][2]
        # x,y,z=current_stuff.Get_StuffPlate_Pos()
        # z+=stuff_height_offset
        # busy_flag=arm.Goto_Target_Pos((x,y,z),50,arm.Ctrl_Mode.LINEAR,put_speed)
        # if(busy_flag==False):
        #     cnt.Increment()
        #     self.Output("Mission({}) 垂直回升".format(self.Name))
        cnt.Increment()
    elif(cnt.Get()==5):
        busy_flag=arm.Run_Preset_Action(arm.ActionGroup.HOLD_STUFF)
        if(busy_flag==False):
            cnt.Reset()
            self.Change_Stage()
            self.Output("Mission({}) 姿态收缩".format(self.Name))