import numpy as np
from typing import Tuple
from mission.Setup import MissionDef
from subsystems.AGV import myAGV
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