import os
from typing import List, Callable, Any,Union
import time
import numpy as np
from enum import Enum
# import sys
# import subprocess

def setup():
    # subprocess.call(['chmod', '+x', '/home/zhang/Logistic_Robot2025/proj/mission/run.sh'])
    # os.system('chmod +x /home/zhang/Logistic_Robot2025/proj/mission/run.sh')

    pass


class Correction_PosDef(Enum):
    # 修正位置定义: 原料区、加工区、暂存区
    Material=0x00
    Processing=0x01
    Storage=0x02


# 非时变任务类,用于纠正、夹取放置等任务的定义
class MissionDef():
    def __init__(self,name:str,
                 run_func:Callable[...,Any],
                 para_list:List[Any]=[[]],
                 verbose_flag:bool=False):
        
        # 任务名称
        self.Name = name

        # 运行函数定义
        self.Run_func = run_func

        # 任务阶段标志
        self.Stage_Flag = np.uint8(0)

        # 参数列表，在个别任务中索引不一定与stage同步,注意甄别
        self.Para_List = para_list

        # 任务结束标志
        self.End_Flag = False

        # 任务开始时间
        self.Start_Time = None

        # 任务阶段开始时间
        self.Phase_Start_Time = None

        # 运行过程中是否输出详细信息
        self.Verbose_Flag = verbose_flag

    def Change_Stage(self,stage:np.uint8):
        self.Stage_Flag = np.uint8(stage)

    # 复用任务时必须先调用reset()重置该任务标志位
    def Reset(self):
        self.Change_Stage(0)
        self.End_Flag = False
        self.Start_Time=time.time()
        if(self.Verbose_Flag==True):
            print("Mission({}) Reset".format(self.Name))

    # 结束任务，在Run()函数中调用，返回True表示任务结束
    def End(self)->float:
        self.End_Flag = True
        mission_duration=time.time()-self.Start_Time
        if(self.Verbose_Flag==True):
            print("Mission({}) End, Duration:{}s"
                  .format(self.Name,mission_duration))
        return mission_duration

    # 运行任务，放在循环中，外部判断返回值，True则切换到下一任务或结束循环
    def Run(self)->bool:
        self.Run_func(self)
        return self.End_Flag
    

# 时变任务类，用于行走等任务的定义
class MissionDef_t(MissionDef):
    def __init__(self,name:str,
                 run_func:Callable[...,Any],
                 para_list:List[Any]=[[]],
                 time_list:List[float]=[],
                 verbose_flag:bool=False):
        super().__init__(name,run_func,para_list,verbose_flag)
        self.Time_List = time_list


# 任务管理，用于管理多个任务调用顺序和切换
class MissionManager(MissionDef):
    def __init__(self, mission_list:List[Union[MissionDef,MissionDef_t]],
                 para_list:List[Any]=[[]],
                 verbose_flag:bool=False,num_mission:np.uint8=0):
        """
        @功能：任务管理器，用于管理多个任务调用顺序和切换
        @参数: para_list, 任务参数列表,存储全局参数
        @参数: mission_list, 任务列表,存储任务定义
        @参数: verbose_flag, 是否输出详细信息
        @参数: num_mission, 执行到的任务数量(从1开始),若不指定或赋0则表示自动获取任务总量
        """
        super().__init__("MissionManager",None,para_list,verbose_flag)
        self.Mission_List = mission_list
        if(num_mission==0):
            self.Num_Mission = len(mission_list)
        else:
            self.Num_Mission = num_mission

        # 使能子任务重置
        self.Submission_Reset_Flag = True

    def Run(self):
        mission_code=self.Stage_Flag
        if(self.Submission_Reset_Flag==True):
            self.Submission_Reset_Flag=False
            self.Mission_List[mission_code].Reset()
        else:
            end_flag=self.Mission_List[mission_code].Run()
            if(end_flag==True):
                # 重置子任务的重置标志位
                self.Submission_Reset_Flag=True
                self.Change_Stage(mission_code+1)

        if(self.Stage_Flag>=self.Num_Mission):
            self.End()
        return self.End_Flag

