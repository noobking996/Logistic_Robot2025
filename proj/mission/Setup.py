import os
from typing import List, Callable, Any,Union
import time
import numpy as np
# import sys
# import subprocess

def setup():
    # subprocess.call(['chmod', '+x', '/home/zhang/Logistic_Robot2025/proj/mission/run.sh'])
    # os.system('chmod +x /home/zhang/Logistic_Robot2025/proj/mission/run.sh')

    pass


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
    def End(self):
        self.End_Flag = True
        if(self.Verbose_Flag==True):
            print("Mission({}) End, Duration:{}s"
                  .format(self.Name,(time.time()-self.Start_Time)))

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
                 verbose_flag:bool=False):
        super().__init__("MissionManager",None,para_list,verbose_flag)
        self.Mission_List = mission_list

