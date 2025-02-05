import os
from typing import List, Callable, Any,Union
import time
import numpy as np
from enum import Enum
import logging
from logging import Logger,DEBUG,INFO,WARNING,ERROR,CRITICAL
from cv2 import VideoCapture, VideoWriter
import cv2 as cv
# import sys
# import subprocess

# subprocess.call(['chmod', '+x', '/home/zhang/Logistic_Robot2025/proj/mission/run.sh'])
# os.system('chmod +x /home/zhang/Logistic_Robot2025/proj/mission/run.sh')


def Logger_Setup(mission_code:str="Logistic_Handling",
                 level_list:List[int]=[DEBUG,INFO,DEBUG])->Logger:

    '''
    @功能：初始化公用日志记录器
    @参数:mission_code, 任务代码,决定文件名
    @参数:level_list, 日志级别列表,规定logger,file_handler,console_handler的日志级别
        可选值:[logging.DEBUG,logging.INFO,logging.WARNING,logging.ERROR,logging.CRITICAL]
        参数顺序:[level_logger, level_file_handler, level_console_handler]
        默认值:[logging.DEBUG, logging.DEBUG, logging.INFO]
    @返回值:Logger对象
    '''

    level_logger,level_file_handler,level_console_handler=level_list

    # 创建一个日志记录器
    logger = logging.getLogger('public_logger')
    logger.setLevel(level_logger)

    # 创建文件处理器
    file_handler = logging.FileHandler("proj/assets/logs/{}.log".format(mission_code))
    file_handler.setLevel(level_file_handler)

    # 创建终端处理器
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level_console_handler)

    # 创建格式化器
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', 
                                  datefmt='%Y-%m-%d %H:%M:%S')

    # 设置格式化器
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)

    # 添加处理器
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    return logger


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

        # 日志记录器
        self.Logger = None

        # 触发标志位,用于特定条件下才会触发的任务
        self.Trigger_Flag = False

        # 回调函数,任务结束后调用
        self.Callback_Func=None

        # 发生回调后输出信息
        self.Callback_Output=None

    def Set_Callback(self,callback_func:Callable[...,Any],output:str="Callback Executed"):
        self.Callback_Func=callback_func
        self.Callback_Output=output

    def Set_Logger(self,mission_logger:Logger):
        self.Logger = mission_logger

    def Change_Stage(self,stage:np.uint8):
        self.Stage_Flag = np.uint8(stage)

    def Output(self,output_str:str,output_level:int=DEBUG):
        '''
        @功能: 输出任务信息
        @参数: output_str, 输出的信息
        @参数: output_level, 输出级别,可选值:[DEBUG,INFO,WARNING,ERROR,CRITICAL],默认DEBUG
        @使用建议: 1.建议在DEBUG & INFO级别之间选择,DEBUG用于输出详细信息,INFO用于输出重要信息;
        2. 在任务运行过程中调用该函数输出信息,便于调试和跟踪
        '''
        if(self.Logger!=None):
            self.Logger.log(output_level,output_str)
        else:
            print(output_str)

    def Reset(self):
        """
        @注意事项:复用任务时必须先调用reset()重置该任务标志位,否则会导致任务状态混乱;
        @功能: 重置任务标志位,记录开始时间; 
        """
        self.Change_Stage(0)
        self.End_Flag = False
        self.Start_Time=time.time()
        if(self.Verbose_Flag==True):
            self.Output("Mission({}) Ready".format(self.Name),INFO)

    # 结束任务，在Run()函数中调用，返回True表示任务结束
    def End(self,callback_flag:bool=True)->float:
        """
        @功能: 结束任务
        @参数: callback_flag, 是否执行回调函数,默认为True,
        建议出现错误而强制结束任务时赋值False(需要释放资源的情况除外)
        @返回值: mission_duration, 任务运行时间(包含回调执行时间)
        @使用建议: 在Run_func()函数的最后手动调用end方法以结束任务;
        运行出错时,任务会自动结束,无需手动调用end方法;
        """
        self.End_Flag = True
        if(callback_flag==True):
            if(self.Callback_Func!=None):
                self.Callback_Func(self)
                self.Output("Mission({}) {}".format(self.Name,self.Callback_Output),INFO)
        if(self.Start_Time!=None):
            # 防止任务还未初始化就结束的情况
            mission_duration=time.time()-self.Start_Time
        else:
            mission_duration=0.0
        if(self.Verbose_Flag==True):
            self.Output("Mission({}) End, Duration(s):{}".
                        format(self.Name,mission_duration),INFO)
        return mission_duration

    # 运行任务，放在循环中，外部判断返回值，True则切换到下一任务或结束循环
    def Run(self)->bool:
        '''
        @功能: 运行任务,放在循环中,外部判断返回值,True则切换到下一任务或结束循环;
        False表示任务未结束;None表示任务运行出错
        @返回值: bool, True表示任务结束
        '''
        try:
            self.Run_func(self)
            return self.End_Flag
        # 若捕获异常,结束任务并输出错误信息,返回None,外部接收后应跳转至error_handler
        except Exception as e:
            self.Output("Mission({}) Run Error: {}".format(self.Name,str(e)),ERROR)
            self.End(False)
            return None
        
    def Run_Triggered_By(self,condition)->bool:
        """
        @功能:触发条件首次为True时,自动初始化任务并使能任务运行
        @参数: condition, 触发条件
        @返回值: error_flag, True表示任务运行出错
        """
        error_flag=False
        if(self.Trigger_Flag==False):
            if(condition==True):
                self.Trigger_Flag=True
                self.Reset()
        else:
            if(self.Run()==None):
                error_flag=True
        return error_flag
    

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

        self.Permanent_Mission_LIst=[]
        self.Trigger_Func_List=[]

    def Set_Permanent_Mission(self,mission_list:List[Union[MissionDef,MissionDef_t]]=[],
                                  trigger_func_list:List[Callable[...,Any]]=[],
                                  num_mission:np.uint8=2):
        """
        @方法命名错误:子任务应表述为subtask或mission,添加新特征/方法时注意规范
        @功能：设置常驻子任务及其条件触发函数
        @参数: mission_list, 常驻子任务列表,存储任务定义
        @参数: trigger_func_list,包含:
        1. 触发函数列表,用于判断是否触发常驻子任务(需要实时访问管理器中的任务状态);
        @参数: num_mission, 常驻子任务最大数量,默认为2
        @使用建议: 1.常驻子任务的触发和结束应在任务管理器的Run()函数中实现,以便于任务管理器的统一管理;
        """
        # 常驻子任务列表
        self.Permanent_Mission_LIst=mission_list
        if(len(self.Permanent_Mission_LIst)>num_mission):
            self.Output("Mission({}): More than {} permanent subtasks"
                        .format(self.Name,num_mission),ERROR)
            self.End()
            return
        self.Trigger_Func_List=trigger_func_list
        if(len(self.Trigger_Func_List)>2):
            self.Output("Mission({}): More than 2 trigger functions"
                        .format(self.Name),ERROR)
            self.End()

    # 子任务错误处理
    def Error_Handler(self,mission_code:np.uint8=255):
        error_mission_code=np.uint8(mission_code)
        self.End(False)

    def Run(self):
        # 执行部分常驻任务
        # 通过Trigger_Func返回值判断是否出错,若出错(true),强制跳出循环,结束所有任务
        p_error_flag=False
        if(len(self.Permanent_Mission_LIst)>0):
            p_error_flag=(self.Trigger_Func_List[0])(self)
        if(p_error_flag==True):
            self.Error_Handler()
        if(self.End_Flag==True):
            return True

        # 执行列表任务
        # 若执行完所有列表任务,结束所有任务
        # 通过子任务返回的end_flag检测错误,若出错(none),强制跳出循环,结束所有任务
        mission_code=self.Stage_Flag
        if(self.Submission_Reset_Flag==True):
            # 设置子任务的日志记录器
            if(self.Logger!=None):
                self.Mission_List[mission_code].Set_Logger(self.Logger)
            # 初始化子任务
            self.Submission_Reset_Flag=False
            self.Mission_List[mission_code].Reset()
        else:
            end_flag=self.Mission_List[mission_code].Run()
            if(end_flag==True):
                # 重置子任务的重置标志位
                self.Submission_Reset_Flag=True
                self.Change_Stage(mission_code+1)
            elif(end_flag==None):
                self.Error_Handler(mission_code)
        if(self.Stage_Flag>=self.Num_Mission):
            self.End()
        if(self.End_Flag==True):
            return True

        # 执行另一部分常驻任务
        # # 通过Trigger_Func返回值判断是否出错,若出错(true),不再循环,结束所有任务
        if(len(self.Permanent_Mission_LIst)>1):
            p_error_flag=(self.Trigger_Func_List[1])(self)
        if(p_error_flag==True):
            self.Error_Handler()

        return self.End_Flag
    
    def End(self,callback_flag:bool=True)->float:
        """
        @功能:任务列表执行完毕时: 
        1. 自动结束常驻子任务(目的是获取其运行时间,以及通过callback执行释放资源等操作);
        2. 结束任务管理器运行
        """ 
        for mission in self.Permanent_Mission_LIst:
            # 无论任务管理器是否正常关闭,常驻子任务都正常结束以释放资源
            mission.End()
        return super().End(callback_flag)

