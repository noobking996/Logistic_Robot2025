import cv2 as cv
import numpy as np
import math
from typing import List,Tuple
import logging
from logging import Logger

class Manipulator:
    def __init__(self,arm_params_list:List[Tuple],logger:Logger,
                 type:str="3R_Articulated",name:str="myManipulator"):
        """
        @功能: 创建机械臂对象,存储关节长度、执行器偏移量等参数
        @参数: arm_params_list: 机械臂参数列表, 格式为[(link_lengths),(actuator_offsets)]
        @参数: logger: 日志记录器
        @参数: type: 机械臂类型
        @参数: name: 机械臂名称
        """
        self.Name=name
        self.Type=type
        self.Link_lengths=[]
        self.Actuator_offsets=None
        try:
            link_lengths=arm_params_list[0]
            actuator_offsets=arm_params_list[1]
            for length in link_lengths:
                if length<=0:
                    raise ValueError("Link length should be positive")
                self.Link_lengths.append(length)
            if(len(actuator_offsets)!=3):
                raise ValueError("Actuator offset should be 3 dimensional")
            self.Actuator_offsets=actuator_offsets
            logger.debug("link_lengths:{}; actuator_offsets:{}"
                         .format(link_lengths,actuator_offsets))
        except Exception:
            logger.error("Invalid params for {}({} manipulator)".format(name,type),exc_info=True)
            return None
        
    def INverse_Kinematics_3RAtype(self,target_position:Tuple,yaw_only:False)->Tuple:
        """
        @功能: 计算空间关节式3R机械臂逆运动学解
        @参数: target_position: 目标位置(x,y,z)/mm
        @参数: yaw_only: 是否只计算yaw角度
        @返回: 关节角度列表(theta3:yaw,theta1,theta2)float型
        """
        x0,y0,z0=target_position
        theta3=float(0)
        theta2=float(0)
        theta1=float(0)
        x4,y4,z4=self.Actuator_offsets
        l0,l1,l2=self.Link_lengths

        z1=z4
        x1=math.sqrt(x0**2+y0**2-z1**2)
        theta3=math.atan2(y0,x0)+math.atan2(z1,x1)
        theta3=math.degrees(theta3)

        if(yaw_only==False):
            y1=z0
            xb=x1-x4-l0
            yb=y1-y4
            c2=0.5*(xb**2+yb**2-(l1**2+l2**2))/(l1*l2)
            s2_a=math.sqrt(1-c2**2)
            theta2=math.atan2(s2_a,c2)
            theta2=math.degrees(theta2)
            theta1=math.atan2(yb,xb)-math.atan2(l2*s2_a,l1+l2*c2)
            theta1=math.degrees(theta1)

        return (theta3,theta1,theta2)
            

def test_Manipulator():
    myLogger=Logger("myLogger",logging.DEBUG)
     # 创建终端处理器
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', 
                                  datefmt='%Y-%m-%d %H:%M:%S')
    console_handler.setFormatter(formatter)
    myLogger.addHandler(console_handler)
    arm=Manipulator([(65,130,130),(0,0,0)],myLogger)

    theta3,theta1,theta2=arm.INverse_Kinematics_3RAtype((114,0,51.4),False)
    print(theta3,theta1,theta2)

if __name__=="__main__":
    test_Manipulator()