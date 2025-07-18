import cv2 as cv
import numpy as np
import os
import Generic_Mission as GM
import time
from math import sin,cos,radians
from logging import Logger,DEBUG,INFO,WARNING,ERROR,CRITICAL

from subsystems.AGV import MOVJ_Drection
from subsystems.Manipulator import myManipulator
from subsystems.Computer_Vision import Video_Setup
from mission import Setup
from mission.Setup import MissionManager, MissionDef, MissionDef_t
from mission.Setup import myObject
from mission.Setup import Correction_PosDef as CP
from mission import Mission_Function as MF

#####################################################################################

Standby=MissionDef("待机",MF.Standby_Func,[[0],[(190,0,5),500],[300]],True)

Departure=MissionDef_t("启停区出发",MF.Departure_Func,[[-100,0,0],[0,400,0],[0,50,0]],
                       [1.6,1.2,0.1],True)

Scan_QRcode=MissionDef("扫码",MF.Scan_QRcode_Func,[[True,0]],True)

QRcode_2_RawMaterial=MissionDef_t("扫码->原料",MF.QRcode_2_RawMaterial_Func,[[0,700,0]],[1.16],True)

RawMaterial_Pos_Correction=MissionDef("原料区纠正",MF.Pos_Correction_Func,
                                      [[CP.Material],[100],[0.1,(20,20),(20,20),5,None],
                                       None,None,[False],None,[(False,False),(0,3),0.5]],True)

RawMaterial_Picking=MissionDef("原料区夹取",MF.RawMaterial_Picking_Func,
                               [[120,40,40],[5],[100,100,300,150],[400,250]],True)

RawMaterial_2_Processing=MissionDef_t("原料区->加工区(1)",MF.RawMaterial_2_Processing_Func,
                                      [[0,500,0],[MOVJ_Drection.Left_Forward,210,84],[0,500,0],
                                       [MOVJ_Drection.Left_Forward,200,84],[0,500,0]],
                                       [0.5,1.02,2.55,1.05,1.1],True)

RawMaterial_2_Processing_Round2=MissionDef_t("原料区->加工区(2)",MF.RawMaterial_2_Processing_Func,
                                      [[0,500,0],[MOVJ_Drection.Left_Forward,210,84],[0,500,0],
                                       [MOVJ_Drection.Left_Forward,200,84],[0,500,0]],
                                       [0.5,0.99,2.55,1.05,1.1],True)

RawMaterial_2_Processing_Stable=MissionDef_t("原料区->加工区(稳定)",MF.RawMaterial_2_Processing_Stable_Func,
                                             [[0,320,0],[0,0,890],[0,500,0],
                                              [MOVJ_Drection.Left_Forward,200,84],[0,500,0]],
                                              [1.5,2.5,3.2,1.05,0.98],True)

RawMaterial_2_Processing_Stable_Round2=MissionDef_t("原料区->加工区(稳定)2",MF.RawMaterial_2_Processing_Stable_Func,
                                             [[0,320,0],[0,0,885],[0,500,0],
                                              [MOVJ_Drection.Left_Forward,200,84],[0,500,0]],
                                              [1.5,2.5,3.2,1.05,0.98],True)

Processing_Pos_Correction=MissionDef("加工区纠正",MF.Pos_Correction_Func,
                                      [[CP.Processing],[200,200],[0.1,(10,10),(20,20),0,(125,140)],
                                       [0.1,(2,2),(5,5),(160,200)],[0.1,0.7,1,1.5,None,-80],
                                       [True,True],[30,18],[False,(-179,-177),0.5]],True)

Processing_PickAndPlace=MissionDef("加工区放置回收",MF.Processing_PickAndPlace_Func,
                                   [[200,200,200],[150,250,300,50],[200,150,350,60],
                                    [150,200,200],[150,200,40],[(0,0,0),(0,0,0),(0,0,0)],
                                    [(0,0,0),(0,0,0),(0,0,0)]],True)

Processing_2_Storage=MissionDef_t("加工区->暂存区",MF.Three_Section_Turn_Func,
                                  [[0,-500,0],[MOVJ_Drection.Left_Backward,200,84],[0,-500,0]],
                                  [1.6,1.05,0.9],True)

Storage_Pos_Correction=MissionDef("暂存区纠正",MF.Pos_Correction_Func,
                                   [[CP.Processing],[200,200],[0.1,(10,10),(20,20),0,(125,140)],
                                       [0.1,(2,2),(5,5),(160,200)],[0.1,0.7,1,1.5,None,-80],
                                       [True,True],[30,18],[False,(91,93),0.5]],True)

# 第一轮专属任务
Storage_Place=MissionDef("暂存区放置",MF.Storage_Place_Func,[[200,200,200],[150,250,300,50],
                                                        [200,150,350,60],[False],
                                                        [(0,0,0),(0,0,0),(0,0,0)]],True)

Storage_2_RawMaterial=MissionDef_t("暂存区->原料区",MF.Three_Section_Turn_Func,
                                    [[0,-400,0],[MOVJ_Drection.Left_Backward,100,80],[0,-400,0]],
                                    [2.2,1.07,0.5],True)

# 第二轮专属任务
Storage_Stacking=MissionDef("暂存区码垛",MF.Storage_Place_Func,[[200,200,200],[150,250,300,50],
                                                           [200,100,0,40],[True],
                                                           [(0,0,0),(0,0,0),(0,0,0)]],True)

Storage_Go_Home=MissionDef_t("暂存区->启停区",MF.Storage_Go_Home_Func,
                             [[0,-400,0],[MOVJ_Drection.Left_Backward,100,79],
                              [0,-400,0],[200,-200,0]],
                             [2.2,1.07,3.7,0.85],True)

Rawmaterial_Go_Home=MissionDef_t("原料区->启停区",MF.Material_Go_Home_Func,
                                 [[5],[0,-400,0],[200,-200,0]],[0.7,3.2,0.8],True)
MF.Home_Pos_Compensation=[20,-10,0]
Rawmaterial_Go_Home.Set_Callback(MF.Home_Callback,"Home Pos Compensation")

Home_Pos_Correction=MissionDef("启停区位置纠正",MF.Home_Pos_Correction_Func,None,True)

# 物流搬运任务管理,共22个子任务
# 参数列表内容:1. 常驻任务触发条件;
                                    # 第一轮 13
Logistics_Handling=MissionManager([Standby,Departure,Scan_QRcode,QRcode_2_RawMaterial,
                                   RawMaterial_Pos_Correction,RawMaterial_Picking,
                                   RawMaterial_2_Processing_Stable,Processing_Pos_Correction,
                                   Processing_PickAndPlace,Processing_2_Storage,
                                   Storage_Pos_Correction,Storage_Place,Storage_2_RawMaterial,
                                    # 第二轮 11
                                   RawMaterial_Pos_Correction,RawMaterial_Picking,
                                   RawMaterial_2_Processing_Stable_Round2,Processing_Pos_Correction,
                                   Processing_PickAndPlace,Processing_2_Storage,
                                   Storage_Pos_Correction,Storage_Stacking,Storage_2_RawMaterial,
                                   Rawmaterial_Go_Home,Home_Pos_Correction],[[0,0,1]],True,0)


# 二值化调参任务定义
# 参数列表内容: [b_th],[g_th],[r_th],[th_HighOrLow,th_CoarseOrPrecise],[图片编号]
Thresholding_Test=MissionDef("二值化调参",MF.Thresholding_Test_Func,
                             [[205,255],[200,255],[195,255],[True,True],[0]],True)

# 测试模式下的任务码预设值
MF.rgb_order_list=[[1,2,3],[2,3,1]]

# 测试任务管理器(视觉相关调试,只能在本地终端启动)
# 参数列表内容:1. 常驻任务触发条件(这里可将录像开启条件设为100,即一直不开启);
Partial_MIssion_Test=MissionManager([Rawmaterial_Go_Home],[[0,0,0]],True,0)

#####################################################################################

# 任务代号
Mission_Code="debug_0716_1611"

# 创建公共日志记录器
Public_Logger=Setup.Logger_Setup(Mission_Code,[DEBUG,DEBUG,DEBUG])

#####################################################################################

# 是否显示任务码
MF.show_missionCode=False

# 是否在完整任务流中跳过夹取/放置(行走调试用)
MF.Skip_All_PickPlace(False)

# 是否使用陀螺仪全局代替视觉方案进行角度纠正
MF.use_gyro_flag=True

#####################################################################################

# 初始化视频流
myVideo=Video_Setup(Mission_Code,Public_Logger)

# 视频帧捕获任务定义
Frame_Capture=MissionDef("视频帧捕获",MF.Frame_Capture_Func,None,True)
Frame_Capture.Set_Logger(Public_Logger)
Frame_Capture.Set_VideoStream(myVideo)
Frame_Capture.Set_Callback(MF.Frame_Capture_Callback,"Cap Released")

# 视频帧标记+显示任务定义(内含键盘按键状态读取)
Frame_Mark_Display=MissionDef("视频帧标记+显示",MF.Frame_Mark_Display_Func,None,True)
Frame_Mark_Display.Set_Logger(Public_Logger)
Frame_Mark_Display.Set_VideoStream(myVideo)
Frame_Mark_Display.Set_Callback(MF.Frame_Mark_Display_Callback,"Windows Closed")

# 视频帧保存任务定义
Frame_Save=MissionDef("视频帧保存",MF.Frame_Save_Func,None,True)
Frame_Save.Set_Logger(Public_Logger)
Frame_Save.Set_VideoStream(myVideo)
Frame_Save.Set_Callback(MF.Frame_Save_Callback,"VideoWriter Released")

#####################################################################################

# 初始化机械臂对象
yaw_compensation=-5             # 全局yaw轴误差补偿<舵机微分值>
MF.RM.ProcessPutFetch_Yaw_Comp=2  # 加工区放置/回收专用的yaw轴误差补偿<deg>(在全局补偿的基础上)
MF.RM.StuffPlatePutFetch_Yaw_Comp=2
MF.arm=myManipulator([(65,130,130),(71,-20-1.12,0)],Public_Logger,MF.myServo)
MF.arm.Set_Joint_to_Actuator_Matrix([[[90,430],[90-16.8,500]],
                                    [[(180-90),420],[180-(90+19.2),500]],
                                    [[0,750+yaw_compensation],[60,1000+yaw_compensation]]])
MF.arm.Set_YawAccRatio(0.2,0.25)
MF.arm.Set_Claw_Angles((800,980))
MF.arm.Set_Radial_Offset(50)

stuff_radius_range=None

# 初始化物块对象
arm_height=148.33               #机械臂坐标系原点距离地面高度

"""场地相关参数"""
material_plate_height=80        # 原料盘高度,原料区夹取用

"""物块相关参数"""
stuff_height=70                 # 物块高度,码垛用
stuff_claw_height=55+3            # 夹持时夹爪距离物料底部的距离
stuff_radius_range=None      # 物块半径范围,原料区纠正/夹取用
MF.RM.Stuff_Disapear_Time=0   # 认为物块消失的时间,原料区放弃第一个用(适用于圆不易检测到的情况)

# 原料区夹取位置
public_material_pos=(0,-260,material_plate_height+stuff_claw_height-arm_height)
# 加工区放置距离(abs(y))
public_processing_distance=280
# 物料盘位置误差补偿
x4_conpensation=18
plate_angle=radians(25)
x_delta=x4_conpensation*cos(plate_angle)
y_delta=x4_conpensation*sin(plate_angle)
y_delta_minus=-y_delta
blue_stuff=myObject("circle",myVideo,[(200,20,20),(255,180,190)],
                   [(176.46-x_delta,82.28-y_delta,-61+stuff_claw_height),
                    public_material_pos,
                    (150+7,-public_processing_distance-6,stuff_claw_height-arm_height)],
                    stuff_radius_range)
blue_stuff.Set_Mixing_Portion((0,-3,3))
blue_stuff.Set_Height(stuff_height)
green_stuff=myObject("circle",myVideo,[(100,210,40),(250,255,180)],
                    [(194.7-x4_conpensation+5,0,-61+stuff_claw_height),
                     public_material_pos,
                     (0,-public_processing_distance-3,stuff_claw_height-arm_height)],
                    stuff_radius_range)
green_stuff.Set_Mixing_Portion((-1,2,-1))
green_stuff.Set_Height(stuff_height)
red_stuff=myObject("circle",myVideo,[(70,60,180),(255,180,255)],
                  [(176.46-x_delta,-82.28-y_delta_minus,-61+stuff_claw_height),
                   public_material_pos,
                   (-150-1,-public_processing_distance-2,stuff_claw_height-arm_height)],
                   stuff_radius_range)
red_stuff.Set_Mixing_Portion((2,0,-2))
red_stuff.Set_Height(stuff_height)

MF.Stuff_List_Init((red_stuff,green_stuff,blue_stuff))

#####################################################################################

MF.material_plate=myObject("circle",myVideo,[(210,210,210),(255,255,255)],
                           [None,public_material_pos,None],stuff_radius_range)
# MF.material_plate.Set_TransMatrix(0.15)

MF.green_ring=myObject("circle",myVideo,None,[None,None,(0,-public_processing_distance,
                                                         200-arm_height)])
MF.green_ring.Set_Mixing_Portion((-1,2,-1))

MF.edge_line=myObject("line",myVideo,[(210,205,200),(255,255,255)])

#####################################################################################

def main():
    mission_manager=Logistics_Handling
    mission_manager.Set_Logger(Public_Logger)
    mission_manager.Set_VideoStream(myVideo)
    mission_manager.Reset()
    mission_manager.Set_Permanent_Mission([Frame_Capture,Frame_Mark_Display,Frame_Save],
                                             [MF.Frame_Capture_Trigger,
                                              MF.Frame_Mark_Save_Trigger],3)
    end_flag=False
    while(True):
        end_flag=mission_manager.Run()
        if(end_flag==True):
            print("End of All Missions")
            break


if(__name__=="__main__"):
    main()