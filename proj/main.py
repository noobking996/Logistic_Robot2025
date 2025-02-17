import cv2 as cv
import numpy as np
import os
import Generic_Mission as GM
import time
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

Departure=MissionDef_t("启停区出发",MF.Departure_Func,[[-350,400,0],[0,100,0]],[0.8,0],True)

Scan_QRcode=MissionDef("扫码",MF.Scan_QRcode_Func,[[True,0]],True)

QRcode_2_RawMaterial=MissionDef_t("扫码->原料",MF.QRcode_2_RawMaterial_Func,[[0,700,0]],[1],True)

RawMaterial_Pos_Correction=MissionDef("原料区纠正",MF.Pos_Correction_Func,
                                      [[CP.Material],[0.25,(10,10),(30,30)]],True)

RawMaterial_Picking=MissionDef("原料区夹取",MF.RawMaterial_Picking_Func,
                               [[120,40,50],[5],[100,100,300,150],[400,250]],True)

RawMaterial_2_Processing=MissionDef_t("原料区->加工区",MF.RawMaterial_2_Processing_Func,
                                      [[0,500,0],[MOVJ_Drection.Left_Forward,280,60],[0,900,0],
                                       [MOVJ_Drection.Left_Forward,120,140],[0,500,0]],
                                       [0,0.95,1.45,1.35,1],True)

Processing_Pos_Correction=MissionDef("加工区纠正",MF.Pos_Correction_Func,
                                      [[CP.Processing]],True)

Processing_PickAndPlace=MissionDef("加工区放置回收",MF.Processing_PickAndPlace_Func,
                                   None,True)

Processing_2_Storage=MissionDef_t("加工区->暂存区",MF.Three_Section_Turn_Func,
                                  [[0,-700,0],[MOVJ_Drection.Left_Backward,190,70],[0,-500,0]],
                                  [1.05,1.47,1.2],True)

Storage_Pos_Correction=MissionDef("暂存区纠正",MF.Pos_Correction_Func,[[CP.Storage]],True)

# 第一轮专属任务
Storage_Place=MissionDef("暂存区放置",MF.Storage_Place_Func,None,True)

Storage_2_RawMaterial=MissionDef_t("暂存区->原料区",MF.Three_Section_Turn_Func,
                                    [[0,-700,0],[MOVJ_Drection.Left_Backward,190,70],[0,-300,0]],
                                    [1.1,1.7,0.8],True)

# 第二轮专属任务
Storage_Stacking=MissionDef("暂存区码垛",MF.Storage_Stacking_Func,None,True)

Storage_Go_Home=MissionDef_t("暂存区->启停区",MF.Storage_Go_Home_Func,
                             [[0,-700,0],[MOVJ_Drection.Left_Backward,190,70],
                              [0,-800,0],[200,-200,0]],
                             [1.05,1.05,2,1.2],True)

Home_Pos_Correction=MissionDef("启停区位置纠正",MF.Home_Pos_Correction_Func,None,True)

# 物流搬运任务管理,共22个子任务
# 参数列表内容:1. 常驻任务触发条件;
                                    # 第一轮 13
Logistics_Handling=MissionManager([Standby,Departure,Scan_QRcode,QRcode_2_RawMaterial,
                                   RawMaterial_Pos_Correction,RawMaterial_Picking,
                                   RawMaterial_2_Processing,Processing_Pos_Correction,
                                   Processing_PickAndPlace,Processing_2_Storage,
                                   Storage_Pos_Correction,Storage_Place,Storage_2_RawMaterial,
                                    # 第二轮 10
                                   RawMaterial_Pos_Correction,RawMaterial_Picking,
                                   RawMaterial_2_Processing,Processing_Pos_Correction,
                                   Processing_PickAndPlace,Processing_2_Storage,
                                   Storage_Pos_Correction,Storage_Stacking,Storage_Go_Home,
                                   Home_Pos_Correction],[[0,0,1]],True,4)


# 二值化调参任务定义
# 参数列表内容: [b_th],[g_th],[r_th],[th_HighOrLow,th_CoarseOrPrecise],[图片编号]
Thresholding_Test=MissionDef("二值化调参",MF.Thresholding_Test_Func,
                             [[200,255],[20,180],[20,190],[True,True],[0]],True)
# 测试任务管理器(视觉相关调试,只能在本地终端启动)
# 参数列表内容:1. 常驻任务触发条件(这里可将录像开启条件设为100,即一直不开启);
Partial_MIssion_Test=MissionManager([RawMaterial_Picking],[[0,0,0]],True,0)

#####################################################################################

# 任务代号
Mission_Code="debug_0216_1927"

# 创建公共日志记录器
Public_Logger=Setup.Logger_Setup(Mission_Code,[DEBUG,DEBUG,DEBUG])

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
yaw_compensation=5
x4_conpensation=10
MF.arm=myManipulator([(65,130,130),(71+x4_conpensation,-20-1.12,0)],Public_Logger,MF.myServo)
MF.arm.Set_Joint_to_Actuator_Matrix([[[90,430],[90-16.8,500]],
                                    [[(180-90),420],[180-(90+19.2),500]],
                                    [[0,500+yaw_compensation],[120,1000+yaw_compensation]]])
MF.arm.Set_YawAccRatio(0.2,0.25)
MF.arm.Set_Claw_Angles((800,980))

# 初始化物块对象
arm_height=148.33           #机械臂坐标系原点距离地面高度
stuff_claw_height=55+3        # 夹持时夹爪距离物料底部的距离
material_plate_height=80    # 原料盘高度
public_material_pos=(0,-240-x4_conpensation,material_plate_height+stuff_claw_height-arm_height) # 原料区夹取位置
blue_stuff=myObject("circle",myVideo,[(200,20,20),(255,180,190)],
                   [(176.46,82.28,-61+stuff_claw_height),
                    public_material_pos,
                    (0,0,stuff_claw_height-arm_height)])
blue_stuff.Set_Mixing_Portion((0,-3,3))
green_stuff=myObject("circle",myVideo,[(100,210,40),(250,255,180)],
                    [(194.7,0,-61+stuff_claw_height),
                     public_material_pos,
                     (0,0,stuff_claw_height-arm_height)])
green_stuff.Set_Mixing_Portion((-1,2,-1))
red_stuff=myObject("circle",myVideo,[(70,60,180),(255,180,255)],
                  [(176.46,-82.28-6,-61+stuff_claw_height),
                   public_material_pos,
                   (0,0,stuff_claw_height-arm_height)])
red_stuff.Set_Mixing_Portion((2,0,-2))
MF.Stuff_List_Init((red_stuff,green_stuff,blue_stuff))

#####################################################################################

MF.material_plate=myObject("ellipse",myVideo,[(210,210,210),(255,255,255)])
MF.material_plate.Set_TransMatrix(0.15)

MF.green_ring=myObject("circle",myVideo,None,[None,None,(0,-250,200-arm_height)])
MF.green_ring.Set_Mixing_Portion((-1,2,-1))

MF.edge_line=myObject("line",myVideo,[(210,220,210),(255,255,255)])

#####################################################################################

def main():
    mission_manager=Partial_MIssion_Test
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

def num_mission_test():
    print("Number of missions:",Logistics_Handling.Num_Mission)

# 行走相关任务调试(可远程启动)
def single_mission_test():
    Storage_Go_Home.Reset()
    end_flag=False
    while(True):
        end_flag=Storage_Go_Home.Run()
        if(end_flag==True):
            print("Mission Complete")
            break


if(__name__=="__main__"):
    main()
    # num_mission_test()
    # single_mission_test()
    # GM.main()