import cv2 as cv
import numpy as np
import os
import Generic_Mission as GM
import time

from subsystems.AGV import MOVJ_Drection
from mission.Setup import MissionManager, MissionDef, MissionDef_t
from mission.Setup import Correction_PosDef as CP
from mission import Mission_Function as MF


Departure=MissionDef_t("启停区出发",MF.Departure_Func,[[-350,400,0],[0,200,0]],[0.8,0],True)

Scan_QRcode=MissionDef("扫码",MF.Scan_QRcode_Func,None,True)

QRcode_2_RawMaterial=MissionDef_t("扫码->原料",MF.QRcode_2_RawMaterial_Func,[[0,700,0]],[0.85],True)

RawMaterial_Pos_Correction=MissionDef("原料区纠正",MF.Pos_Correction_Func,
                                      [[CP.Material]],True)

RawMaterial_Picking=MissionDef("原料区夹取",MF.RawMaterial_Picking_Func,None,True)

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
Logistics_Handling=MissionManager([Departure,Scan_QRcode,QRcode_2_RawMaterial,      # 第一轮 12
                                   RawMaterial_Pos_Correction,RawMaterial_Picking,
                                   RawMaterial_2_Processing,Processing_Pos_Correction,
                                   Processing_PickAndPlace,Processing_2_Storage,
                                   Storage_Pos_Correction,Storage_Place,Storage_2_RawMaterial,

                                   RawMaterial_Pos_Correction,RawMaterial_Picking,  # 第二轮 10
                                   RawMaterial_2_Processing,Processing_Pos_Correction,
                                   Processing_PickAndPlace,Processing_2_Storage,
                                   Storage_Pos_Correction,Storage_Stacking,Storage_Go_Home,
                                   Home_Pos_Correction],None,True,0)


def main():
    Logistics_Handling.Reset()
    end_flag=False
    while(True):
        end_flag=Logistics_Handling.Run()
        if(end_flag==True):
            print("Mission Complete")
            break

def num_mission_test():
    print("Number of missions:",Logistics_Handling.Num_Mission)

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