import cv2 as cv
import numpy as np
import os
import Generic_Mission as GM
import time

from subsystems.AGV import MOVJ_Drection
from mission.Setup import MissionManager, MissionDef, MissionDef_t
from mission import Mission_Function as MF

forward_time_list=[0,1,2]

RawMaterial_2_Processing=MissionDef_t("原料区->加工区",MF.RawMaterial_2_Processing_Func,
                                      [[0,500,0],[MOVJ_Drection.Right_Forward,250,90],
                                       [0,790,0]],forward_time_list,True)


def main():
    RawMaterial_2_Processing.Reset()
    end_flag=False
    while(True):
        end_flag=RawMaterial_2_Processing.Run()
        if(end_flag==True):
            print("Mission Complete")
            break
    # print(RawMaterial_2_Processing.Name)
    # pass

if(__name__=="__main__"):
    main()