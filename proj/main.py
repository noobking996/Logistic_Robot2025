import cv2 as cv
import numpy as np
import os
import Generic_Mission as GM
import time

from mission.Setup import MissionManager, MissionDef, MissionDef_t
from mission import Mission_Function as MF

RawMaterial_2_Processing=MissionDef_t("原料区->加工区",MF.RawMaterial_2_Processing_Func,
                                      [[0,100,0],[1000,300],[0,100,0]],[1,2,3],True)


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