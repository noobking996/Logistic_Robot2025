import numpy as np
import math
import cv2 as cv

class Average_Filter:
    def __init__(self, sequence_len:np.uint8):
        self.Sequence_Length=np.uint8(sequence_len)
        self.Sequence=[]
        self.Average=0

    def Get_Filtered_Value(self, value):
        """
        * 获得均值滤波后的值
        @param value: 输入值,可以是单个值或numpy数组
        @return: 均值滤波后的值
        """
        self.Sequence.append(value)
        if len(self.Sequence)>self.Sequence_Length:
            self.Sequence.pop(0)
        self.Average=np.mean(self.Sequence, axis=0)
        return self.Average
    
    def Reset(self):
        self.Sequence.clear()
        self.Average=0