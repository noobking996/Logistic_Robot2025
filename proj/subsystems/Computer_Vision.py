from enum import Enum
from cv2 import VideoCapture
import cv2 as cv
from logging import Logger
from typing import Tuple
import numpy as np


# 视频流类,用于存储一些视频流的参数
class Video_Stream:
    def __init__(self,cap:VideoCapture,mission_logger:Logger,video_filePath:str,mission_code:str="Video Stream"):
        self.Cap=cap

        width=cap.get(cv.CAP_PROP_FRAME_WIDTH)
        width=int(width)
        height=cap.get(cv.CAP_PROP_FRAME_HEIGHT)
        height=int(height)
        video_fps=cap.get(cv.CAP_PROP_FPS)
        mission_logger.info("Video Capture: width={}, height={}, fps={}"
                            .format(width,height,video_fps))
        self.Frame_Shape=(height,width)
        self.Frame_Shape_Half=(height>>1,width>>1)
        self.Frame_Shape_Quarter=(height>>2,width>>2)

        # 自带窗口名取名为任务代码
        self.Window_Name=mission_code
        cv.namedWindow(self.Window_Name,cv.WINDOW_AUTOSIZE)
        cv.moveWindow(self.Window_Name,100,-39)

        # 设置显示文字标记的位置、字体、大小、颜色、粗细
        self.Text_pos=(3,25)
        self.Text_font_size=0.8
        self.Text_font_color=(0,250,0)
        self.Text_font_thickness=2

        # cross_pos：(column,row)
        # 准星定位在图像中心
        cross_length=25
        cross_pos=(int(round(width/2,0)),int(round(height/2,0)))
        cross_length_half=int(round(cross_length/2,0))
        cross_c_p1=(cross_pos[0],cross_pos[1]-cross_length_half)
        cross_c_p2=(cross_pos[0],cross_pos[1]+cross_length_half)
        cross_r_p1=(cross_pos[0]-cross_length_half,cross_pos[1])
        cross_r_p2=(cross_pos[0]+cross_length_half,cross_pos[1])
        self.CrossLine_vPoints=[cross_c_p1,cross_c_p2]
        self.CrossLine_hPoints=[cross_r_p1,cross_r_p2]

        video_fourcc=cv.VideoWriter.fourcc(*'XVID')
        self.Video_Saved=cv.VideoWriter(video_filePath,video_fourcc,video_fps,
                                        (width,height),True)

    def Mark_Text(self,frame,text:str,pos:Tuple=None,font_size:float=None,
                  font_thickness:int=None,color:Tuple=None):
        """
        @功能: 标记文字
        @参数: frame, 帧
        @参数: text, 文字

        @参数(选填): pos, 文字位置(c,r)
        @参数(选填): font_size, 文字大小,float型
        @参数(选填): font_thickness, 文字粗细,int型
        @参数(选填): color, 文字颜色(b,g,r)
        @注意事项: 选填参数未赋值时,自动采用视频流默认参数
        """
        error_flag=False
        if(pos==None):
            pos=self.Text_pos
        elif(len(pos)!=2):
            error_flag=True
        if(font_size==None):
            font_size=self.Text_font_size
        if(color==None):
            color=self.Text_font_color
        elif(len(color)!=3):
            error_flag=True
        if(font_thickness==None):
            font_thickness=self.Text_font_thickness
        if(error_flag==True):
            raise ValueError("Invalid parameters for Mark_Text")
        text_font=cv.FONT_HERSHEY_SIMPLEX
        cv.putText(frame,text,pos,text_font,font_size,color,font_thickness,cv.LINE_AA)

    def Mark_Cross(self,frame):
        # 设置十字准星的位置、长度、颜色(BGR)、粗细
        # frame_shape:(num_rows,num_cols,num_channels)
        cross_color=(0,50,0)
        cross_thickness=1
        # 标记十字(可以用cv::drawMarker()代替)
        cv.line(frame,self.CrossLine_vPoints[0],self.CrossLine_vPoints[1],cross_color,
                cross_thickness,cv.LINE_AA)
        cv.line(frame,self.CrossLine_hPoints[0],self.CrossLine_hPoints[1],cross_color,
                cross_thickness,cv.LINE_AA)

    def Make_Thumbnails(self,img,shrink_ratio:np.uint8=4,dsize:Tuple[int,int]=None,
                        quality_HighOrLow=False)->np.ndarray:
        """
        @功能: 生成缩略图
        @参数: img, 图片
        @参数: shrink_ratio, 缩小比例,预设2和4倍
        @参数(选填): size, 缩略图大小(w,h),若填写该值则忽略shrink_ratio
        @参数: quality_HighOrLow, 图片质量高(True)/低(False)
        @返回值: 缩略图
        """
        # 若不填写dsize,则根据shrink_ratio确定缩略图大小
        if(dsize==None):
            height=0
            width=0
            if(shrink_ratio==2):
                height,width=self.Frame_Shape_Half
            elif(shrink_ratio==4):
                height,width=self.Frame_Shape_Quarter
            dsize=(width,height)
        interpolation_algorithm=None
        if(quality_HighOrLow==True):
            interpolation_algorithm=cv.INTER_AREA
        else:
            interpolation_algorithm=cv.INTER_NEAREST
        return cv.resize(img,dsize,interpolation=interpolation_algorithm)

    def Paste_Img(self,frame:np.ndarray,img_pasted:np.ndarray,pos:Tuple[int,int]=None,
                  org_buttom_right=True)->None:
        """
        @功能: 将图片img粘贴到frame中,覆盖其一部分
        @参数: frame, 帧
        @参数: img_pasted, 图片
        @ 参数: pos, 粘贴位置(c,r),若不填写,则赋值为frame右下角
        @参数(选填): org_buttom_right,决定图片坐标原点在其左上角(false)/右下角(true)
        @注意事项: 原图在右下角时,pos=(0,0)表示粘贴位置在左上角
        """
        height,width=img_pasted.shape[:2]
        if(pos==None):
            pos=(self.Frame_Shape[1],self.Frame_Shape[0])
        c,r=pos
        if(org_buttom_right==False):
            c0,r0=c,r
            c1,r1=c+width,r+height
        else:
            c0,r0=c-width,r-height
            c1,r1=c,r
        frame[r0:r1,c0:c1]=img_pasted
        # 有说法认为在函数中的赋值通过引用实现,因此局部变量frame可能与外部变量frame指向同一内存地址
        # 因此,在函数中对frame的修改,也会反映到外部变量frame中,无需手动返回

    def Update_Window(self,frame):
        """
        @功能: 更新窗口显示
        @参数: frame, 显示的帧
        """
        cv.imshow(self.Window_Name,frame)
        

    def Get_Frame_Shape(self):
        """
        @功能: 获取帧大小
        @返回值: 帧大小
        """
        return self.Frame_Shape

    def Read_Frame(self):
        """
        @功能: 读取帧
        @返回值: retval是否成功读取
        @返回值: frame_captured读取到的帧
        """
        return self.Cap.read()
    
    def Release_VideoCapture(self):
        """
        @功能: 释放帧捕获
        """
        self.Cap.release()

    def Save_Frame(self,frame):
        """
        @功能: 保存视频
        @参数: frame, 帧
        """
        self.Video_Saved.write(frame)

    def Release_VideoWriter(self):
        """
        @功能: 释放视频写入器
        """
        self.Video_Saved.release()

def Video_Setup(mission_code:str,mission_logger:Logger):
    cap=VideoCapture(0)
    if(cap.isOpened()):
        file_path=("proj/assets/videos_temp/{}.avi".format(mission_code))
        video=Video_Stream(cap,mission_logger,file_path,mission_code)
        mission_logger.debug("Video Setup OK")
        return video
    else:
        mission_logger.error("Video Setup Error")
        return None


def main():
    pass

if (__name__=="__main__"):
    main()