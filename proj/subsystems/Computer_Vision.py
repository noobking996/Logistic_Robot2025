from enum import Enum
from cv2 import VideoCapture
import cv2 as cv
from logging import Logger
from typing import Tuple


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
        self.Frame_Size=(width,height)

        # 自带窗口名取名为任务代码
        self.Window_Name=mission_code
        cv.namedWindow(self.Window_Name,cv.WINDOW_AUTOSIZE)
        cv.moveWindow(self.Window_Name,320,180)

        # 设置显示文字标记的位置、字体、大小、颜色、粗细
        self.Text_pos=(10,35)
        self.Text_font_size=1.5
        self.Text_font_color=(0,255,0)
        self.Text_font_thickness=3

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
                                        self.Frame_Size,True)

    def Mark_Text(self,frame,text:str):
        """
        @功能: 标记文字
        @参数: frame, 帧
        @参数: text, 文字
        """
        text_font=cv.FONT_HERSHEY_SIMPLEX
        cv.putText(frame,text,self.Text_pos,text_font,self.Text_font_size,
                   self.Text_font_color,self.Text_font_thickness,cv.LINE_AA)

    def Mark_Cross(self,frame):
        # 设置十字准星的位置、长度、颜色(BGR)、粗细
        # frame_shape:(num_rows,num_cols,num_channels)
        cross_color=(0,255,0)
        cross_thickness=2
        # 标记十字
        cv.line(frame,self.CrossLine_vPoints[0],self.CrossLine_vPoints[1],cross_color,
                cross_thickness,cv.LINE_AA)
        cv.line(frame,self.CrossLine_hPoints[0],self.CrossLine_hPoints[1],cross_color,
                cross_thickness,cv.LINE_AA)

    def Update_Window(self,frame):
        """
        @功能: 更新窗口显示
        @参数: frame, 显示的帧
        """
        cv.imshow(self.Window_Name,frame)
        

    def Get_Frame_Size(self):
        """
        @功能: 获取帧大小
        @返回值: 帧大小
        """
        return self.Frame_Size

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