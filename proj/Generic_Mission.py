import cv2 as cv
import numpy as np
import os
import serial
import time
import logging
from logging import Logger,DEBUG,INFO,WARNING,ERROR,CRITICAL
# from typing import Sized
# import pynput as pt

# 设置工作目录
# target_directory = '/home/zhang/Logistic_Robot2025'
# os.chdir(target_directory)

def Send_Cmd():
    # 使用端口为uart3(tx=gpio4,rx=gpio5)
    uartPort="/dev/ttyAMA2"

    #创建Serial类对象
    myUART=serial.Serial(uartPort,115200,timeout=0.5)

    # myUART.open() 
    # 已修改boot配置文件，自动打开

    # 发送数据帧
    if(myUART.is_open):
        # 发送数据帧
        data_write=bytearray([0x00,0x01,0x02])
        # myUART.write("树莓派串口测试".encode("utf-8"))
        myUART.write(data_write)

        # 读取数据帧
        # print("waiting...")
        # dataframe_size=3
        # data_read=myUART.read(dataframe_size)
        # print("data_read=",data_read.hex(' '))
    else:
        print("Serial open failure")


def WaitKey_Test():
    img=cv.imread("proj/assets/images/img_green_filter.png",cv.IMREAD_GRAYSCALE)
    cv.imshow("img",img)
    # key=None
    # 13是回车键的值
    while(True):
        key=cv.waitKey(-1)
        key=key & 0xFF
        if(key==ord('q')):
            break
        print(key)


# pollkey()会检测按键是否被点击，而非按下
# 按键值返回-1表示没有按键按下
def Poll_Key_Test():
    img=cv.imread("proj/assets/img_green_filter.png",cv.IMREAD_GRAYSCALE)
    cv.imshow("img",img)
    key=None
    while(key!=13):
        key=cv.pollKey()
        # print(key)
        if(key==-1):
            print("none!!")
        else:
            print(key)


def Camera_Test():
    # 初始化摄像头
    cap = cv.VideoCapture(0)  # 0 通常是默认摄像头的标识

    # 获取尺寸、帧率等特征
    width=cap.get(cv.CAP_PROP_FRAME_WIDTH)
    width=int(width)
    height=cap.get(cv.CAP_PROP_FRAME_HEIGHT)
    height=int(height)
    video_fps=cap.get(cv.CAP_PROP_FPS)
    print("video properties:(width,height,fps)=({},{},{})".format(width,height,video_fps))

    # 获得编码格式
    # Four Character Code ,定义视频文件的压缩编码格式
    # 编码格式选择: https://acn7ftxrakg9.feishu.cn/docx/EfDLd3np0oT423x3WwtchDCrnJ0?from=from_copylink
    # 采用XVID编码格式，MPEG-4编码格式的一种
    video_fourcc=cv.VideoWriter.fourcc(*'XVID')

    # 初始化视频保存参数
    frame_size=(width,height)
    video_file_path="/home/zhang/Videos/LogisticRobot2025_Capture/video_saved.avi"
    video_saved=cv.VideoWriter(video_file_path,video_fourcc,video_fps,frame_size,True)
    
    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("无法打开摄像头")
        exit()
    
    status_flag=0
    key=None

    annote_flag=1
    # 设置显示标记的位置、字体、大小、颜色、粗细
    annote_pos=(10,35)
    annote_font=cv.FONT_HERSHEY_SIMPLEX
    annote_font_size=1.5
    annote_font_color=(0,255,0)
    annote_font_thickness=3


    # 设置十字准星的位置、长度、颜色(BGR)、粗细
    # frame_shape:(num_rows,num_cols,num_channels)
    frame_shape=(height,width)
    cross_length=25
    cross_color=(0,255,0)
    cross_thickness=2

    # cross_pos：(column,row)
    cross_pos=(int(round(frame_shape[1]/2,0)),int(round(frame_shape[0]/2,0)))
    cross_length_half=int(round(cross_length/2,0))
    cross_c_p1=(cross_pos[0],cross_pos[1]-cross_length_half)
    cross_c_p2=(cross_pos[0],cross_pos[1]+cross_length_half)
    cross_r_p1=(cross_pos[0]-cross_length_half,cross_pos[1])
    cross_r_p2=(cross_pos[0]+cross_length_half,cross_pos[1])


    # 获取初始时间
    time_0=time.time()

    # 循环读取视频帧
    while True:
        # 获取当前时间
        current_time=round((time.time()-time_0),1)
        annote_text="{}s".format(current_time)

        # 按键检测
        # 按位与操作是为过滤8位以上的位。可能是键盘读取值不是纯ASCII值
        # 回车的ASCII值为13 但是用'\r'识别失败

        # 按 'c' 切换显示模式(RGB/GRAY_SCALE)
        key=(cv.pollKey() & 0xFF)
        if(key==ord('c')):
            if(status_flag==0):
                status_flag=1
            elif(status_flag==1):
                status_flag=0

        # 按 'q' 退出循环
        elif(key==ord('q')):
            break

        # 读取一帧画面
        retval, frame = cap.read()
    
        # 如果正确读取帧，retval为True
        if(retval==False):
            print("无法接收帧，请退出")
            break

        # 帧尺寸为(480，640，3)==(height,width,channel)
        # print("frame shape:",frame.shape)

        frame_converted=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
        frame_converted=cv.cvtColor(frame_converted,cv.COLOR_GRAY2BGR)

        if(status_flag==0):
            if(annote_flag==1):
                frame_display=cv.putText(frame,annote_text,annote_pos,annote_font,
                                         annote_font_size,annote_font_color,
                                         annote_font_thickness)
                # 添加十字准星
                # 纵向线
                frame_display=cv.line(frame_display,cross_c_p1,cross_c_p2,cross_color,
                                      cross_thickness,cv.LINE_AA)
                # 横向线
                frame_display=cv.line(frame_display,cross_r_p1,cross_r_p2,cross_color,
                                      cross_thickness,cv.LINE_AA)
            else:
                frame_display=frame
        elif(status_flag==1):
            if(annote_flag==1):
                frame_display=cv.putText(frame_converted,annote_text,annote_pos,annote_font,
                                           annote_font_size,annote_font_color,
                                           annote_font_thickness,cv.LINE_AA)
            else:
                frame_display=frame_converted

        # 更新画面，显示帧
        cv.imshow('Video Stream', frame_display)
        video_saved.write(frame_display)
    
    
    # 释放摄像头资源
    cap.release()
    # 释放视频文件资源
    video_saved.release()
    # 关闭所有OpenCV窗口
    cv.destroyAllWindows()


def Logging_test():
    # 创建日志文件
    # logging.basicConfig(filename='proj/log/test.log',level=logging.DEBUG)

    # 创建一个日志记录器
    logger = logging.getLogger('my_logger')
    logger.setLevel(logging.DEBUG)

    mission_code="预调试"

    # 创建文件处理器
    file_handler = logging.FileHandler("proj/assets/logs/{}.log".format(mission_code))
    file_handler.setLevel(logging.DEBUG)

    # 创建终端处理器
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)

    # 创建格式化器
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', 
                                  datefmt='%Y-%m-%d %H:%M:%S')

    # 设置格式化器
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)

    # 添加处理器
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    # 记录日志消息
    logger.log(DEBUG,'这是一个调试信息')
    logger.log(INFO,'这是一个信息信息')
    logger.log(WARNING,'这是一个警告信息')
    logger.log(ERROR,'这是一个错误信息')
    logger.log(CRITICAL,'这是一个严重错误信息')


def Window_Test():
    cv.namedWindow("window")
    cv.moveWindow("window",320,180)
    img=cv.imread("proj/assets/img_green_filter.png",cv.IMREAD_GRAYSCALE)
    while(True):
        cv.imshow("window",img)

        key=(0xff & cv.pollKey())
        if(key==ord('q')):
            cv.destroyAllWindows()
            break

def main():
    # print("run in workspace:",os.getcwd())

    # img=cv.imread("proj/assets/img_green_filter.png",cv.IMREAD_GRAYSCALE)
    # print("size=",img.shape)
    # print("dtype=",img.dtype)
    # cv.imwrite("proj/assets/img.png",img)

    # Send_Cmd()
    WaitKey_Test()
    # Camera_Test()
    # Logging_test()
    # Window_Test()

# 作为主函数文件运行时运行main()，作为库导入时不运行
if(__name__=="__main__"):
    main()