import cv2 as cv
import numpy as np
import os
import serial
import time
import logging
from matplotlib import pyplot as plt
from logging import Logger,DEBUG,INFO,WARNING,ERROR,CRITICAL
from subsystems.Keyboard import Keyboard_Enum as kb
from typing import Sized,List
# import pynput as pt

# 设置工作目录
# target_directory = '/home/zhang/Logistic_Robot2025'
# os.chdir(target_directory)

def Show_Code_Test():
    t0=time.time()
    flag=0
    while(True):
        key=(0xff & cv.pollKey())
        if(key==ord('q')):
            break
        if(time.time()-t0>=1):
            flag+=1
            t0=time.time()
            print(flag)
        if(flag==1):
            img=np.zeros((400,730),np.uint8)
            img=cv.putText(img,"123+321",(0,250),cv.FONT_ITALIC,5,(255,255,255),10,cv.LINE_AA)
            cv.namedWindow("window",cv.WINDOW_AUTOSIZE)
            cv.moveWindow("window",65,0)
            cv.imshow("window",img)
        elif(flag==10):
            cv.destroyAllWindows()
            break
    # cv.waitKey(0)
    # cv.imwrite("img_saved.png",img)
    # cv.destroyAllWindows()

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

def Frame_Annote_Test():
    cv.namedWindow("window")
    cv.moveWindow("window",100,-39)
    frame=cv.imread("proj/assets/images/frame_captured.png",cv.IMREAD_COLOR)
    # 设置显示标记的位置、字体、大小、颜色、粗细
    # 标记文字的原点似乎不是左上角，而是左下角?
    annote_pos=[3,380]
    annote_font=cv.FONT_HERSHEY_SIMPLEX
    annote_font_size=0.8
    annote_font_color=(0,255,0)
    annote_font_thickness=2
    annote_text=["{}B:(144,255)".format('*'),
                  "{}G:(144,255)".format(' '),
                  "{}R:(144,255)".format(' ')]
    text_interval=30


    # 设置十字准星的位置、长度、颜色(BGR)、粗细
    # frame_shape:(num_rows,num_cols,num_channels)
    frame_shape=(480,640)
    cross_length=25
    cross_color=(0,255,0)
    cross_thickness=1

    # cross_pos：(column,row)
    cross_pos=(int(round(frame_shape[1]/2,0)),int(round(frame_shape[0]/2,0)))
    cross_length_half=int(round(cross_length/2,0))
    cross_c_p1=(cross_pos[0],cross_pos[1]-cross_length_half)
    cross_c_p2=(cross_pos[0],cross_pos[1]+cross_length_half)
    cross_r_p1=(cross_pos[0]-cross_length_half,cross_pos[1])
    cross_r_p2=(cross_pos[0]+cross_length_half,cross_pos[1])
    for i in range(3):
        annote_pos[1]+=text_interval
        pos=np.array(annote_pos)
        cv.putText(frame,annote_text[i],pos,annote_font,
                    annote_font_size,annote_font_color,
                    annote_font_thickness)
    # 添加十字准星
    cv.line(frame,cross_c_p1,cross_c_p2,cross_color,
                            cross_thickness,cv.LINE_AA)
    cv.line(frame,cross_r_p1,cross_r_p2,cross_color,
                            cross_thickness,cv.LINE_AA)
    cv.imshow("window",frame)
    key=cv.waitKey(0)
    if(key==kb.ENTER.value):
        cv.imwrite("proj/assets/images/frame_annotated.png",frame)
    cv.destroyAllWindows()

def RGB_Statistical_Test():
    frame=cv.imread("proj/assets/images/frame_annotated.png",cv.IMREAD_COLOR)
    cv.cvtColor(frame,cv.COLOR_BGR2RGB)
    plt.imshow(frame)
    plt.show()
    cv.waitKey(0)
    plt.close('all')

def Overlay_Test():
    frame=np.zeros((480,640,1),dtype=np.uint8)
    frame_processed=frame.copy()
    print("frame shape=",frame.shape)
    frame_processed.fill(255)

    # 法一: 自行计算形状
    # ov_shape=[np.uint16(480)>>1,np.uint16(640)>>1]
    #     # resize()的dsize是(width,height)? 理解为(x*fx,y*fy)
    #     # size & shape 似乎是不同的概念
    #     # size=width*height
    # print("ov_shape=",ov_shape)
    # ov_shape=ov_shape[::-1]
    # frame_resized:np.ndarray=cv.resize(frame_processed,ov_shape)

    # 法二: 输入倍率
    # 插值方法似乎默认为LINER
    frame_resized:np.ndarray=cv.resize(frame_processed,None,fx=0.25,fy=0.25)

    # shape=(height,width)||(num_rows,num_cols);切勿与cv中的像素坐标(c,r)||(u,v)相混淆
    height,width=frame_resized.shape
    
    print("ov_size=",width,height)
    ov_pos=(480-height,640-width)
    # frame[ov_pos[0]:ov_pos[0]+width,ov_pos[1]:ov_pos[1]+height,0]=frame_resized
    frame[ov_pos[0]:(ov_pos[0]+height),ov_pos[1]:(ov_pos[1]+width),0]=frame_resized
    # frame[ov_pos[0]:ov_pos[0]+width][ov_pos[1]:ov_pos[1]+height][0]=frame_resized
    # frame[(ov_pos[0]):ov_pos[0]-width:-1,(ov_pos[1]):ov_pos[1]-height:-1,0]=frame_resized
    # print("resized_size",frame_resized.size)
    # print("resized_shape",frame_resized.shape)
    cv.imshow("frame",frame)
    # cv.imshow("frame_resized",frame_resized)
    key=(0xff & cv.waitKey(0))
    if(key==kb.ENTER.value):
        cv.imwrite("proj/assets/images/img_ovTest.png",frame)
    cv.destroyAllWindows()

def Overlay_Test2():
    # 决定缩略图坐标原点在其左上角(false)/右下角(true)
    org_flag=True

    frame=cv.imread("proj/assets/images/img_green_filter.png",cv.IMREAD_COLOR)
    frame_processed=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
    frame_processed=cv.cvtColor(frame_processed,cv.COLOR_GRAY2BGR)

    height,width,channal=frame_processed.shape
    ov_pos=(width,height)
    height=height>>2
    width=width>>2
    dsize=(width,height)
    frame_processed=cv.resize(frame_processed,dsize,cv.INTER_NEAREST)

    c,r=ov_pos
    if(org_flag==False):
        # 左上->右下
        frame[r:r+height,c:c+width,:]=frame_processed
    else:
        # 右下->左上
        frame[r-height:r,c-width:c,:]=frame_processed
    cv.imshow("frame",frame)
    key=(0xff & cv.waitKey(0))
    if(key==kb.ENTER.value):
        cv.imwrite("proj/assets/images/img_ovTest_liner.png",frame)
    cv.destroyAllWindows()

from pyzbar.pyzbar import decode
from pyzbar.pyzbar import Decoded
def QRcode_Test():
    img=cv.imread("proj/assets/images/frame_captured_0.png",cv.IMREAD_COLOR)
    img_processed=cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    obj_list:List[Decoded]=decode(img_processed)
    try:
        if(len(obj_list)==0):
            print("No QRcode detected")
        for obj in obj_list:
            # type为str型, data为bytes型
            print('\"Type\":{}, type:{},  '.format(obj.type,type(obj.type)))
            print("data type:",type(obj.data))
            code_data:bytes=obj.data
            print('Data(raw): ', code_data)
            data_decoded=code_data.decode()
            print('Data(decoded): ', data_decoded)
            c,r,width,height=obj.rect
            cv.rectangle(img,(c,r),(c+width,r+height),(0,0,255),2)
        cv.imshow("img",img)
        key=0xff & cv.waitKey(0)
        if(key==kb.ENTER.value):
            cv.imwrite("proj/assets/images/img_qr.png",img)
    except Exception as e:
        print(e)

def str_test():
    str1="12345"
    for char in str1:
        print(char)

def Ellipse_Test():
    myLogger=Logger("myLogger",logging.DEBUG)
    file_handler = logging.FileHandler("proj/assets/logs/Ellipse_Test.log")
    file_handler.setLevel(DEBUG)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', 
                                  datefmt='%Y-%m-%d %H:%M:%S')
    file_handler.setFormatter(formatter)
    myLogger.addHandler(file_handler)
    img=cv.imread("proj/assets/images/plate_inrange0.png",cv.IMREAD_GRAYSCALE)
    # 使用external模式，只检测外轮廓
    # 轮廓近似方法为cv.CHAIN_APPROX_NONE
    # 使用cv.CHAIN_APPROX_SIMPLE模式只能检测到一个点，无法绘制椭圆
    contours,_=cv.findContours(img,cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    img=cv.cvtColor(img,cv.COLOR_GRAY2BGR)
    # print("contours=",contours)
    myLogger.debug("num_contours={}".format(len(contours)))
    # 遍历轮廓，计算轮廓的外接椭圆
    for cnt in contours:
        # print("cnt=",cnt)
        # try:
        arcLength=cv.arcLength(cnt,True)
        area=cv.contourArea(cnt)
        if (arcLength>1000):
            myLogger.debug("area={}".format(area))
            myLogger.debug("arcLength={}".format(arcLength))
            ellipse=cv.fitEllipse(cnt)
            myLogger.debug("ellipse={}".format(ellipse))
            # 绘制椭圆
            cv.ellipse(img,ellipse,(0,255,0),2)
        # except:
        #     pass
    cv.imshow("img",img)
    key=(0xff & cv.waitKey(0))
    if(key==kb.ENTER.value):
        cv.imwrite("proj/assets/images/plate_ellipse_2.png",img)
    cv.destroyAllWindows()
def main():
    # print("run in workspace:",os.getcwd())

    # img=cv.imread("proj/assets/img_green_filter.png",cv.IMREAD_GRAYSCALE)
    # print("size=",img.shape)
    # print("dtype=",img.dtype)
    # cv.imwrite("proj/assets/img.png",img)

    # Send_Cmd()
    # WaitKey_Test()
    # Camera_Test()
    # Logging_test()
    # Window_Test()
    # Frame_Annote_Test()
    # RGB_Statistical_Test()
    # Overlay_Test2()
    # QRcode_Test()
    # str_test()
    # Ellipse_Test()
    Show_Code_Test()

# 作为主函数文件运行时运行main()，作为库导入时不运行
if(__name__=="__main__"):
    main()