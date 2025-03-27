import cv2 as cv

def ExtractFrame():
    # 读取视频文件路径
    file_path="proj/assets/videos_temp/debug_0326_2251.avi"

    # 需要截取的帧所在时间列表,单位:s
    frame_timeList=[3.6,4.8]
    error_s=0.3
    timeList=[]
    for time in frame_timeList:
        # 似乎有0.3s误差
        time-=error_s
        time_ms=time*1000
        time_ms=int(round(time_ms))
        timeList.append(time_ms)
    print("截取帧时间列表(ms):",time_ms)

    # 读取视频文件
    cap = cv.VideoCapture(file_path)

    cnt=0

    # 循环读取视频帧
    while cap.isOpened():
        ret, frame = cap.read()
        cnt+=1
        # print("正在读取第{}帧...".format(cnt))
        if (ret==True):
            # 截取指定帧
            videoTime_ms=cap.get(cv.CAP_PROP_POS_MSEC)
            if videoTime_ms in timeList:
                cv.imwrite("proj/frame_extraction/frame_extracted/frame_{}.png"
                        .format(videoTime_ms), frame)
                print("已保存帧: frame_{}".format(videoTime_ms))
        else:
            print("程序结束,退出...")
            break

    # 释放资源
    cap.release()

def main():
    ExtractFrame()

if(__name__=="__main__"):
    main()