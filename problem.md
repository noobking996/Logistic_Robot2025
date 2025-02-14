# 问题汇总

## 1. ssh中运行python脚本时cv.imshow()失败
* 报错如下:
```
qt.qpa.xcb: could not connect to display
qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "/home/zhang/miniconda3/envs/computer_vision/lib/python3.8/site-packages/cv2/qt/plugins" even though it was found.
This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.

Available platform plugins are: xcb.
```
* 在raspi本地终端中运行时正常

## 2. gpiozero库无法使用

## 3. 视频录制FFMPEG编码报错
``` 
[h264_v4l2m2m @ 0x4f5f4b0] Could not find a valid device
[h264_v4l2m2m @ 0x4f5f4b0] can't configure encoder
[ERROR:0@0.190] global cap_ffmpeg_impl.hpp:3194 open Could not open codec h264_v4l2m2m, error: Unspecified error (-22)
[ERROR:0@0.190] global cap_ffmpeg_impl.hpp:3211 open VIDEOIO/FFMPEG: Failed to initialize VideoWriter
[ WARN:0@0.205] global cap.cpp:643 open VIDEOIO(CV_IMAGES): raised OpenCV exception:

OpenCV(4.10.0) /io/opencv/modules/videoio/src/cap_images.cpp:430: error: (-215:Assertion failed) !filename_pattern.empty() in function 'open'
```