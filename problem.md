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