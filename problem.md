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
```
/home/zhang/miniconda3/envs/computer_vision/lib/python3.8/site-packages/gpiozero/devices.py:295: PinFactoryFallback: Falling back from lgpio: No module named 'lgpio'
  warnings.warn(
Traceback (most recent call last):
  File "/home/zhang/miniconda3/envs/computer_vision/lib/python3.8/site-packages/gpiozero/pins/pi.py", line 408, in pin
    pin = self.pins[info]
KeyError: PinInfo(number=12, name='GPIO18', names=frozenset({'BCM18', 18, 'J8:12', 'GPIO18', 'WPI1', 'BOARD12', '18'}), pull='', row=6, col=2, interfaces=frozenset({'', 'pwm', 'dpi', 'spi', 'gpio', 'pcm'}))

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "proj/subsystems/LED.py", line 42, in <module>
    main()
  File "proj/subsystems/LED.py", line 39, in main
    ZERO_test()
  File "proj/subsystems/LED.py", line 30, in ZERO_test
    led = LED(18)
  File "/home/zhang/miniconda3/envs/computer_vision/lib/python3.8/site-packages/gpiozero/devices.py", line 103, in __call__
    self = super().__call__(*args, **kwargs)
  File "/home/zhang/miniconda3/envs/computer_vision/lib/python3.8/site-packages/gpiozero/output_devices.py", line 192, in __init__
    super().__init__(pin, active_high=active_high,
  File "/home/zhang/miniconda3/envs/computer_vision/lib/python3.8/site-packages/gpiozero/output_devices.py", line 74, in __init__
    super().__init__(pin, pin_factory=pin_factory)
  File "/home/zhang/miniconda3/envs/computer_vision/lib/python3.8/site-packages/gpiozero/mixins.py", line 75, in __init__
    super().__init__(*args, **kwargs)
  File "/home/zhang/miniconda3/envs/computer_vision/lib/python3.8/site-packages/gpiozero/devices.py", line 549, in __init__
    pin = self.pin_factory.pin(pin)
  File "/home/zhang/miniconda3/envs/computer_vision/lib/python3.8/site-packages/gpiozero/pins/pi.py", line 410, in pin
    pin = self.pin_class(self, info)
  File "/home/zhang/miniconda3/envs/computer_vision/lib/python3.8/site-packages/gpiozero/pins/rpigpio.py", line 101, in __init__
    GPIO.setup(self._number, GPIO.IN, self.GPIO_PULL_UPS[self._pull])
RuntimeError: Cannot determine SOC peripheral base address
```

* 转而使用RPi.GPIO库后依然有问题:
``` 
Traceback (most recent call last):
  File "proj/subsystems/LED.py", line 8, in <module>
    GPIO.setup(17, GPIO.OUT)
RuntimeError: Cannot determine SOC peripheral base address
```

* 使用系统默认环境运行gpiozero后没问题
  * 可见是miniconda环境问题
* 怀疑是miniconda环境中没有lgpio库的原因
* 试图安装lgpio库,报错:
```
Looking in indexes: https://pypi.tuna.tsinghua.edu.cn/simple
Collecting lgpio
  Using cached https://pypi.tuna.tsinghua.edu.cn/packages/56/33/26ec2e8049eaa2f077bf23a12dc61ca559fbfa7bea0516bf263d657ae275/lgpio-0.2.2.0.tar.gz (90 kB)
  Preparing metadata (setup.py) ... done
Building wheels for collected packages: lgpio
  Building wheel for lgpio (setup.py) ... error
  error: subprocess-exited-with-error
  
  × python setup.py bdist_wheel did not run successfully.
  │ exit code: 1
  ╰─> [8 lines of output]
      running bdist_wheel
      running build
      running build_py
      running build_ext
      building '_lgpio' extension
      swigging lgpio.i to lgpio_wrap.c
      swig -python -o lgpio_wrap.c lgpio.i
      error: command 'swig' failed: No such file or directory
      [end of output]
  
  note: This error originates from a subprocess, and is likely not a problem with pip.
  ERROR: Failed building wheel for lgpio
  Running setup.py clean for lgpio
Failed to build lgpio
ERROR: ERROR: Failed to build installable wheels for some pyproject.toml based projects (lgpio)
```
* 安装swig工具后,再次尝试,报错:
```
 × python setup.py bdist_wheel did not run successfully.
  │ exit code: 1
  ╰─> [18 lines of output]
      running bdist_wheel
      running build
      running build_py
      running build_ext
      building '_lgpio' extension
      swigging lgpio.i to lgpio_wrap.c
      swig -python -o lgpio_wrap.c lgpio.i
      creating build/temp.linux-aarch64-cpython-38
      gcc -pthread -B /home/zhang/miniconda3/envs/computer_vision/compiler_compat -Wl,--sysroot=/ -Wsign-compare -DNDEBUG -g -fwrapv -O3 -Wall -Wstrict-prototypes -fPIC -Isrc -I/home/zhang/miniconda3/envs/computer_vision/include/python3.8 -c lgpio_wrap.c -o build/temp.linux-aarch64-cpython-38/lgpio_wrap.o
      In file included from lgpio_wrap.c:3192:
      src/lgpio.h:377:1: warning: function declaration isn’t a prototype [-Wstrict-prototypes]
        377 | typedef void (*callbk_t) ();
            | ^~~~~~~
      creating build/lib.linux-aarch64-cpython-38
      gcc -pthread -shared -B /home/zhang/miniconda3/envs/computer_vision/compiler_compat -L/home/zhang/miniconda3/envs/computer_vision/lib -Wl,-rpath=/home/zhang/miniconda3/envs/computer_vision/lib -Wl,--no-as-needed -Wl,--sysroot=/ build/temp.linux-aarch64-cpython-38/lgpio_wrap.o -llgpio -o build/lib.linux-aarch64-cpython-38/_lgpio.cpython-38-aarch64-linux-gnu.so
      /home/zhang/miniconda3/envs/computer_vision/compiler_compat/ld: cannot find -llgpio: No such file or directory
      collect2: error: ld returned 1 exit status
      error: command '/usr/bin/gcc' failed with exit code 1
      [end of output]
```
* 怀疑库文件编译头文件缺失,尝试apt安装liblgpio-dev软件包,安装完成后,pip成功安装lgpio库;之后尝试运行gpiozero,成功运行.
### 总结
* miniconda环境与外部系统环境隔离,导致个别系统文件无法共享,需要单独安装一些库/工具包,如lgpio库
* 无法使用gpiozero库的原因是miniconda环境中缺少lgpio库,而lgpio库又依赖于swig工具,而swig工具又依赖于gcc编译器,而gcc编译器又依赖于系统头文件,而系统头文件又依赖于系统软件包
* 因此要解决这个问题,首先需要安装swig工具,然后安装liblgpio-dev软件包,最后再miniconda环境中安装lgpio库.

## 3. 视频录制FFMPEG编码报错
``` 
[h264_v4l2m2m @ 0x4f5f4b0] Could not find a valid device
[h264_v4l2m2m @ 0x4f5f4b0] can't configure encoder
[ERROR:0@0.190] global cap_ffmpeg_impl.hpp:3194 open Could not open codec h264_v4l2m2m, error: Unspecified error (-22)
[ERROR:0@0.190] global cap_ffmpeg_impl.hpp:3211 open VIDEOIO/FFMPEG: Failed to initialize VideoWriter
[ WARN:0@0.205] global cap.cpp:643 open VIDEOIO(CV_IMAGES): raised OpenCV exception:

OpenCV(4.10.0) /io/opencv/modules/videoio/src/cap_images.cpp:430: error: (-215:Assertion failed) !filename_pattern.empty() in function 'open'
```