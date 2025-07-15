## 目录结构介绍
```text
Logistic_Robot2025/
├── arm_J2A_research.md
├── paspi_pinout.md
├── problem.md
├── proj
│   ├── assets
│   │   ├── images
│   │   │   ├── img_green_filter.png
│   │   │   └── img_qr.png
│   │   └── videos_temp
│   ├── frame_extraction
│   │   ├── captureVideoFrame.py
│   │   └── frame_extracted
│   ├── Generic_Mission.py
│   ├── main.py
│   ├── mission
│   │   ├── Math_Tools.py
│   │   ├── Mission_Function.py
│   │   ├── __pycache__
│   │   │   ├── Math_Tools.cpython-38.pyc
│   │   │   ├── Mission_Function.cpython-38.pyc
│   │   │   ├── Reusable_Module.cpython-38.pyc
│   │   │   ├── Run_Function.cpython-38.pyc
│   │   │   └── Setup.cpython-38.pyc
│   │   ├── Reusable_Module.py
│   │   └── Setup.py
│   ├── __pycache__
│   │   ├── Generic_Mission.cpython-38.pyc
│   │   ├── main.cpython-38.pyc
│   │   └── serial_test.cpython-38.pyc
│   └── subsystems
│       ├── AGV.py
│       ├── Buttom.py
│       ├── Computer_Vision.py
│       ├── Keyboard.py
│       ├── LED.py
│       ├── Manipulator.py
│       └── __pycache__
│           ├── AGV.cpython-38.pyc
│           ├── Buttom.cpython-38.pyc
│           ├── Computer_Vision.cpython-38.pyc
│           ├── Keyboard.cpython-38.pyc
│           └── Manipulator.cpython-38.pyc
├── README.md
└── todoList.md
```
### ```proj```
项目程序文件
- ```asset``` 运行视频 & 图片以及文本日志保存目录
- ```frame_extraction``` 内含工具脚本, 用于读取视频中的帧, 以便进行视频日志分析
- ```Generic_Mission.py``` 通用任务, 内含测试任务定义
- ```main.py``` 主程序, 运行该脚本启动机器人
- ```mission``` 重要目录, 包含机器人任务执行函数和任务结束回调函数定义 (```Mission_Function.py```, ```Reusable_Module.py```)、任务类和任务管理器类定义 (```Setup.py```), 以及常用数学工具函数 (```Math_Tools.py```)
- ```subsystems``` 底盘、机械臂、按钮、视觉系统的驱动程序和键盘按键定义等
### 其它
- ```problem.md``` 记录开发过程常见问题
- ```arm_J2A_research.md``` 记录关节空间到执行器空间映射关系建立方式
- ```paspi_pinout.md``` 树莓派5B引脚定义, 以及机器人引脚资源配置
- ```todoList.md``` 开发时的备忘录, 里面有一些有用的信息