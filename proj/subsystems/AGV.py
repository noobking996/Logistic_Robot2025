import serial
import numpy as np
from enum import Enum
from typing import List,Union

# 创建长度固定为10的数据帧
data_frame_size=10
AGV_Data_Frame=bytearray(data_frame_size)

# 定义AGV指令枚举
class AGVCommand(Enum):
    VELOCITY_CONTROL=0x00
    MOVJ_CONTROL=0x01

class MOVJ_Drection(Enum):
    Left_Forward=0x00
    Right_Forward=0x01
    Left_Backward=0x02
    Right_Backward=0x03

# 定义AGV类
class myAGV:
    def __init__(self,device_id:np.uint8,uart_port:str,baud_rate:int):
        # 指定设备ID
        self.device_id=device_id
        # 指定底盘指令串口，并进行相关配置
        self.uartPort=serial.Serial(uart_port,baud_rate,timeout=0.5)


    def __str__(self):
        return ("AGV:\n\tid: {}\n\tSerial Port: {}\n\tBaud Rate: {}"
                .format(hex(self.device_id),
                        self.uartPort.name,
                        self.uartPort.baudrate))


    def Velocity_Control(self,Velo_List:List[np.int16]):

        """
        @参数Velo_List：广义速度列表[Vx_mm_s,Vy_mm_s,Omege_deg_s],int16类型
        @作用：装载并发送数据帧，共使用8有效字节
        @数据帧格式：命令声明+速度方向 + Vx_h + Vx_l + Vy_h + Vy_l + Omege_h + Omege_l
        """

        # 装载命令声明
        AGV_Data_Frame[0]=AGVCommand.VELOCITY_CONTROL.value

        Vx_mm_s=np.int16(Velo_List[0])
        Vy_mm_s=np.int16(Velo_List[1])
        Omege_deg_s=np.int16(Velo_List[2])

        # 设置广义速度方向标志位
        sign_flag=np.uint8(0x00)
        if(Vx_mm_s<0):
            sign_flag|=(0x01<<0)
        if(Vy_mm_s<0):
            sign_flag|=(0x01<<1)
        if(Omege_deg_s<0):
            sign_flag|=(0x01<<2)
        AGV_Data_Frame[1]=sign_flag

        # 装载速度大小
        Vx=np.uint16(abs(Vx_mm_s))
        Vy=np.uint16(abs(Vy_mm_s))
        Omege=np.uint16(abs(Omege_deg_s))
        Vx_bytes=Vx.tobytes('C')
        Vy_bytes=Vy.tobytes('C')
        Omege_bytes=Omege.tobytes('C')
        # 发送顺序为高位在前，低位在后
        AGV_Data_Frame[2]=Vx_bytes[1]
        AGV_Data_Frame[3]=Vx_bytes[0]
        AGV_Data_Frame[4]=Vy_bytes[1]
        AGV_Data_Frame[5]=Vy_bytes[0]
        AGV_Data_Frame[6]=Omege_bytes[1]
        AGV_Data_Frame[7]=Omege_bytes[0]
        # 一共使用8字节数据

        # 发送数据帧
        self.uartPort.write(AGV_Data_Frame)

        # 接收回应
        # 1.下位机暂无回应帧
        # 2.阻塞方式读取回应数据可能影响实时性，故暂时不处理，等待后续版本


    def MOVJ_control(self,param_list:List[Union[MOVJ_Drection, np.uint16]]):

        """
        @参数param_list：圆弧运动参数列表[direction,rou_mm,omega_deg_s],
            direction为MOVJ_Drection类型，rou_mm,omega_deg_s为uint16类型
        @作用：装载并发送数据帧，共使用6有效字节
        @数据帧格式：命令声明+方向 + rou_h + rou_l + omega_h + omega_l
        """

        # 校验参数列表
        if len(param_list) != 3:
            raise ValueError("param_list必须包含3个元素：MOVJ_Drection 和 两个 np.uint16 类型的元素")

        direction = param_list[0]
        rou_mm = np.uint16(param_list[1])
        omega_deg_s = np.uint16(param_list[2])

        # 装载命令声明
        AGV_Data_Frame[0]=AGVCommand.MOVJ_CONTROL.value

        # 装载方向
        AGV_Data_Frame[1]=direction.value

        # 装载速度大小
        rou_bytes=np.uint16(rou_mm).tobytes('C')
        omega_bytes=np.uint16(omega_deg_s).tobytes('C')
        # 发送顺序为高位在前，低位在后
        AGV_Data_Frame[2]=rou_bytes[1]
        AGV_Data_Frame[3]=rou_bytes[0]
        AGV_Data_Frame[4]=omega_bytes[1]
        AGV_Data_Frame[5]=omega_bytes[0]
        # 一共使用6字节数据

        # 发送数据帧
        self.uartPort.write(AGV_Data_Frame)

        # 接收回应
        # 1.下位机暂无回应帧
        # 2.阻塞方式读取回应数据可能影响实时性，故暂时不处理，等待后续版本  



###################################################测试代码

def Tobytes_Test():
    num16=np.uint16(200)
    # 按照C语言的字节序(按行存储)，将数据转换为字节数组
    # 字节顺序为低位在前，高位在后
    bytes_num=num16.tobytes('C')
    print(hex(bytes_num[1]))

def data_frame_test():
    print(AGV_Data_Frame)

def Velocity_Control_Test():
    agv=myAGV(0x01,"/dev/ttyAMA2",115200)
    agv.Velocity_Control([-100,200,300])

def MOVJ_direction_Enum_Test():
    print(MOVJ_Drection.Right_Backward.value)

def MOVJ_control_Test():
    agv=myAGV(0x01,"/dev/ttyAMA2",115200)
    agv.MOVJ_control(MOVJ_Drection.Right_Forward,[1000,300])

def main():
    Velocity_Control_Test()

if(__name__=="__main__"):
    main()