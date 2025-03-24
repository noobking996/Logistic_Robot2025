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
    POS_CTRL=0x02
    SET_PARAMs=0x03
    ANGLE_CORRECTION=0x04

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
        @param Velo_List: 广义速度列表[Vx_mm_s,Vy_mm_s,Omege_deg_s],int16类型\n
        1. 功能:装载并发送数据帧,共使用8有效字节
        2. 数据帧格式:命令声明+速度方向 + Vx_h + Vx_l + Vy_h + Vy_l + Omege_h + Omege_l
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
        @param param_list:圆弧运动参数列表[direction,rou_mm,omega_deg_s],
            direction为MOVJ_Drection类型,rou_mm,omega_deg_s为uint16类型
        1. 功能:装载并发送数据帧,共使用6有效字节
        2. 数据帧格式:命令声明+方向 + rou_h + rou_l + omega_h + omega_l
        """

        # 校验参数列表
        if len(param_list) != 3:
            raise ValueError("param_list必须包含3个元素:MOVJ_Drection 和 两个 np.uint16 类型的元素")

        direction:MOVJ_Drection = param_list[0]
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


    def Position_Control(self,pos_param_list:List[np.int16]):
        """
        * 底盘位置控制
        @param pos_param_list: 位置控制参数列表[x_mm,y_mm,z_mm,theta_degx10],int16类型
        1. 功能:装载并发送数据帧,共使用8有效字节
        2. 数据帧格式: 命令声明 + 位移方向 + x_h + x_l + y_h + y_l + theta_h + theta_l
        """
        # 装载命令声明
        AGV_Data_Frame[0]=AGVCommand.POS_CTRL.value

        x_mm=np.int16(pos_param_list[0])
        y_mm=np.int16(pos_param_list[1])
        # 参数theta_degx10为转动角度值*10,目的是实现小角度转动(分辨率0.1度)
        theta_degx10=np.int16(pos_param_list[2])

        # 设置广义速度方向标志位
        sign_flag=np.uint8(0x00)
        if(x_mm<0):
            sign_flag|=(0x01<<0)
        if(y_mm<0):
            sign_flag|=(0x01<<1)
        if(theta_degx10<0):
            sign_flag|=(0x01<<2)
        AGV_Data_Frame[1]=sign_flag

        # 装载速度大小
        x=np.uint16(abs(x_mm))
        y=np.uint16(abs(y_mm))
        theta=np.uint16(abs(theta_degx10))
        x_bytes=x.tobytes('C')
        y_bytes=y.tobytes('C')
        theta_bytes=theta.tobytes('C')
        # 发送顺序为高位在前，低位在后
        AGV_Data_Frame[2]=x_bytes[1]
        AGV_Data_Frame[3]=x_bytes[0]
        AGV_Data_Frame[4]=y_bytes[1]
        AGV_Data_Frame[5]=y_bytes[0]
        AGV_Data_Frame[6]=theta_bytes[1]
        AGV_Data_Frame[7]=theta_bytes[0]
        # 一共使用8字节数据

        # 发送数据帧
        self.uartPort.write(AGV_Data_Frame)

    
    def Angle_Correction(self,target_angle_deg:np.int16):
        """
        * 角度校准
        @param target_angle_deg: 目标角度值,int16类型,范围为(-180,180]
        1. 功能:装载并发送数据帧,共使用3有效字节
        2. 数据帧格式: 命令声明 + 角度正负值标志位 + 目标角度值
        3.注意: 函数内部没有声明target_angle_deg为int16型,可能有bug
        """
        # 校验目标角度值
        if(target_angle_deg<=-180 or target_angle_deg>180):
            raise ValueError("角度值必须在(-180,180]范围内")
        # 装载命令声明
        AGV_Data_Frame[0]=AGVCommand.ANGLE_CORRECTION.value
        minus_flag=np.uint8(0x00)
        # 设置角度正负值标志位
        if(target_angle_deg<0):
            minus_flag=np.uint8(0x01)
        AGV_Data_Frame[1]=minus_flag
        # 装载角度大小
        angle=np.uint8(abs(target_angle_deg))
        AGV_Data_Frame[2]=angle
        # 一共使用3字节数据

        # 发送数据帧
        self.uartPort.write(AGV_Data_Frame)


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