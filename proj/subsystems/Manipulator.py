import cv2 as cv
import numpy as np
import math
import time
from math import pi,sin,cos,degrees,radians
from typing import List,Tuple,Any
from enum import Enum
import logging
from logging import Logger
import serial

class Servo:
    def __init__(self,uart_port:str,baud_rate:int=None,
                 servo_type:str="HTS-35",angle_range:Tuple[Any,Any]=(0,1000)):
        """
        @功能:舵机控制通信层
        @参数: uart_port: 串口端口号
        @参数: baud_rate: 波特率,默认为9600
        @参数: servo_type: 舵机类型,默认为幻尔机器人HTS-35
        @参数: angle_range: 舵机角度范围,默认为(0,1000)
        """
        self.uartPort=serial.Serial(uart_port,baud_rate,timeout=0.5)
        self.type=servo_type
        self.Angle_Range=angle_range
        self.dataFrame_Header=bytearray([0x55,0x55])
        self.Maximum_idNum=40
        self.Maximum_ActionGroup_Code=230
        self.timeRange_ms=(50,30000)

    def Angle_Ctrl(self,angles:List[Tuple[np.uint8,np.uint16]],time_ms:np.uint16):
        """
        @功能: 舵机角度控制,当前仅适用于控制幻尔机器人HTS-35舵机
        @参数: angles: 舵机角度列表,格式为(id,angle)
            HTS-35的角度值是(0,240)映射到(0,1000)
        @参数: time_ms: 控制周期,单位毫秒
        @数据帧格式:
        frame_header(2B) + data_length + reg_add + {num_servo + (time_l + time_h + 
        [id + angle_l + angle_h]+[...]+...)}
        """
        reg_add=np.uint8(0x03)
        num_servo=len(angles)
        data_length=3*num_servo+5
        num_servo=np.uint8(num_servo)
        data_length=np.uint8(data_length)
        # frame_length=data_length+2
        data_frame=bytearray()
        data_frame.extend(self.dataFrame_Header)
        data_frame.extend(data_length.tobytes('C'))
        data_frame.extend(reg_add.tobytes('C'))
        data_frame.extend(num_servo.tobytes('C'))
        if(time_ms<self.timeRange_ms[0] or time_ms>self.timeRange_ms[1]):
            raise ValueError("Servo: Time out of range")
        # 低位在前,高位在后
        time_lh=np.uint16(time_ms).tobytes('C')
        data_frame.extend(time_lh)
        for angle in angles:
            id=angle[0]
            if(id<0 or id>self.Maximum_idNum):
                raise ValueError("Servo: id out of range")
            id=np.uint8(id)
            servo_angle=angle[1]
            if(servo_angle<self.Angle_Range[0] or servo_angle>self.Angle_Range[1]):
                raise ValueError("Servo: angle out of range")
            angle_lh=np.uint16(servo_angle).tobytes('C')
            data_frame.extend(id.tobytes('C'))
            data_frame.extend(angle_lh)
        # print(data_frame.hex(' '))
        # 发送控制指令
        self.uartPort.write(data_frame)
        # 舵机角度范围限制
        # angles=[max(self.Angle_Range[0],min(self.Angle_Range[1],angle)) for angle in angles]

    def Run_ActionGroup(self,action_grp_code:np.uint8,num_runs:np.uint16=1):
        """
        @功能: 执行动作组
        @参数: action_grp_code: 动作组代码
        @参数: num_runs: 动作组运行次数,默认为1次,若赋0,则一直运行
        @数据帧格式:
        frame_header(2B) + data_length(==5) + reg_add + action_grp_code + 
        num_runs_l + num_runs_h
        """
        reg_add=np.uint8(0x06)
        data_length=np.uint8(5)
        grp_code=action_grp_code
        if(grp_code<0 or grp_code>self.Maximum_ActionGroup_Code):
            raise ValueError("Action group code out of range")
        grp_code=np.uint8(grp_code)
        # 低位在前,高位在后
        num_runs_lh=np.uint16(num_runs).tobytes('C')
        data_frame=bytearray()
        data_frame.extend(self.dataFrame_Header)
        data_frame.extend(data_length.tobytes('C'))
        data_frame.extend(reg_add.tobytes('C'))
        data_frame.extend(grp_code.tobytes('C'))
        data_frame.extend(num_runs_lh)
        # print(data_frame.hex(' '))
        # 发送控制指令
        self.uartPort.write(data_frame)

class myManipulator:
    def __init__(self,arm_params_list:List[Tuple],logger:Logger,servo:Servo,
                 type:str="3R_Articulated",name:str="myManipulator"):
        """
        @功能: 创建机械臂对象,存储关节长度、执行器偏移量等参数
        @参数: arm_params_list: 机械臂参数列表, 格式为[(link_lengths:l0,l1,l2...),
        (actuator_offsets:x4,y4,z4)]
        @参数: logger: 日志记录器
        @参数: servo: 舵机对象
        @参数: type: 机械臂类型
        @参数: name: 机械臂名称
        """
        self.Name=name
        self.Type=type
        self.Logger=logger
        self.Link_lengths=[]
        self.Actuator_offsets=None
        # 关节角限制(theta)
        self.Joint_Angle_limit=None
        # joint space->actuator space参数
        self.Joint_to_Actuator_Matrix=[]
        # 舵机对象,目前只支持使用同一种舵机
        self.Servo=servo
        # 记录当前关节角,初始全部赋0,可能有bug
        self.Current_JointAngles=np.zeros(3)
        # 存储直线轨迹插补模式下产生的[当前工作空间坐标,距离初始点距离]
        self.Linear_Interpolation_Storager=[np.zeros(3),float(0)]
        # 直线轨迹插补参数[总距离,单位向量,步长,间隔时间]
        self.Linear_Interpolation_Params=[float(0),np.array([0,0,0]),float(0),float(0)]
        # 状态标志变量,用于判断机械臂工作状态
        self.Status_Flag=np.uint8(0)
        # 等待结束时间,用于等待机械臂就位
        self.End_Wait_Time=0
        # yaw轴加减速参数:[缓冲角度/°,加减速时间/ms,缓冲角度比率,加减速时间比率,目标yaw轴角]
        self.Yaw_AccParam=[float(0),np.uint16(0),float(0),float(0),float(0)]
        # 夹爪张开/闭合角度(舵机微分值)
        self.Claw_Angles=None
        try:
            link_lengths=arm_params_list[0]
            actuator_offsets=arm_params_list[1]
            for length in link_lengths:
                if length<=0:
                    raise ValueError("Link length should be positive")
                self.Link_lengths.append(length)
            if(len(actuator_offsets)!=3):
                raise ValueError("Actuator offset should be 3 dimensional")
            self.Actuator_offsets=actuator_offsets
            logger.info("Manipulator: link_lengths:{}; actuator_offsets:{}"
                         .format(link_lengths,actuator_offsets))
        except Exception:
            logger.error("Invalid params for {}({} manipulator)".format(name,type),exc_info=True)
            return None

    def Set_Claw_Angles(self,open_closed_angles:Tuple[np.uint16,np.uint16]):
        """
        @功能: 设置夹爪张开/闭合角度(舵机微分值)
        """
        self.Claw_Angles=open_closed_angles

    def Store_Point_and_Distance(self,point:np.ndarray,distance:float):
        """
        @功能: 存储中间点和距离初始点距离,用于直线插补
        @参数: point: 点坐标
        @参数: distance: 距离初始点距离
        """
        if(len(point)!=3):
            raise ValueError("Point should be 3 dimensional")
        self.Linear_Interpolation_Storager[0]=point
        self.Linear_Interpolation_Storager[1]=distance
    
    def Get_Point_and_Distance(self)->List:
        """
        @返回: [点,距离]
        """
        return self.Linear_Interpolation_Storager

    def Store_Linear_Interpolation_Params(self,total_distance:float,unit_vector:np.ndarray,
                                          step_length:float,gap_time_s:float):
        self.Linear_Interpolation_Params[0]=float(total_distance)
        if(len(unit_vector)!=3):
            raise ValueError("Unit vector should be 3 dimensional")
        self.Linear_Interpolation_Params[1]=unit_vector
        self.Linear_Interpolation_Params[2]=float(step_length)
        self.Linear_Interpolation_Params[3]=float(gap_time_s)

    def Get_Linear_Interpolation_Params(self)->List:
        """
        @返回: [总距离/mm,单位向量,步长/mm,间隔时间/s]
        """
        return self.Linear_Interpolation_Params

    def Store_Target_YawAngle(self,theta3:float):
        self.Yaw_AccParam[4]=float(theta3)

    def Get_Target_YawAngle(self)->float:
        return self.Yaw_AccParam[4]

    def Set_YawAccRatio(self,angle_ratio:float,time_ratio:float):
        """
        @功能: 设置yaw轴加减速参数
        @参数: acc_angle: 缓冲角度/°
        @参数: angle_ratio: 缓冲角度比率
        @参数: time_ratio: 加减速时间比率
        """
        self.Yaw_AccParam[2]=float(angle_ratio)
        self.Yaw_AccParam[3]=float(time_ratio)

    def Get_YawAccRatio(self)->Tuple[float,float]:
        """
        @功能: 获取yaw轴加减速参数
        @返回值: (angle_ratio,time_ratio)
        """
        return (self.Yaw_AccParam[2],self.Yaw_AccParam[3])

    def Store_YawAccTime(self,acc_time_ms:np.uint16):
        """
        @功能: 存储yaw轴加减速时间
        @参数: acc_time: 加减速时间/ms
        """
        self.Yaw_AccParam[1]=np.uint16(acc_time_ms)

    def Get_YawAccTime(self)->np.uint16:
        return self.Yaw_AccParam[1]

    def Store_YawAccAngle(self,acc_angle_deg:float):
        """
        @功能: 存储yaw轴缓冲角度
        @参数: acc_angle: 缓冲角度/°
        """
        self.Yaw_AccParam[0]=float(acc_angle_deg)

    def Get_YawAccAngle(self)->float:
        return self.Yaw_AccParam[0]

    def Change_Status(self,status_flag:np.uint8):
        self.Status_Flag=np.uint8(status_flag)

    def Update_Current_jointAngle(self,joint_code:np.uint8,theta:float):
        """
        @功能: 更新当前机械臂关节角
        @参数: theta: 新的角度
        """
        self.Current_JointAngles[joint_code-1]=theta

    def Set_Joint_to_Actuator_Matrix(self,mapping_groups:List[List[List[float]]]):
        """
        @功能: 通过每个关节两个[关节角*,舵机角度值]映射组确定关节角度到舵机角度值的线性映射关系
        @参数: mapping_groups: 映射关系列表, 格式为[[alpha1,servo_angle1],[alpha2,Servo_angle2],...]
        @警告: 1. 这里得出的矩阵反映的是alpha与servo_angle的关系(所以并非严格意义上的joint space->
        actuator space),已知theta,想要求servo_angle,应先通过加减运算求出alpha,
        再通过映射关系求出servo_angle;
        2. 所以上面提到的关节角并非theta而是alpha
        """
        for grp in mapping_groups:
            if(len(grp)!=2):
                raise ValueError("Each mapping group should have 2 elements")
            mapping_grp_t=np.transpose(grp)
            alpha_grp=mapping_grp_t[0]
            servo_angle_grp=mapping_grp_t[1]
            # 求解Ax=b
            A=np.array([[1,alpha_grp[0]],[1,alpha_grp[1]]])
            A_inv=np.linalg.inv(A)
            b=np.array(servo_angle_grp)
            x=A_inv@b
            self.Joint_to_Actuator_Matrix.append(x)
        self.Logger.debug("Joint_to_Actuator_Matrix:{}".format(self.Joint_to_Actuator_Matrix))

    def Set_JointAngle_Limit(self,limits:Tuple[float,float]):
        self.Joint_Angle_limit=limits
        
    def Joint_Angle_Ctrl(self,joint_angles:List[Tuple[np.uint8,float]],time_ms:np.uint16):
        """
        @功能: 机械臂关节角度控制
        @参数: joint_angles: 关节角度列表[(1,theta1),(2,theta2),(3,theta3)]
        @参数: time_ms: 到位时间,单位毫秒
        @注意: 目前只适配了3R_Articulated类型的机械臂
        """
        if(len(joint_angles)>3):
            raise ValueError("Joint angle list have 3 elements at most")
        theta_list=np.zeros((2,3))
        for angle in joint_angles:
            joint_code=angle[0]
            if(joint_code<1 or joint_code>3):
                raise ValueError("Joint angle should be 1,2,3")
            theta=angle[1]
            theta_list[0][joint_code-1]=theta
            theta_list[1][joint_code-1]=1
        # 若没有指定某个theta的值,
        # 如输入joint_angles=[(1,90),(2,60)],则认为theta3无需改变,使用当前值
        # 此举是为了让alpha计算不被未赋值(==0)的theta干扰
        for i in range(3):
            if(theta_list[1][i]==0):
                theta_list[0][i]=self.Current_JointAngles[i]
        theta1,theta2,theta3=theta_list[0]
        alpha3=theta3
        alpha1=theta1
        alpha2=-(theta2+theta1)
        alpha_list=[alpha1,alpha2,alpha3]
        servo_angles=[]
        for i in range(3):
            # 若theta_x未指定,则认为angle_x无需改变
            if(theta_list[1][i]!=0):
                # 更新当前关节角
                joint_code=i+1
                self.Update_Current_jointAngle(joint_code,theta_list[0][i])
                alpha=alpha_list[i]
                x=self.Joint_to_Actuator_Matrix[i]
                # 根据alpha计算servo_angle
                angle=np.uint16(round(x[1]*alpha+x[0]))
                servo_angles.append((joint_code,angle))
        self.Servo.Angle_Ctrl(servo_angles,time_ms)
        self.Logger.debug("(Joint_Angle_Ctrl)joint:{},servo:{}".format(self.Current_JointAngles,servo_angles))

    def Kinematics_3RAtype(self,joint_angles:List[float],use_radian=False):
        """
        @功能: 输入关节角,计算3R机械臂末端(在机械臂基座坐标系下的)空间坐标
        @参数: joint_angles: 关节角度列表[theta1,theta2,theta3]
        @参数: use_radian: 是否输入弧度
        """
        angles_list=[]
        if(use_radian==False):
            # 将输入角度值转为弧度
            # for angle in joint_angles:
            #     angle=radians(angle)
            # 上面的写法有问题,joint_angles中的值无法通过访问angl修改
            for angle in joint_angles:
                angles_list.append(radians(angle))
        else:
            angles_list=joint_angles
        theta1,theta2,theta3=angles_list[:3]
        self.Logger.debug("Kinematics_3RAtype: joint_angles:{}".format(joint_angles))
        p4=np.array(self.Actuator_offsets)
        self.Logger.debug("Kinematics_3RAtype: p4:{}".format(p4))
        l0,l1,l2=self.Link_lengths
        self.Logger.debug("Kinematics_3RAtype: l0:{},l1:{},l2:{}".format(l0,l1,l2))
        p1=np.array([l0+l1*cos(theta1)+l2*cos(theta1+theta2),
                      l1*sin(theta1)+l2*sin(theta1+theta2),
                      0])
        self.Logger.debug("Kinematics_3RAtype: p1:{}".format(p1))
        p1=p1+p4
        self.Logger.debug("Kinematics_3RAtype: p1+p4:{}".format(p1))
        x1,y1,z1=p1
        self.Logger.debug("Kinematics_3RAtype: x1:{},y1:{},z1:{}".format(x1,y1,z1))
        p0=np.array([x1*cos(theta3)+z1*sin(theta3),
                     x1*sin(theta3)-z1*cos(theta3),
                     y1])
        self.Logger.debug("Kinematics_3RAtype: p0:{}".format(p0))
        return p0

    def INverse_Kinematics_3RAtype(self,target_position:Tuple,yaw_or_plane=True,use_radian=False):
        """
        @功能: 计算空间关节式3R机械臂逆运动学解
        @参数: target_position: 目标位置(x,y,z)/mm
        @参数: yaw_or_plane: 只计算yaw角度(true)或平面姿态(false)
            (目的是yaw轴与其它轴解耦,减少特定任务下的计算量)
        @参数: use_radian: 是否使用弧度制
        @返回: (theta1,theta2) 或 theta3
        """
        x0,y0,z0=target_position
        theta3=float(0)
        theta2=float(0)
        theta1=float(0)
        x4,y4,z4=self.Actuator_offsets
        l0,l1,l2=self.Link_lengths

        z1=z4
        x1=math.sqrt(x0**2+y0**2-z1**2)

        if(yaw_or_plane==False):
            y1=z0
            xb=x1-x4-l0
            yb=y1-y4
            c2=0.5*(xb**2+yb**2-(l1**2+l2**2))/(l1*l2)
            # 此处s2_a2取正/负值会导向两个不同的解
            s2_a=-math.sqrt(1-c2**2)
            theta2=math.atan2(s2_a,c2)
            theta1=math.atan2(yb,xb)-math.atan2(l2*s2_a,l1+l2*c2)
            if(use_radian==False):
                theta2=math.degrees(theta2)
                theta1=math.degrees(theta1)
            return (theta1,theta2)
        else:
            theta3=math.atan2(y0,x0)+math.atan2(z1,x1)
            if(use_radian==False):
                theta3=math.degrees(theta3)
            return theta3

    class Ctrl_Mode(Enum):
        """
        @功能: 机械臂控制模式
        """
        POINT_TO_POINT=0
        LINEAR=2
        YAW_ROTATION=3

    def Goto_Target_Pos(self,target_position:Tuple[float,float,float],time_ms:np.uint16,
                        Mode:Ctrl_Mode=Ctrl_Mode.POINT_TO_POINT,
                        speed_mm_s:np.uint16=10)->bool:
        """
        @返回: busu_flag: 忙碌标志,True表示机械臂正在工作,应保持当前任务状态,持续循环到返回false为止,
                False表示机械臂工作结束,可以改变任务状态,执行新任务
        """
        # 忙碌标志变量,由外部判断
        # True表示机械臂正在工作,应保持当前任务状态,持续循环到返回false为止
        # False表示机械臂工作结束,可以改变任务状态,执行新任务
        busy_flag=True
        if(Mode==self.Ctrl_Mode.POINT_TO_POINT):
            if(self.Status_Flag==0):
                self.Change_Status(1)
                # 解算角度并发送运动指令
                theta3=self.INverse_Kinematics_3RAtype(target_position)
                theta1,theta2=self.INverse_Kinematics_3RAtype(target_position,False)
                self.Joint_Angle_Ctrl([(1,theta1),(2,theta2),(3,theta3)],time_ms)
                # 计算等待时间
                self.End_Wait_Time=time.time()+0.001*time_ms
                self.Logger.debug("({}) 开始移动".format(self.Name))
            elif(self.Status_Flag==1):
                # 等待机械臂到位
                if(time.time()>=self.End_Wait_Time):
                    self.Change_Status(0)
                    busy_flag=False
                    self.Logger.debug("({}) 到达目标点:{}".format(self.Name,target_position))
        elif(Mode==self.Ctrl_Mode.LINEAR):
            if(self.Status_Flag==0):
                self.Change_Status(1)
                # 计算p0(当前点),unit_vector(单位向量),D_p1_2_p0(总距离),step_length(步长)
                theta1,theta2,theta3=self.Current_JointAngles
                p0=self.Kinematics_3RAtype([theta1,theta2,theta3])
                vector_pn_2_p0=np.array(target_position)-p0
                D_pn_2_p0=np.linalg.norm(vector_pn_2_p0)
                unit_vector=vector_pn_2_p0/D_pn_2_p0
                gap_time_s=0.001*time_ms
                step_length=speed_mm_s*gap_time_s
                # 存储参数和中间变量
                self.Store_Linear_Interpolation_Params(D_pn_2_p0,unit_vector,step_length,
                                                       gap_time_s)
                self.Store_Point_and_Distance(p0,0)
                self.Logger.debug("({}) 开始直线运动,当前点:{},参数:{}"
                                  .format(self.Name,p0,self.Linear_Interpolation_Params))
            elif(self.Status_Flag==1):
                # 按照步长和方向进行直线运动
                p,d=self.Get_Point_and_Distance()
                D,unit_vector,steplen,gap_time=self.Get_Linear_Interpolation_Params()
                if(D-d<steplen):
                    self.Change_Status(0)
                    p=target_position
                    busy_flag=False
                    self.Logger.debug("({}) 直线运动完成,到达目标点:{}"
                                      .format(self.Name,p))
                else:
                    self.Change_Status(2)
                    p=p+unit_vector*steplen
                    d=d+steplen
                    self.Store_Point_and_Distance(p,d)
                    self.Logger.debug("({}) 正在直线运动,当前点:{},已移动距离:{}"
                                      .format(self.Name,p,d))
                # 对点进行逆解,并控制机械臂关节角
                x,y,z=p
                self.End_Wait_Time=time.time()+gap_time
                theta3=self.INverse_Kinematics_3RAtype((x,y,z))
                theta1,theta2=self.INverse_Kinematics_3RAtype((x,y,z),False)
                self.Joint_Angle_Ctrl([(1,theta1),(2,theta2),(3,theta3)],time_ms)
            elif(self.Status_Flag==2):
                # 等待微分运动完成
                if(time.time()>=self.End_Wait_Time):
                    self.Change_Status(1)
        elif(Mode==self.Ctrl_Mode.YAW_ROTATION):
            if(self.Status_Flag==0):
                self.Change_Status(1)
                angle_ratio,time_ratio=self.Get_YawAccRatio()
                # 按照比率计算加减速时间,并存储
                acc_time=round(time_ratio*time_ms)
                self.Store_YawAccTime(acc_time)
                # 按照比率计算缓冲角度,并存储
                theta3=self.INverse_Kinematics_3RAtype(target_position)
                theta3_0=self.Current_JointAngles[2]
                delta=theta3-theta3_0
                acc_angle=angle_ratio*delta
                # if(delta<0):
                #     acc_angle=-acc_angle
                self.Store_YawAccAngle(acc_angle)
                self.Store_Target_YawAngle(theta3)
                # 计算加速目标角度
                target_angle=theta3_0+acc_angle
                # 计算等待时间
                time_s=0.001*acc_time
                self.End_Wait_Time=time.time()+time_s
                self.Logger.debug("({}) yaw轴开始加速,目标角:{};执行时间:{}"
                                  .format(self.Name,target_angle,time_s))
                # 发送加速指令
                acc_time=max(acc_time,self.Servo.timeRange_ms[0])
                self.Joint_Angle_Ctrl([(3,target_angle)],acc_time)
            elif(self.Status_Flag==1):
                # 等待加速完成
                if(time.time()>=self.End_Wait_Time):
                    self.Change_Status(2)
                    self.Logger.debug("({}) yaw轴加速完成".format(self.Name))
            elif(self.Status_Flag==2):
                self.Change_Status(3)
                # 读取之前计算的缓冲角度和加减速时间
                acc_time=self.Get_YawAccTime()
                acc_angle=self.Get_YawAccAngle()
                theta3=self.Get_Target_YawAngle()
                # 计算匀速运动目标角度,匀速运动时间
                target_angle=theta3-acc_angle
                uniform_motion_time=time_ms-2*acc_time
                # 计算等待时间
                time_s=0.001*uniform_motion_time
                self.End_Wait_Time=time.time()+time_s
                self.Logger.debug("({}) yaw轴开始匀速运动,目标角:{};执行时间:{}"
                                  .format(self.Name,target_angle,time_s))
                # 发送匀速运动指令
                uniform_motion_time=max(uniform_motion_time,self.Servo.timeRange_ms[0])
                self.Joint_Angle_Ctrl([(3,target_angle)],uniform_motion_time)
            elif(self.Status_Flag==3):
                # 等待匀速运动完成
                if(time.time()>=self.End_Wait_Time):
                    self.Change_Status(4)
                    self.Logger.debug("({}) yaw轴匀速运动完成".format(self.Name))
            elif(self.Status_Flag==4):
                self.Change_Status(5)
                # 读取加减速时间
                acc_time=self.Get_YawAccTime()
                theta3=self.Get_Target_YawAngle()
                # 计算等待时间
                time_s=0.001*acc_time
                self.End_Wait_Time=time.time()+time_s
                self.Logger.debug("({}) yaw轴开始减速到,目标角:{};执行时间:{}"
                                  .format(self.Name,theta3,time_s))
                # 发送减速指令
                acc_time=max(acc_time,self.Servo.timeRange_ms[0])
                self.Joint_Angle_Ctrl([(3,theta3)],acc_time)
            elif(self.Status_Flag==5):
                # 等待减速完成
                if(time.time()>=self.End_Wait_Time):
                    self.Change_Status(0)
                    self.Logger.debug("({}) yaw轴减速完成".format(self.Name))
                    busy_flag=False
        return busy_flag

    class ActionGroup(Enum):
        """
        @功能: 动作组对应编号
        @成员格式:(action_code,run_time_ms,action_name)
        """
        SCAN_QRCODE=(1,300,"scan_qrcode")
        MATERIAL_CORRECTION=(2,200,"material_correction")
        HOLD_STUFF=(3,200,"hold_stuff")

    def Run_Preset_Action(self,action:ActionGroup)->bool:
        busy_flag=True
        if(self.Status_Flag==0):
            self.Change_Status(1)
            action_grp_code=action.value[0]
            action_run_time_ms=action.value[1]
            self.Servo.Run_ActionGroup(action_grp_code)
            self.End_Wait_Time=time.time()+0.001*action_run_time_ms
            self.Logger.debug("({}) 执行动作组:{}".format(self.Name,action.value[2]))
        elif(self.Status_Flag==1):
            # 等待机械臂到位
            if(time.time()>=self.End_Wait_Time):
                self.Change_Status(0)
                self.Logger.debug("({}) 动作组执行完毕".format(self.Name))
                busy_flag=False
        return busy_flag

    def Claw_Cmd(self,cmd:bool,inloop_flag=True,time_ms:np.uint16=50):
        """
        @功能: 控制夹爪闭合/张开
        @参数: cmd: True为闭合,False为张开
        @参数: inloop_flag 是否在循环中使用,默认true
        @参数: time_ms 执行时间
        """
        busy_flag=True
        if(self.Status_Flag==0):
            if(inloop_flag==True):
                self.Change_Status(1)
            claw_angle_list:Tuple=self.Claw_Angles
            index=np.uint8(cmd)
            self.Servo.Angle_Ctrl([(4,claw_angle_list[index])],time_ms)
            self.End_Wait_Time=time.time()+0.001*time_ms
        elif(self.Status_Flag==1):
            # 等待夹爪闭合/张开
            if(time.time()>=self.End_Wait_Time):
                self.Change_Status(2)
                if(cmd==True):
                    self.Logger.debug("({}) 夹爪闭合".format(self.Name))
                else:
                    self.Logger.debug("({}) 夹爪张开".format(self.Name))
                self.End_Wait_Time=time.time()+0.05
        elif(self.Status_Flag==2):
            # 额外等待100ms
            if(time.time()>=self.End_Wait_Time):
                busy_flag=False
                self.Change_Status(0)
        return busy_flag


def test_Manipulator():
    myLogger=Logger("myLogger",logging.DEBUG)
     # 创建终端处理器
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.DEBUG)
    file_handler = logging.FileHandler("proj/assets/logs/Man_Test.log")
    file_handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', 
                                  datefmt='%Y-%m-%d %H:%M:%S')
    console_handler.setFormatter(formatter)
    file_handler.setFormatter(formatter)
    myLogger.addHandler(console_handler)
    myLogger.addHandler(file_handler)
    myServo=Servo("/dev/ttyAMA4",9600)
    # myServo.Angle_Ctrl([(9,800),(2,800),(1,500)],800)
    # myServo.Run_ActionGroup(8)
    # myServo.Angle_Ctrl([(1,800)],1000)

    try:
        arm=myManipulator([(65,130,130),(71,-20-1.12,0)],myLogger,myServo)

        # target_pos=(198.2-5,0,-6+50)
        # myLogger.debug("target_pos:{}".format(target_pos))
        # theta3=arm.INverse_Kinematics_3RAtype(target_pos)
        # myLogger.debug("theta3:{}".format(theta3))
        # theta1,theta2=arm.INverse_Kinematics_3RAtype(target_pos,False)
        # myLogger.debug("theta1,theta2:{},{}".format(theta1,theta2))
        # alpha_3=theta3
        # alpha_1=theta1
        # alpha_2=-(theta2+theta1)
        # print("angles(321):",alpha_3,alpha_2,alpha_1)
        # print("thetas(321):",theta3,theta2,theta1)

        # 正运动学测试
        # cali_pos=arm.Kinematics_3RAtype([theta1,theta2,theta3])
        # myLogger.debug("cali_pos:{}".format(cali_pos))

        # 设置joint space->actuator space的映射关系
        arm.Set_Joint_to_Actuator_Matrix([[[90,430],[90-16.8,500]],
                                        [[(180-90),420],[180-(90+19.2),500]],
                                        [[0,500],[120,1000]]])
        print("Joint_to_Actuator_Matrix:",arm.Joint_to_Actuator_Matrix)

        # arm.Joint_Angle_Ctrl([(1,theta1),(2,theta2),(3,theta3)],100)
        # arm.Joint_Angle_Ctrl([(1,theta1),(2,theta2)],100)
        # arm.Joint_Angle_Ctrl([(3,theta3)],1000)
        # arm.Run_Preset_Action(myManipulator.ActionGroup.GRP1)
        # arm.Claw_Cmd(False)

        arm.Update_Current_jointAngle(1,119.23159548)
        arm.Update_Current_jointAngle(2,-141.05393609)
        arm.Update_Current_jointAngle(3,0)

        arm.Set_YawAccRatio(0.2,0.25)
        # target_pos=(0,-100,-6+50)
        target_pos=(198.2-5,100,44)

        # 逆运动学控制测试
        while(True):
            busy_flag=arm.Goto_Target_Pos(target_pos,50,arm.Ctrl_Mode.LINEAR,200)
            if(busy_flag==False):
                break
    except Exception:
        myLogger.error("ERROR",exc_info=True)

def test_kinematics(angles:Tuple[float])->Tuple[float]:
    angle_1=angles[0]
    angle_2=angles[1]
    x2=65+130*(math.cos(math.radians(angle_1))+math.cos(math.radians(angle_2)))
    y2=130*(math.sin(math.radians(angle_1))+math.sin(math.radians(angle_2)))
    return x2,y2

def Matrix_test():
    # 可通过两组数据求解关节角与舵机角度微分值的关系(求解出0次项系数x0,1次项系数x1)
    theta1=90

    # 输入参数
    mapping_group=[[90,430],[90-16.8,500]]
    mapping_grp_t=np.transpose(mapping_group)
    theta_grp=mapping_grp_t[0]
    alpha_grp=mapping_grp_t[1]
    # 求解Ax=b
    A=np.array([[1,theta_grp[0]],[1,theta_grp[1]]])
    A_inv=np.linalg.inv(A)
    b=np.array(alpha_grp)
    x=A_inv@b
    print(x)

    # alpha1=x1*theta1+x0
    alpha1=np.uint16(round(x[1]*theta1+x[0]))
    print(alpha1)

    # a=np.array([[1,0],[0,2]])
    # # b=np.array([1,2])
    # b=np.array([[1],[2]])
    # print(a@b)

if __name__=="__main__":
    test_Manipulator()
    # print(test_kinematics((-27.78,120.52)))
    # Matrix_test()