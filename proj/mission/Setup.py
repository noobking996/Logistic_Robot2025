import os
# import sys
# import subprocess

def setup():
    # subprocess.call(['chmod', '+x', '/home/zhang/Logistic_Robot2025/proj/mission/run.sh'])
    # os.system('chmod +x /home/zhang/Logistic_Robot2025/proj/mission/run.sh')

    pass

class Mission_Typedef():
    def __init__(self, mission_flag_num, mission_func, mission_status, mission_start_time, mission_end_time):
        # 状态机任务唯一id,不可重复，从0开始,按照任务的先后顺序排列
        self.flag_num = mission_flag_num

class Mission_List():
    def __init__(self):
        self.mission_list = []

    def add_mission(self, mission):
        self.mission_list.append(mission)

    def get_mission_list(self):
        return self.mission_list

def Mission_Init():
    pass