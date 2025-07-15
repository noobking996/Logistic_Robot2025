#!/bin/bash

# 设置自定义模块路径
export PYTHONPATH="/home/zhang/Logistic_Robot2025"
echo ${PYTHONPATH}

# 设置python script工作目录
cd /home/zhang/Logistic_Robot2025

# 运行视觉文件
sudo /home/zhang/miniconda3/envs/computer_vision/bin/python3.8 proj/Generic_Mission.py
cd -
