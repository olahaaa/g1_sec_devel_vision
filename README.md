# 前提 

## 配置G1 ros2环境
参考https://github.com/unitreerobotics/unitree_ros2.git 安装在`$HOME`下
添加G1环境脚本 setup.bash

## 安装realsensesdk2 和 realsense_ros包
参考 https://github.com/realsenseai/realsense-ros.git 和 https://github.com/realsenseai/librealsense.git

## 安装apriltag识别包
参考 https://github.com/christianrauch/apriltag_ros.git


## /src
img_sub_node 静态图像订阅测试
tfsub_node 订阅tf变化测试

## /launch
aptag_recog_demo.launch.py 摄像头测试
img_show_demo.lanch.py 图像订阅测试