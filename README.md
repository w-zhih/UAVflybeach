# ROS开发记录

>  塔克R10系列机器人，ROS基础部分开发说明文档

## ROS版本记录
> 软件开发版本记录  

v1.0版本（20220505）-----------------------------------------------

1. 安装ROS Noetic版本
2. xtark_robot底盘驱动程序开发
3. xtark_bringup启动功能包开发
4. xtark_calibrate功能包开发，开发调试使用
5. xtark_description机器人URDF功能包开发
6. xtark_teleop远程控制功能包开发，目前支持手柄键盘控制
7. xtark_slam建图功能包开发
8. xtark_nav导航功能包开发
9. xtark_package 第三方源码安装功能包
10. xtark_doc 机器人说明文档
11. 建图导航功能包为二进制安装
12. ros_astra_camera 功能包修改了深度相机TF文件astra_frames_xtark.launch
    增加xtark_astra.launch启动文件
13. 思岚A1雷达修改了雷达方向，0度位置旋转了180度，头部在前方
14. 蓝海广电雷达，修改了输出信息
15. 增加xtark_doc文件夹，作为机器人ROS说明文件


待更新---------------------------

1. 待修改蓝海雷达，增加启动时启动雷达旋转，关闭时关闭雷达旋转
2. 文档待更新

## 环境变量 
> 机器人端bashrc文件配置  

R10系列机器人bashrc环境变量配置

```bash
#-------------XTARK ROBOT CONFIG--------------------------------------
# ROS 环境变量配置
source /opt/ros/noetic/setup.bash
source ~/xtark/ros_ws/devel/setup.bash
source ~/xtark/cartographer_ws/install_isolated/setup.bash

# 机器人型号，麦轮mec/四差速fwd/二差速twd/阿克曼akm/三轮全向omni/履带tak 
export ROBOT_TYPE=omni

# 雷达型号，思岚A1：rplidar_a1  塔克TOF雷达:xtlidar_t1
export LIDAR_TYPE=xtlidar_t1

# ROS 网络配置
interface=wlan0
export ROS_IP=`ifconfig $interface | grep -o 'inet [^ ]*' | cut -d " " -f2`
export ROS_MASTER_URI=http://$ROS_IP:11311

# alias 跳转快捷指令
alias cw='cd ~/xtark/ros_ws'
alias cs='cd ~/xtark/ros_ws/src'
alias cm='cd ~/xtark/ros_ws && catkin_make'

# BASH终端显示
echo ""
echo "-------------------- XTARK ROBOT INFO --------------------"
echo -e "  ROBOT_TYPE: \033[32mxtark-r10-$ROBOT_TYPE\033[0m  LIDAR_TYPE: \033[32m$LIDAR_TYPE\033[0m"
echo -e "  ROS_MASTER_URI: \033[32m$ROS_MASTER_URI\033[0m"
echo "----------------------------------------------------------"
echo ""

#-------------XTARK ROBOT CONFIG--------------------------------------
```

