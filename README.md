<!--
 * @@Descripttion: 
 * @@version: 
 * @@encoding: utf-8
 * @@Author: qiurongcan
 * @Date: 2025-12-04 16:23:54
 * @LastEditTime: 2025-12-04 16:45:29
-->
# px4_gazebo_fuel_sim

本仓库基于PX4和gazebo的仿真环境下，使用fastlio定位以及fuel未知环境自主探索路径算法

## 仿真环境

| 名称     | 具体版本                                   |
| :------- | :----------------------------------------- |
| 操作系统 | Ubuntu20.04                                |
| ROS      | ROS1 Noetic                                |
| Gazebo   | Gazebo11                                   |
| PX4      | v1.14.0(这个版本不影响，配置上有略微区别） |
| 传感器   | Mid360                                     |
| 定位算法 | Fast LIO2                                  |
| 搜索算法 | FUEL                                       |

## PX4+MID360配置

参考仓库 `https://github.com/qiurongcan/Mid360_px4.git`中的配置方法进行配置，详细阅读

## FAST LIO2配置

## FASTLIO为PX4无人机提供定位源
这个节点的作用是转化FastLIO输出的`/Odometry`话题为`/mavros/vision_pose/pose`，给px4提供无GPS情况下的定位  
### QGC参数设置
**v1.14.0（包括）之后的版本**  
|参数|设置|备注|
|:---|:---|:---|
|EKF2_HGT_REF|Vision||
|EKF2_GPS_CTRL| 0 |有GPS的情况下设置为0|
|EKF2_EV_CTRL|15|

**如下图所示**
![set2](./figs/set1.png)

**v1.14.0（不包括)之前的版本**
|参数|设置|
|:---|:---|
|EKF2_HGT_MODE|Vision|
|EKF2_AID_MASK|24|
```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
# 将仓库中的camera_pose_node这个文件夹复制到 ~/catkin_ws/src 目录下
catkin_make
source devel/setup.bash
```
```shell
# 打开一个新的终端
roslaunch camera_pose_node pose_tf.launch
```
## 无人机自动起飞（仿真环境）

## FUEL配置
