<!--
 * @@Descripttion: 
 * @@version: 
 * @@encoding: utf-8
 * @@Author: qiurongcan
 * @Date: 2025-12-04 16:23:54
 * @LastEditTime: 2026-03-20 18:29:26
-->

# px4_gazebo_fuel_sim

|分支|说明|备注|
|---|---|---|
|main|主要的仿真分支||
|real_exp|实机实验分支代码||
|old_stable_version|古早的稳定版分支实机和仿真||

本仓库基于PX4和gazebo的仿真环境下，使用fastlio定位以及fuel未知环境自主探索路径算法
```shell
实验流程:
1 启动px4仿真环境
2 启动Fast LIO定位
3 FastLIO定位 转 Px4
4 启动se3控制器
5 启动 fuel 脚本
6 [操作] 手动/脚本 切换OFFBOARD模式 无人机自动起飞
7 [操作] rviz选点 / rostopic 发布move_base
```


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

## 安装FUEL之前需要下载这个
```shell
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```

**需要安装的库**

```shell
sudo apt-get install libdw-dev
```

## 1 PX4+MID360配置

参考仓库 `https://github.com/qiurongcan/Mid360_px4.git`中的配置方法进行配置，详细阅读

**配置完成后**

```shell
# 这里默认无人机型号在mavros_posix_sitl.launch文件中替换成了iris_mid360
roslaunch px4 mavrox_posix_sitl.launch
```

## 2 FAST LIO2配置

可以参考Fast LIO原仓库 `https://github.com/hku-mars/FAST_LIO.git`进行配置，但是编译会有些报错，自己自行解决即可

如果觉得麻烦可以直接复制本仓库中的 `FASTLIO`到工作空间下，我这个修复了一些报错

```shell
mkdir -p ~/catkin_ws/src
# 目前设定是已经将FASTLIO复制到 ~/catkin_ws/src 目录下
cd ~/catkin_ws
catkin_make
source ./devel/setup.bash
```

```shell
# 如果在真机部署，或者觉得太卡了，在mapping_mid360.launch中将rviz设置为false
roslaunch fast_lio mapping_mid360.launch
```

## 2 FASTLIO为PX4无人机提供定位源

这个节点的作用是转化FastLIO输出的 `/Odometry`话题为 `/mavros/vision_pose/pose`，给px4提供无GPS情况下的定位

### QGC参数设置

**v1.14.0（包括）之后的版本**  

| 参数          | 设置   | 备注                 |
| :------------ | :----- | :------------------- |
| EKF2_HGT_REF  | Vision |                      |
| EKF2_GPS_CTRL | 0      | 有GPS的情况下设置为0 |
| EKF2_EV_CTRL  | 15     |                      |

**如下图所示**
![set2](./figs/set1.png)

**v1.14.0（不包括)之前的版本**

| 参数          | 设置   |
| :------------ | :----- |
| EKF2_HGT_MODE | Vision |
| EKF2_AID_MASK | 24     |

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

## 3 启动控制器
将仓库中的`controller`复制到`~/catkin_ws/src`目录下
```shell
# 复制以后
cd ~/catkin_ws/
catkin_make
# 执行起飞脚本
. devel/setup.bash
roslaunch se3_controller sitl_se3_controller.launch
```

**注意！注意！注意！** 做到这一步代码可以先测试一下
```shell
cd /path/to/controller/tools/
python3 mode_switch.py OFFBOARD
```
此时会看到无人机自动起飞， 说明定位没有问题和控制器也没有问题


## 4FUEL配置

将这个仓库中的 `FUEL` 文件夹复制到 `~/catkin_ws/src/`目录下

```shell
cd ~/catkin_ws/
catkin_make
```

**运行代码**

```shell
# 新建第一个终端 运行后出现一个rviz
roslaunch exploration_manger exploration.launch
```
之后在弹出的RVIZ中，使用2D Nav Goal 选择一个目标。程序就开始自主探索了

## 致谢
