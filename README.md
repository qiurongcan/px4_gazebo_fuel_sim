<!--
 * @@Descripttion: 
 * @@version: 
 * @@encoding: utf-8
 * @@Author: qiurongcan
 * @Date: 2025-12-04 16:23:54
 * @LastEditTime: 2025-12-09 18:05:21
-->

# px4_gazebo_fuel_sim

本仓库基于PX4和gazebo的仿真环境下，使用fastlio定位以及fuel未知环境自主探索路径算法
```shell
实验流程:
启动px4仿真环境 ---> 启动Fast LIO定位 ---> 启动无人机起飞脚本 ---> 使用FUEL自动探索脚本
```
## 特别说明
如果想要做实机实验的话，流程大概如下
1. 先给无人机上电，启动`roslaunch mavros px4.launch`
2. 启动Fastlio，`roslaunch fast_lio mapping_mid360.launch`
3. 将fastlio的定位信息传给px4无人机`/mavros/vision_pose/pose`，需要用转化节点`roslaunch camera_pose_node pose_tf.launch`
4. 这个时候手动起飞无人机，先遥控器切换到position模式，然后遥控器解锁，遥控器操作无人机起飞并悬停
5. 无人机起飞后，执行fuel的代码，`roslaunch exploration_manger exploration_manger.launch`,出现一个rviz界面，这个时候先不要动
6. 执行控制脚本`rosrun exploration_manger fuel_nav`
7. 前面这些操作没有问题后，遥控器切换`OFFBOARD`模式，这个时候无人机会悬停在1m左右的高度
8. 然后在rviz中选点，然后无人机就会开始自主探索了

**注意！！！**
1. 一定要有一个人手上拿着遥控器，如果要出现问题了，快速切换OFFBOARD模式，然后降落
2. 或者及时断电，一定注意安全
3. 注意如果遥控器切换position模式后没办法解锁，应该是定位出问题了。


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

**配置完成后**

```shell
# 这里默认无人机型号在mavros_posix_sitl.launch文件中替换成了iris_mid360
roslaunch px4 mavrox_posix_sitl.launch
```

## FAST LIO2配置

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

## FASTLIO为PX4无人机提供定位源

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

## 无人机自动起飞（仿真环境）
将仓库中的`px4_real_control_examle`复制到`~/catkin_ws/src`目录下
```shell
cd ~/catkin_ws/src/px4_real_control_example/scripts/
# 执行起飞脚本
python3 takeoff_and_park_vio.py
# 等待一段时间，无人机自动起飞，这个时候不用管，运行FUEL代码这部分代码就会自动失效
```
**注意！注意！注意！等无人机起飞到指定位置再执行下一步代码。这个代码是模拟实机实验时手动起飞无人机**
## FUEL配置

将这个仓库中的 `FUEL` 文件夹复制到 `~/catkin_ws/src/`目录下

```shell
cd ~/catkin_ws/
catkin_make
```

**运行代码**

```shell
# 新建第一个终端 运行后出现一个rviz
roslaunch exploration_manger exploration.launch

# 新建第二个终端
rosrun exploration_manger fuel_nav
```
之后在弹出的RVIZ中，使用2D Nav Goal 选择一个目标。程序就开始自主探索了

## 致谢
