#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
BodyRate Position Hold Example
功能: 通过计算位置误差，生成角速度和推力指令，使无人机在 (0, 0, 2.5) 悬停。
核心: 串级 PID 控制 (位置环 -> 角度环 -> 角速度输出)
"""

import rospy
import math
from mavros_msgs.msg import State, AttitudeTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion

# ==========================================
# 1. 参数配置 (PID Gains & Setpoints)
# ==========================================
TARGET_X = 0.0
TARGET_Y = 0.0
TARGET_Z = 2.5
TARGET_YAW = 0.0

# 基础悬停油门 (你测出来的数值)
HOVER_THRUST = 0.756 

# 位置环 PID 参数 (Position -> Velocity/Angle)
Kp_pos_xy = 1.0   # 水平位置比例系数
Kd_pos_xy = 0.6   # 水平速度阻尼系数
Kp_z      = 1.5   # 高度比例系数
Kd_z      = 0.8   # 高度速度阻尼系数

# 姿态环 P 参数 (Angle -> BodyRate)
Kp_att    = 6.0   # 角度转角速度的系数

# 限制幅度 (安全保护)
MAX_TILT_ANGLE = math.radians(20) # 最大倾斜 20 度
MAX_THRUST = 0.9
MIN_THRUST = 0.1

# ==========================================
# 2. 全局状态变量
# ==========================================
current_state = State()
local_pose = PoseStamped()
local_vel = TwistStamped()

# 欧拉角 (Roll, Pitch, Yaw)
curr_roll = 0.0
curr_pitch = 0.0
curr_yaw = 0.0

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global local_pose, curr_roll, curr_pitch, curr_yaw
    local_pose = msg
    # 将四元数转换为欧拉角，方便后面计算误差
    q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    (curr_roll, curr_pitch, curr_yaw) = euler_from_quaternion(q)

def vel_cb(msg):
    global local_vel
    local_vel = msg

def clamp(value, min_val, max_val):
    return max(min(value, max_val), min_val)

def main():
    rospy.init_node('bodyrate_position_hold', anonymous=True)

    # 订阅
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, vel_cb)

    # 发布
    local_att_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

    rate = rospy.Rate(30.0) # 提高频率到 30Hz 以获得更好的控制效果

    # 等待连接
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
        rospy.loginfo("Waiting for FCU connection...")

    rospy.loginfo("FCU Connected! Ready for OFFBOARD.")

    # 构建消息
    att_cmd = AttitudeTarget()
    att_cmd.header.frame_id = "body"
    att_cmd.type_mask = 128 # 忽略姿态(Orientation)，只使用角速度(BodyRate)

    while not rospy.is_shutdown():
        # 0. 获取当前状态
        curr_x = local_pose.pose.position.x
        curr_y = local_pose.pose.position.y
        curr_z = local_pose.pose.position.z
        
        curr_vx = local_vel.twist.linear.x
        curr_vy = local_vel.twist.linear.y
        curr_vz = local_vel.twist.linear.z

        # ========================================
        # 1. 高度控制 (Z轴 PID -> 推力)
        # ========================================
        err_z = TARGET_Z - curr_z
        err_vz = 0.0 - curr_vz  # 期望垂直速度为 0
        
        # 计算需要的额外推力增量
        thrust_correction = (Kp_z * err_z) + (Kd_z * err_vz)
        
        # 最终推力 = 基础悬停油门 + 修正量 (并做归一化映射，这里简化处理)
        # 注意：这里的系数 0.05 是为了将位置误差转换为 0-1 的推力调整量，需要根据实际情况微调
        desired_thrust = HOVER_THRUST + (thrust_correction * 0.05)
        desired_thrust = clamp(desired_thrust, MIN_THRUST, MAX_THRUST)

        # ========================================
        # 2. 水平位置控制 (XY轴 PID -> 期望加速度 -> 期望倾角)
        # ========================================
        err_x = TARGET_X - curr_x
        err_y = TARGET_Y - curr_y
        err_vx = 0.0 - curr_vx
        err_vy = 0.0 - curr_vy

        # 计算期望加速度 (P控制器)
        des_acc_x = (Kp_pos_xy * err_x) + (Kd_pos_xy * err_vx)
        des_acc_y = (Kp_pos_xy * err_y) + (Kd_pos_xy * err_vy)

        # 将加速度映射为期望倾斜角度 (小角度近似: a = g * tan(theta) ≈ g * theta)
        # 坐标系: MAVROS ENU (前是X, 左是Y)
        # 向前加速(+X) 需要低头 (Pitch负)
        # 向左加速(+Y) 需要左倾 (Roll负)
        des_pitch = -des_acc_x / 9.8 
        des_roll  = -des_acc_y / 9.8  # 注意: ENU坐标系下，向左倾斜是 Roll 负值吗？
                                      # 修正: 右手定则，X向前，Y向左，Z向上。
                                      # 绕X轴旋转(Roll): 右倾为正，左倾为负。
                                      # 要向左加速(+Y)，需要机身向左倾斜，即 Roll 为正? 
                                      # 不，通常向左飞是 Roll 负 (Left wing down)。
                                      # 让我们用简单逻辑: Error Y > 0 (目标在左边) -> 需要向左飞 -> Roll 应该倾斜。
                                      # 实际调试中如果方向反了，把这里的符号反一下即可。
                                      # 标准 ENU: Roll Positive = Right Wing Down (向右飞).
                                      # 所以要向左飞(+Y), 需要 Roll Negative. 
                                      # 公式: des_roll = -des_acc_y / 9.8 是对的 (正误差导致负Roll).

        # 限制最大倾角
        des_roll = clamp(des_roll, -MAX_TILT_ANGLE, MAX_TILT_ANGLE)
        des_pitch = clamp(des_pitch, -MAX_TILT_ANGLE, MAX_TILT_ANGLE)

        # 偏航角控制 (简单的 P 控制)
        err_yaw = TARGET_YAW - curr_yaw
        # 处理 -pi 到 pi 的跳变
        if err_yaw > math.pi: err_yaw -= 2*math.pi
        if err_yaw < -math.pi: err_yaw += 2*math.pi
        des_yaw_rate = 1.0 * err_yaw

        # ========================================
        # 3. 姿态控制 (角度误差 -> 角速度)
        # ========================================
        # 我们现在有了期望角度 (des_roll, des_pitch)，需要计算该以多快速度转过去
        
        target_roll_rate = Kp_att * (des_roll - curr_roll)
        target_pitch_rate = Kp_att * (des_pitch - curr_pitch)
        target_yaw_rate = clamp(des_yaw_rate, -1.0, 1.0)

        # ========================================
        # 4. 发送指令
        # ========================================
        att_cmd.header.stamp = rospy.Time.now()
        att_cmd.body_rate.x = target_roll_rate
        att_cmd.body_rate.y = target_pitch_rate
        att_cmd.body_rate.z = target_yaw_rate
        att_cmd.thrust = desired_thrust

        local_att_pub.publish(att_cmd)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
