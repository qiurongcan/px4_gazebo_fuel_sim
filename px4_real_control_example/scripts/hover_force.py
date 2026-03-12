#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simple BodyRate + Thrust Hover Example (Python Version)
功能: 持续发送 0 角速度和固定推力指令。
注意: 此版本去除了自动解锁和自动切 OFFBOARD 的功能，需要手动操作。
"""

import rospy
from mavros_msgs.msg import State, AttitudeTarget

# 全局变量存储当前状态
current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    # 1. 初始化节点
    rospy.init_node('simple_rate_hover_python', anonymous=True)

    # 2. 订阅状态 (用于检查连接)
    rospy.Subscriber("mavros/state", State, state_cb)

    # 3. 发布控制指令 (核心话题)
    # 话题: mavros/setpoint_raw/attitude
    # 消息类型: mavros_msgs/AttitudeTarget
    local_att_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

    # 设置循环频率 (必须 > 2Hz，推荐 20Hz 以上)
    rate = rospy.Rate(20.0)

    # 等待 MAVROS 连接到飞控
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
        rospy.loginfo("Waiting for FCU connection...")

    rospy.loginfo("FCU connected!")

    # ==========================================
    # 4. 构建控制指令 (核心部分)
    # ==========================================
    hover_cmd = AttitudeTarget()
    hover_cmd.header.frame_id = "body"
    
    # type_mask 是关键！
    # 128 (二进制 10000000) 代表 "IGNORE_ATTITUDE" (忽略姿态四元数)
    # 这告诉 PX4：忽略 orientation 字段，使用 body_rate 字段
    hover_cmd.type_mask = 128

    # 设置目标角速度 (rad/s)
    # 0.0 代表保持当前角度不旋转 (类似于自稳模式的手感，但不自动回平)
    hover_cmd.body_rate.x = 0.0 # Roll rate
    hover_cmd.body_rate.y = 0.0 # Pitch rate
    hover_cmd.body_rate.z = 0.0 # Yaw rate

    # 设置推力 (0.0 ~ 1.0)
    # 警告：请根据你的机型调整此值。
    # Gazebo Iris 约为 0.55 - 0.71 (取决于载重)
    # 真机请从 0.1 开始测试！
    # [0.755 0.757]
    hover_cmd.thrust = 0.756

    # ==========================================
    
    rospy.loginfo("Sending setpoints... Ready for manual OFFBOARD switch.")

    while not rospy.is_shutdown():
        # 更新时间戳
        hover_cmd.header.stamp = rospy.Time.now()
        
        # 持续发布指令
        # 即使你现在处于 Manual/Stabilized 模式，这些指令也会被发送，
        # 但只有当你切入 OFFBOARD 模式时，PX4 才会执行它们。
        local_att_pub.publish(hover_cmd)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
