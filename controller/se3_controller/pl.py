#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mavros_msgs.msg import AttitudeTarget, VFR_HUD

class ThrustVisualizer:
    def __init__(self):
        # 初始化节点
        rospy.init_node('thrust_visualizer_node', anonymous=True)

        # --- 配置参数 ---
        self.window_size = 3000  # 窗口大小：图表中最多显示多少个数据点
        self.start_time = rospy.Time.now().to_sec()

        # --- 数据存储 (用于绘图) ---
        # 1. Attitude Target 数据
        self.time_att = []
        self.thrust_att = []
        
        # 2. VFR HUD 数据
        self.time_vfr = []
        self.throttle_vfr = []

        # --- 订阅话题 ---
        # 订阅姿态设定值 (期望推力)
        self.sub_att = rospy.Subscriber(
            "/mavros/setpoint_raw/attitude", 
            AttitudeTarget, 
            self.callback_attitude
        )

        # 订阅 VFR HUD (当前油门/推力状态)
        self.sub_vfr = rospy.Subscriber(
            "/mavros/vfr_hud", 
            VFR_HUD, 
            self.callback_vfr
        )

        # --- 初始化绘图 ---
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
        self.fig.suptitle('Real-time Thrust/Throttle Visualization')

        # 子图1配置
        self.line_att, = self.ax1.plot([], [], 'r-', label='Setpoint Thrust')
        self.ax1.set_ylabel('Thrust (0.0 - 1.0)')
        self.ax1.set_title('/mavros/setpoint_raw/attitude')
        self.ax1.grid(True)
        self.ax1.legend(loc='upper right')
        self.ax1.set_ylim(0.5, 0.9) # 假设推力是归一化的

        # 子图2配置
        self.line_vfr, = self.ax2.plot([], [], 'b-', label='VFR Throttle')
        self.ax2.set_ylabel('Throttle')
        self.ax2.set_xlabel('Time (seconds since start)')
        self.ax2.set_title('/mavros/vfr_hud')
        self.ax2.grid(True)
        self.ax2.set_ylim(0.3, 0.9) # 假设推力是归一化的
        self.ax2.legend(loc='upper right')
        # 如果 VFR HUD 是 0-100，这里可能需要调整 ylim，或者让它自动缩放
        self.ax2.set_autoscale_on(True) 

    def callback_attitude(self, msg):
        """处理 AttitudeTarget 消息"""
        current_time = rospy.Time.now().to_sec() - self.start_time
        val = msg.thrust

        self.time_att.append(current_time)
        self.thrust_att.append(val)

        # 限制数据长度，保持滑动窗口
        if len(self.time_att) > self.window_size:
            self.time_att.pop(0)
            self.thrust_att.pop(0)

    def callback_vfr(self, msg):
        """处理 Vfr_Hud 消息"""
        current_time = rospy.Time.now().to_sec() - self.start_time
        val = msg.throttle

        self.time_vfr.append(current_time)
        self.throttle_vfr.append(val)

        # 限制数据长度
        if len(self.time_vfr) > self.window_size:
            self.time_vfr.pop(0)
            self.throttle_vfr.pop(0)

    def update_plot(self, frame):
        """动画更新函数，由 matplotlib 定时调用"""
        
        # 更新子图1的数据
        if self.time_att:
            self.line_att.set_data(self.time_att, self.thrust_att)
            self.ax1.set_xlim(max(0, self.time_att[-1] - 10), self.time_att[-1] + 1) # X轴显示最近10秒

        # 更新子图2的数据
        if self.time_vfr:
            self.line_vfr.set_data(self.time_vfr, self.throttle_vfr)
            self.ax2.set_xlim(max(0, self.time_vfr[-1] - 10), self.time_vfr[-1] + 1) # X轴显示最近10秒
            
            # 动态调整 Y 轴 (因为 VFR throttle 可能是 0-1 也可能是 0-100)
            self.ax2.relim()
            self.ax2.autoscale_view(scalex=False, scaley=True)

        return self.line_att, self.line_vfr

    def start(self):
        """开始动画循环"""
        # interval=100 表示每 100ms 刷新一次图表 (10Hz)
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)
        plt.show()

if __name__ == '__main__':
    try:
        # 注意：在 ROS 中使用 matplotlib 时
        # plt.show() 会阻塞主线程，而 ROS 的 Subscriber 回调会在后台线程运行
        # 这是一个简单的实现方式，通常足够用于调试
        visualizer = ThrustVisualizer()
        visualizer.start()
    except rospy.ROSInterruptException:
        pass
