#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
from tf.transformations import quaternion_from_euler

class SquareTrajPublisher:
    def __init__(self):
        rospy.init_node('square_traj_publisher')

        # 话题名称需与 se3_controller 订阅的名称一致
        traj_pub_topic = rospy.get_param('~traj_pub_topic', '/command/trajectory')
        self.traj_pub = rospy.Publisher(traj_pub_topic, MultiDOFJointTrajectory, queue_size=10)

        # 定义正方形的四个顶点 (x, y, z, yaw)
        # 假设起飞点是 (0,0)，高度锁定 2.0m
        # 顺时针或逆时针飞行
        self.height = 2.0
        self.side_length = 2.0
        
        self.waypoints = [
            [0.0, 0.0, self.height, 0.0],              # 点1: 原点上方
            [self.side_length, 0.0, self.height, 0.0], # 点2: X轴正向
            [self.side_length, self.side_length, self.height, 0.0], # 点3: 对角
            [0.0, self.side_length, self.height, 0.0]  # 点4: Y轴正向
        ]

        self.current_wp_index = 0
        self.time_per_point = 5.0 # 每个点停留 5 秒
        self.last_switch_time = rospy.Time.now()

        rospy.loginfo("Start publishing Square Trajectory...")
        rospy.loginfo(f"Height: {self.height}m, Side: {self.side_length}m")

        # 启动发布循环
        self.start_publishing()

    def start_publishing(self):
        rate = rospy.Rate(20) # 20Hz 发布频率，确保不触发控制器的超时保护
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            # 检查是否需要切换到下一个目标点
            if (current_time - self.last_switch_time).to_sec() > self.time_per_point:
                self.current_wp_index += 1
                if self.current_wp_index >= len(self.waypoints):
                    self.current_wp_index = 0 # 循环回到起点
                
                self.last_switch_time = current_time
                rospy.loginfo(f"Switching to Waypoint {self.current_wp_index}: {self.waypoints[self.current_wp_index]}")

            # 获取当前目标点数据
            target = self.waypoints[self.current_wp_index]
            self.publish_point(target)

            rate.sleep()

    def publish_point(self, target):
        """
        构造并发布消息
        target: [x, y, z, yaw]
        """
        traj_msg = MultiDOFJointTrajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.header.frame_id = "map" # 或者是 "world"，取决于你的 TF树

        point = MultiDOFJointTrajectoryPoint()

        # 1. 设置位置 (Transform)
        trans = Transform()
        trans.translation.x = target[0]
        trans.translation.y = target[1]
        trans.translation.z = target[2]

        # 设置姿态 (Yaw)
        q = quaternion_from_euler(0, 0, target[3]) # Roll=0, Pitch=0
        trans.rotation.x = q[0]
        trans.rotation.y = q[1]
        trans.rotation.z = q[2]
        trans.rotation.w = q[3]

        # 2. 设置速度 (Velocity) - 全为 0
        vel = Twist()
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 0.0

        # 3. 设置加速度 (Acceleration) - 全为 0
        acc = Twist()
        acc.linear.x = 0.0
        acc.linear.y = 0.0
        acc.linear.z = 0.0
        acc.angular.x = 0.0
        acc.angular.y = 0.0
        acc.angular.z = 0.0

        # 将数据装填进 Point
        point.transforms.append(trans)
        point.velocities.append(vel)
        point.accelerations.append(acc)

        # 将 Point 装填进 Message
        traj_msg.points.append(point)

        self.traj_pub.publish(traj_msg)

if __name__ == '__main__':
    try:
        obj = SquareTrajPublisher()
    except rospy.ROSInterruptException:
        pass
