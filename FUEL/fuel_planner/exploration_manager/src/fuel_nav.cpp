/*****************************************************************************************
 * 自定义控制器跟踪egoplanner轨迹
 * 本代码采用的mavros的速度控制进行跟踪
 * 编译成功后直接运行就行，遥控器先position模式起飞，然后rviz打点，再切offborad模式即可
 ******************************************************************************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define VELOCITY2D_CONTROL 0b101111000111 //设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
//设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这四个我给0，其他都是1.
class Ctrl
{
    public:
        Ctrl();
        void state_cb(const mavros_msgs::State::ConstPtr &msg);
        void position_cb(const nav_msgs::Odometry::ConstPtr &msg);
        void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
        void control(const ros::TimerEvent&);
	    ros::NodeHandle nh;
        visualization_msgs::Marker trackpoint;
        quadrotor_msgs::PositionCommand ego;
        tf::StampedTransform ts;//用来发布无人机当前位置的坐标系坐标轴
        tf::TransformBroadcaster tfBroadcasterPointer;	//广播坐标轴
        unsigned short velocity_mask = VELOCITY2D_CONTROL;
        mavros_msgs::PositionTarget current_goal;
        mavros_msgs::RCIn rc;
        nav_msgs::Odometry position_msg;
        geometry_msgs::PoseStamped target_pos;
        mavros_msgs::State current_state;
        float position_x, position_y, position_z, now_x, now_y, now_yaw, current_yaw, targetpos_x, targetpos_y;
        float ego_pos_x, ego_pos_y, ego_pos_z, ego_vel_x, ego_vel_y, ego_vel_z, ego_a_x, ego_a_y, ego_a_z, ego_yaw, ego_yaw_rate; //EGO planner information has position velocity acceleration yaw yaw_dot
        bool receive, get_now_pos;//触发轨迹的条件判断
        ros::Subscriber state_sub, twist_sub, target_sub, position_sub;
        ros::Publisher local_pos_pub, pubMarker;
        ros::Timer timer;
};
Ctrl::Ctrl()
{
    timer = nh.createTimer(ros::Duration(0.02), &Ctrl::control, this);
    state_sub = nh.subscribe("/mavros/state", 10, &Ctrl::state_cb, this);
    position_sub=nh.subscribe("/Odometry", 10, &Ctrl::position_cb, this);
    target_sub = nh.subscribe("move_base_simple/goal", 10, &Ctrl::target_cb, this);
    twist_sub = nh.subscribe("/planning/pos_cmd", 10, &Ctrl::twist_cb, this);
    local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    pubMarker = nh.advertise<visualization_msgs::Marker>("/track_drone_point", 5);
    get_now_pos = false;
    receive = false;
}
void Ctrl::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

//read vehicle odometry
void Ctrl::position_cb(const nav_msgs::Odometry::ConstPtr&msg)
{
    position_msg=*msg;
	tf2::Quaternion quat;
	tf2::convert(msg->pose.pose.orientation, quat); //把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
	double roll, pitch, yaw;
	tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    ts.stamp_ = msg->header.stamp;
    ts.frame_id_ = "world";
    ts.child_frame_id_ = "drone_frame";
    ts.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    ts.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tfBroadcasterPointer.sendTransform(ts);
	if (!get_now_pos) 
	{
		now_x = position_msg.pose.pose.position.x;
		now_y = position_msg.pose.pose.position.y;
		tf2::Quaternion quat;
		tf2::convert(msg->pose.pose.orientation, quat);
		now_yaw = yaw;
		get_now_pos = true;
	}
    position_x = position_msg.pose.pose.position.x;
    position_y = position_msg.pose.pose.position.y;
    position_z = position_msg.pose.pose.position.z;
	current_yaw = yaw;

}

void Ctrl::target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)//读取rviz的航点
{
    receive = true;
    target_pos = *msg;
    targetpos_x = target_pos.pose.position.x;
    targetpos_y = target_pos.pose.position.y;
}

//读取ego里的位置速度加速度yaw和yaw-dot信息，其实只需要ego的位置速度和yaw就可以了
void Ctrl::twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)//ego的回调函数
{
	ego = *msg;
    ego_pos_x = ego.position.x;
	ego_pos_y = ego.position.y;
	ego_pos_z = ego.position.z;
	ego_vel_x = ego.velocity.x;
	ego_vel_y = ego.velocity.y;
	ego_vel_z = ego.velocity.z;
	ego_yaw = ego.yaw;
	ego_yaw_rate = ego.yaw_dot;
}

void Ctrl::control(const ros::TimerEvent&)
{
    if(!receive) //如果没有在rviz上打点，则offboard模式下会保持在1m的高度
    {
        current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_goal.header.stamp = ros::Time::now();
        current_goal.type_mask = velocity_mask;
        current_goal.velocity.x = (now_x - position_x)*1;;
        current_goal.velocity.y = (now_y - position_y)*1;;
        current_goal.velocity.z = (1 - position_z)*1;
        current_goal.yaw = now_yaw;
        ROS_INFO("请等待");
    }

    //if receive plan in rviz, the EGO plan information can input mavros and vehicle can auto navigation
    if(receive)//触发后进行轨迹跟踪
    {
        current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;//选择local系，一定要local系
        current_goal.header.stamp = ros::Time::now();
        current_goal.type_mask = velocity_mask;//这个就算对应的掩码设置，可以看mavros_msgs::PositionTarget消息格式
        current_goal.velocity.x =  0.5 * ego_vel_x + (ego_pos_x - position_x) * 1;
        current_goal.velocity.y =  0.5 * ego_vel_y + (ego_pos_y - position_y) * 1;
        current_goal.velocity.z =  (ego_pos_z - position_z) * 1; 
        current_goal.yaw = ego_yaw;
        ROS_INFO("EGO规划速度：vel_x = %.2f", sqrt(pow(current_goal.velocity.x, 2)+pow(current_goal.velocity.y, 2)));
        // TODO 增加一个速度过大的限制保护，防止暴走
    }
    local_pos_pub.publish(current_goal);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cxr_egoctrl_v1");
	setlocale(LC_ALL,"");
    Ctrl ctrl;
    ros::spin();
	return 0;
}
