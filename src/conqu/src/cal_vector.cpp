#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <vector>
#include <cmath>

// 麦克纳姆轮解算参数
const double WHEEL_RADIUS = 0.154;  // 轮子半径 (单位: 米)
const double L = 0.393/2;             // 机器人前后轮中心距的一半 (单位: 米)
const double W = 0.49/2;             // 机器人左右轮中心距的一半 (单位: 米)

ros::Publisher odom_pub;  // 发布解算后的里程计数据

void callback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    const std::vector<int32_t>& data = msg->data;

    // 检查数据长度是否符合要求
    if (data.size() != 5) {
        ROS_WARN("Invalid data size. Expected 5 elements.");
        return;
    }

    // 显式转换为 double 类型
    double wheel_speeds[4] = {
        static_cast<double>(data[0]),
        static_cast<double>(data[1]),
        static_cast<double>(data[2]),
        static_cast<double>(data[3])
    };
    for (int i = 0; i < 4; ++i) {
        wheel_speeds[i] = data[i]*2*M_PI / 60.0; // 转化为rad/s,得到角速度
    }

    double yaw_speed = static_cast<double>(data[4]);
    yaw_speed = yaw_speed /100;

    // 麦克纳姆轮解算公式
    double Vx = (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3])*sqrt(2)/2* (WHEEL_RADIUS / 4.0);
    double Vy = (-wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] - wheel_speeds[3]) * (WHEEL_RADIUS / 4.0);
    double omega = (-wheel_speeds[0] + wheel_speeds[1] - wheel_speeds[2] + wheel_speeds[3]) * (WHEEL_RADIUS / (4.0 * (L + W)));

    // 发布解算结果为 /odom_wheel
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";  // 里程计坐标系
    odom_msg.child_frame_id = "base_link";  // 机器人底盘坐标系

    // 设置线速度和角速度
    odom_msg.twist.twist.linear.x = Vx;
    odom_msg.twist.twist.linear.y = Vy;
    odom_msg.twist.twist.angular.z = omega;

    // 发布里程计消息
    odom_pub.publish(odom_msg);

    // 打印解算结果
    ROS_INFO("Vx: %.2f, Vy: %.2f, omega: %.2f", Vx, Vy, omega);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mecanum_solver_node");
    ros::NodeHandle nh;

    // 订阅包含麦克纳姆轮速度和 Yaw 轴速度的话题
    ros::Subscriber sub = nh.subscribe("processed_data", 10, callback);

    // 发布解算后的里程计数据
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_wheel", 10);

    ros::spin();
    return 0;
}