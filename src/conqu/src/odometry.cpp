#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class OdometryNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber encoder_sub_;
    ros::Publisher odom_pub_;
    tf2_ros::TransformBroadcaster odom_broadcaster_;

    // 里程计状态
    double x_ = 0.0;      // x位置
    double y_ = 0.0;      // y位置
    double theta_ = 0.0;  // 朝向角度
    double vx_ = 0.0;     // x方向速度
    double vy_ = 0.0;     // y方向速度
    double vth_ = 0.0;    // 角速度

    // 编码器参数
    double wheel_radius_ = 0.05;  // 轮子半径（米）
    double wheel_base_ = 0.3;     // 轮距（米）
    int encoder_resolution_ = 1000; // 编码器分辨率
    double encoder_to_meters_ = 0.001; // 编码器值到米的转换系数

    // 时间相关
    ros::Time last_time_;
    bool first_time_ = true;

    // 上一次的编码器值
    int last_left_encoder_ = 0;
    int last_right_encoder_ = 0;

public:
    OdometryNode() {
        // 初始化发布者和订阅者
        imu_sub_ = nh_.subscribe("imu/data", 10, &OdometryNode::imuCallback, this);
        encoder_sub_ = nh_.subscribe("processed_data", 10, &OdometryNode::encoderCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);

        // 获取参数
        nh_.param("wheel_radius", wheel_radius_, 0.05);
        nh_.param("wheel_base", wheel_base_, 0.3);
        nh_.param("encoder_resolution", encoder_resolution_, 1000);
        nh_.param("encoder_to_meters", encoder_to_meters_, 0.001);

        last_time_ = ros::Time::now();
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 更新角速度
        vth_ = msg->angular_velocity.z;
        
        // 更新朝向角度
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        theta_ = yaw;
    }

    void encoderCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        if (msg->data.size() < 5) {
            ROS_WARN("Invalid encoder data size");
            return;
        }

        // 获取左右轮编码器值（假设第2个和第4个值是左右轮编码器值）
        int left_encoder = msg->data[1];
        int right_encoder = msg->data[3];

        // 计算编码器增量
        int delta_left = left_encoder - last_left_encoder_;
        int delta_right = right_encoder - last_right_encoder_;

        // 更新上一次的编码器值
        last_left_encoder_ = left_encoder;
        last_right_encoder_ = right_encoder;

        // 计算轮子移动距离（米）
        double left_distance = delta_left * encoder_to_meters_;
        double right_distance = delta_right * encoder_to_meters_;

        // 计算线速度和角速度
        double v = (left_distance + right_distance) / 2.0;  // 平均线速度
        double omega = (right_distance - left_distance) / wheel_base_;  // 角速度

        // 计算x和y方向的速度
        vx_ = v * cos(theta_);
        vy_ = v * sin(theta_);

        // 更新时间
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();
        if (dt == 0) return;  // 避免除以零
        last_time_ = current_time;

        // 更新位置
        x_ += vx_ * dt;
        y_ += vy_ * dt;
        theta_ += omega;

        // 发布里程计消息
        publishOdometry(current_time);
    }

    void publishOdometry(const ros::Time& current_time) {
        // 创建并填充里程计消息
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // 设置位置
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        // 设置朝向
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // 设置速度
        odom.twist.twist.linear.x = vx_;
        odom.twist.twist.linear.y = vy_;
        odom.twist.twist.angular.z = vth_;

        // 发布里程计消息
        odom_pub_.publish(odom);

        // 发布TF变换
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation.x = q.x();
        odom_trans.transform.rotation.y = q.y();
        odom_trans.transform.rotation.z = q.z();
        odom_trans.transform.rotation.w = q.w();

        odom_broadcaster_.sendTransform(odom_trans);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_node");
    OdometryNode odometry_node;
    ros::spin();
    return 0;
} 