#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <deque>

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
    
    // 滤波相关参数
    std::deque<double> vx_buffer_;
    std::deque<double> vy_buffer_;
    std::deque<double> vth_buffer_;
    int buffer_size_ = 5;  // 滑动窗口大小
    
    // 噪声阈值
    double velocity_threshold_ = 0.05;  // 速度变化阈值
    double omega_threshold_ = 0.1;      // 角速度变化阈值

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
        nh_.param("buffer_size", buffer_size_, 5);
        nh_.param("velocity_threshold", velocity_threshold_, 0.05);
        nh_.param("omega_threshold", omega_threshold_, 0.1);

        last_time_ = ros::Time::now();
    }

    // 添加滑动平均滤波函数
    double movingAverage(std::deque<double>& buffer, double new_value) {
        // 检查是否是异常值
        if (!buffer.empty()) {
            double last_value = buffer.back();
            // 如果变化太大，认为是噪声，对其进行修正
            if (std::abs(new_value - last_value) > velocity_threshold_) {
                new_value = last_value + (new_value > last_value ? velocity_threshold_ : -velocity_threshold_);
            }
        }
        
        // 更新缓冲区
        buffer.push_back(new_value);
        if (buffer.size() > buffer_size_) {
            buffer.pop_front();
        }
        
        // 计算平均值
        double sum = 0.0;
        for (const auto& val : buffer) {
            sum += val;
        }
        return sum / buffer.size();
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 获取原始角速度
        double raw_vth = msg->angular_velocity.z;
        
        // 应用滑动平均滤波，减少角速度噪声
        vth_ = movingAverage(vth_buffer_, raw_vth);
        
        // 更新朝向角度
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // 平滑角度变化
        double theta_diff = yaw - theta_;
        if (std::abs(theta_diff) > omega_threshold_) {
            theta_ += (theta_diff > 0 ? omega_threshold_ : -omega_threshold_);
        } else {
            theta_ = yaw;
        }
    }

    void encoderCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        if (msg->data.size() < 5) {
            ROS_WARN("Invalid encoder data size");
            return;
        }

        // 获取左右轮编码器值（假设第2个和第4个值是左右轮编码器值）
        int left_encoder = msg->data[1];
        int right_encoder = msg->data[3];

        // 异常值检测，如果编码器数据跳变过大，则忽略
        if (!first_time_) {
            int max_encoder_jump = 1000; // 根据实际情况调整
            if (std::abs(left_encoder - last_left_encoder_) > max_encoder_jump ||
                std::abs(right_encoder - last_right_encoder_) > max_encoder_jump) {
                ROS_WARN("Encoder jump too large, ignoring this update");
                return;
            }
        }

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

        // 更新时间
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();
        if (dt < 0.001) return;  // 避免除以非常小的值
        last_time_ = current_time;

        // 计算原始x和y方向的速度
        double raw_vx = v * cos(theta_);
        double raw_vy = v * sin(theta_);
        
        // 应用滑动平均滤波
        vx_ = movingAverage(vx_buffer_, raw_vx);
        vy_ = movingAverage(vy_buffer_, raw_vy);

        // 更新位置（使用平滑后的速度）
        x_ += vx_ * dt;
        y_ += vy_ * dt;
        theta_ += vth_ * dt;
        
        // 确保theta_在[-pi, pi]范围内
        theta_ = atan2(sin(theta_), cos(theta_));

        first_time_ = false;
        
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
        
        // 设置协方差矩阵（增加以提高稳定性）
        for (int i = 0; i < 36; i++) {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        // 位置协方差
        odom.pose.covariance[0] = 0.01;  // x位置协方差
        odom.pose.covariance[7] = 0.01;  // y位置协方差
        odom.pose.covariance[35] = 0.01; // 角度协方差
        // 速度协方差
        odom.twist.covariance[0] = 0.01;  // x速度协方差
        odom.twist.covariance[7] = 0.01;  // y速度协方差
        odom.twist.covariance[35] = 0.01; // 角速度协方差

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