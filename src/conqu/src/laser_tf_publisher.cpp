#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_tf_publisher");
  ros::NodeHandle nh;

  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tf_laser1;
  geometry_msgs::TransformStamped tf_laser2;

  ros::Rate rate(10.0);
  while (ros::ok()) {
    ros::Time now = ros::Time::now();

    // laser1 在 base_link 后方（-0.3m）
    tf_laser1.header.stamp = now;
    tf_laser1.header.frame_id = "base_link";
    tf_laser1.child_frame_id = "laser1";
    tf_laser1.transform.translation.x = -0.3;
    tf_laser1.transform.translation.y = 0.0;
    tf_laser1.transform.translation.z = 0.0;
    tf_laser1.transform.rotation.x = 0.0;
    tf_laser1.transform.rotation.y = 0.0;
    tf_laser1.transform.rotation.z = 0.0;
    tf_laser1.transform.rotation.w = 1.0;

    // laser2 在 base_link 前方（+0.3m）
    tf_laser2.header.stamp = now;
    tf_laser2.header.frame_id = "base_link";
    tf_laser2.child_frame_id = "laser2";
    tf_laser2.transform.translation.x = 0.3;
    tf_laser2.transform.translation.y = 0.0;
    tf_laser2.transform.translation.z = 0.0;
    tf_laser2.transform.rotation.x = 0.0;
    tf_laser2.transform.rotation.y = 0.0;
    tf_laser2.transform.rotation.z = 0.0;
    tf_laser2.transform.rotation.w = 1.0;

    // 发布两个 TF
    br.sendTransform(tf_laser1);
    br.sendTransform(tf_laser2);

    rate.sleep();
  }

  return 0;
}
