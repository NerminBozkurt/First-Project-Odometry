#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped t;
  t.header.stamp = msg->header.stamp;
  t.header.frame_id = msg->header.frame_id;  // typically "odom"
  t.child_frame_id  = "base_link";
  t.transform.translation.x = msg->pose.pose.position.x;
  t.transform.translation.y = msg->pose.pose.position.y;
  t.transform.translation.z = msg->pose.pose.position.z;
  t.transform.rotation      = msg->pose.pose.orientation;
  br.sendTransform(t);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_to_tf");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(
    "/odometry", 10, odomCallback
  );
  ros::spin();
  return 0;
}