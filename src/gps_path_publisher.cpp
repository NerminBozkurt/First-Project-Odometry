#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

class GpsPathDrawer {
public:
    GpsPathDrawer() {
        sub_ = nh_.subscribe("/gps_odom", 100, &GpsPathDrawer::odomCallback, this);
        pub_ = nh_.advertise<nav_msgs::Path>("/gps_path", 10);
        path_.header.frame_id = "odom";
        has_last_ = false;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        const auto& pos = msg->pose.pose.position;

        // 1. Skip if position is zero (before GPS ref is set)
        if (pos.x == 0.0 && pos.y == 0.0 && pos.z == 0.0) {
            ROS_WARN_THROTTLE(5, "Skipping zero-position odometry");
            return;
        }

        // 2. Skip if the jump from last point is too big (>100m)
        if (has_last_) {
            double dx = pos.x - last_.x;
            double dy = pos.y - last_.y;
            double dz = pos.z - last_.z;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist > 100.0) {
                ROS_WARN_THROTTLE(5, "Skipping large odometry jump: %.1f meters", dist);
                return;
            }
        }

        last_ = pos;
        has_last_ = true;

        // 3. Add the pose to path
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        path_.poses.push_back(pose);
        path_.header.stamp = ros::Time::now();
        pub_.publish(path_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    nav_msgs::Path path_;

    geometry_msgs::Point last_;
    bool has_last_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_path_publisher");
    GpsPathDrawer pd;
    ros::spin();
    return 0;
}
