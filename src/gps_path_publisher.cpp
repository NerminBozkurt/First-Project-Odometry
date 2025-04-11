#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class GpsPathDrawer {
public:
    GpsPathDrawer() {
        sub_ = nh_.subscribe("/gps_odom", 100, &GpsPathDrawer::odomCallback, this);
        pub_ = nh_.advertise<nav_msgs::Path>("/gps_path", 10);
        path_.header.frame_id = "odom";
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_path_publisher");
    GpsPathDrawer pd;
    ros::spin();
    return 0;
}
