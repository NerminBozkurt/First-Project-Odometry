#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <math.h>

class GpsOdometer {
public:
    GpsOdometer(ros::NodeHandle& nh) {
        nh.param("lat_r", lat_ref_, 0.0);
        nh.param("lon_r", lon_ref_, 0.0);
        nh.param("alt_r", alt_ref_, 0.0);
        geodeticToECEF(lat_ref_, lon_ref_, alt_ref_, Xr_, Yr_, Zr_);
        ROS_INFO("Reference set from parameters: lat=%.6f lon=%.6f alt=%.2f", lat_ref_, lon_ref_, alt_ref_);

        fix_sub_ = nh.subscribe("/swiftnav/front/gps_pose", 1, &GpsOdometer::gpsCallback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/gps_odom", 1);
        path_pub_ = nh.advertise<nav_msgs::Path>("/gps_path", 1);

        path_.header.frame_id = "odom";
        has_last_ = false;
    }

private:
    ros::Subscriber fix_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher path_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    nav_msgs::Path path_;

    // Reference point
    double lat_ref_, lon_ref_, alt_ref_;
    double Xr_, Yr_, Zr_;

    // Last point
    double last_x_, last_y_, last_z_;
    bool has_last_;

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
            ROS_WARN_THROTTLE(5, "No GPS fix yet");
            return;
        }

        // Convert GPS to ECEF → ENU
        double Xp, Yp, Zp;
        geodeticToECEF(msg->latitude, msg->longitude, msg->altitude, Xp, Yp, Zp);

        double x, y, z;
        ecefToENU(Xp, Yp, Zp, x, y, z);

        // Filter large jumps
        if (has_last_) {
            double dx = x - last_x_;
            double dy = y - last_y_;
            double dz = z - last_z_;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist > 100.0) {
                ROS_WARN_THROTTLE(5, "Skipping large odometry jump: %.1f meters", dist);
                return;
            }
        }

        // Estimate heading (yaw)
        double yaw = 0.0;
        if (has_last_) {
            double dx = x - last_x_;
            double dy = y - last_y_;
            if (std::hypot(dx, dy) > 1e-2) {
                yaw = std::atan2(dy, dx);
            }
            ROS_INFO_THROTTLE(1, "dx=%.2f dy=%.2f → yaw=%.2f deg", dx, dy, yaw * 180.0 / M_PI);

        }

        // Update last position
        last_x_ = x;
        last_y_ = y;
        last_z_ = z;
        has_last_ = true;

        // Create quaternion from yaw
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        q.normalize();

        // Publish odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = msg->header.stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "gps";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = z;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom_pub_.publish(odom);

        // Broadcast TF
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, z));
        transform.setRotation(q);
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "gps"));

        // Append to path
        geometry_msgs::PoseStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose.pose;
        path_.poses.push_back(pose);
        path_.header.stamp = msg->header.stamp;
        path_pub_.publish(path_);
    }

    void geodeticToECEF(double lat, double lon, double alt, double& X, double& Y, double& Z) {
        double a = 6378137.0;
        double b = 6356752.0;
        double e_sq = 1 - (b * b) / (a * a);

        double phi = lat * M_PI / 180.0;
        double lambda = lon * M_PI / 180.0;
        double sin_phi = sin(phi), cos_phi = cos(phi);
        double sin_lambda = sin(lambda), cos_lambda = cos(lambda);
        double N = a / sqrt(1 - e_sq * sin_phi * sin_phi);

        X = (N + alt) * cos_phi * cos_lambda;
        Y = (N + alt) * cos_phi * sin_lambda;
        Z = (N * (1 - e_sq) + alt) * sin_phi;
    }

    void ecefToENU(double Xp, double Yp, double Zp, double& x, double& y, double& z) {
        double phi = lat_ref_ * M_PI / 180.0;
        double lambda = lon_ref_ * M_PI / 180.0;
        double sin_phi = sin(phi), cos_phi = cos(phi);
        double sin_lambda = sin(lambda), cos_lambda = cos(lambda);

        double dx = Xp - Xr_;
        double dy = Yp - Yr_;
        double dz = Zp - Zr_;

        x = -sin_lambda * dx + cos_lambda * dy;
        y = -sin_phi * cos_lambda * dx - sin_phi * sin_lambda * dy + cos_phi * dz;
        z = cos_phi * cos_lambda * dx + cos_phi * sin_lambda * dy + sin_phi * dz;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odometer");
    ros::NodeHandle nh("~");
    GpsOdometer node(nh);
    ros::spin();
    return 0;
}
