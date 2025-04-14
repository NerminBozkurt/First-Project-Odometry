#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

class GpsOdometer {
public:
    GpsOdometer(ros::NodeHandle& nh) {
        // Load parameters from launch file
        nh.param("lat_r", lat_ref_, 0.0);
        nh.param("lon_r", lon_ref_, 0.0);
        nh.param("alt_r", alt_ref_, 0.0);

        geodeticToECEF(lat_ref_, lon_ref_, alt_ref_, Xr_, Yr_, Zr_);
        ROS_INFO("Reference set from parameters: lat=%.6f lon=%.6f alt=%.2f", lat_ref_, lon_ref_, alt_ref_);

        fix_sub_ = nh.subscribe("/swiftnav/front/gps_pose", 1, &GpsOdometer::gpsCallback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/gps_odom", 1);
    }

private:
    ros::Subscriber fix_sub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster tf_broadcaster_;

    // Reference point
    double lat_ref_, lon_ref_, alt_ref_;
    double Xr_, Yr_, Zr_;

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
            ROS_WARN_THROTTLE(5, "No GPS fix yet");
            return;
        }

        double Xp, Yp, Zp;
        geodeticToECEF(msg->latitude, msg->longitude, msg->altitude, Xp, Yp, Zp);

        double x, y, z;
        ecefToENU(Xp, Yp, Zp, x, y, z);

        nav_msgs::Odometry odom;
        odom.header.stamp = msg->header.stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "gps";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = z;
        odom.pose.pose.orientation.w = 1.0; // no heading yet

        odom_pub_.publish(odom);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, z));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);  // no heading yet
        transform.setRotation(q);
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "gps"));
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
