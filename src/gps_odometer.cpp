#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

#define DEG2RAD M_PI / 180.0
#define EARTH_A 6378137.0
#define EARTH_B 6356752.3142
#define EARTH_E2 ((EARTH_A * EARTH_A - EARTH_B * EARTH_B) / (EARTH_A * EARTH_A))

class GpsOdometer {
public:
    GpsOdometer() : nh_(), pnh_("~"), ref_set_(false) {
        pnh_.param("lat_r", lat_r_, 0.0);
        pnh_.param("lon_r", lon_r_, 0.0);
        pnh_.param("alt_r", alt_r_, 0.0);
        if (lat_r_ != 0.0 || lon_r_ != 0.0 || alt_r_ != 0.0) {
            ref_set_ = true;
            lla_to_ecef(lat_r_, lon_r_, alt_r_, x0_, y0_, z0_);
        }

        sub_ = nh_.subscribe("/swiftnav/front/gps_pose", 1000, &GpsOdometer::callback, this);
        pub_ = nh_.advertise<nav_msgs::Odometry>("/gps_odom", 10);
    }

    void callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (msg->status.status < 0) return; // invalid fix

        double x, y, z;
        lla_to_ecef(msg->latitude, msg->longitude, msg->altitude, x, y, z);

        if (!ref_set_) {
            x0_ = x;
            y0_ = y;
            z0_ = z;
            lat_r_ = msg->latitude;
            lon_r_ = msg->longitude;
            alt_r_ = msg->altitude;
            ref_set_ = true;
            return;
        }

        double enu_x, enu_y, enu_z;
        ecef_to_enu(x, y, z, enu_x, enu_y, enu_z);

        ros::Time stamp = msg->header.stamp;

        // Calculate heading if we have previous ENU
        double yaw = 0.0;
        if (has_prev_) {
            double dx = enu_x - prev_x_;
            double dy = enu_y - prev_y_;
            if (dx != 0.0 || dy != 0.0)
                yaw = std::atan2(dy, dx);
        }

        has_prev_ = true;
        prev_x_ = enu_x;
        prev_y_ = enu_y;

        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);

        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "gps_link";
        odom.pose.pose.position.x = enu_x;
        odom.pose.pose.position.y = enu_y;
        odom.pose.pose.position.z = enu_z;
        odom.pose.pose.orientation = quat;

        pub_.publish(odom);

        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "gps_link";
        tf_msg.transform.translation.x = enu_x;
        tf_msg.transform.translation.y = enu_y;
        tf_msg.transform.translation.z = enu_z;
        tf_msg.transform.rotation = quat;

        tf_broadcaster_.sendTransform(tf_msg);
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    tf::TransformBroadcaster tf_broadcaster_;

    double lat_r_, lon_r_, alt_r_;
    double x0_, y0_, z0_;
    bool ref_set_;

    double prev_x_ = 0.0, prev_y_ = 0.0;
    bool has_prev_ = false;

    void lla_to_ecef(double lat, double lon, double alt, double& x, double& y, double& z) {
        lat *= DEG2RAD;
        lon *= DEG2RAD;
        double N = EARTH_A / sqrt(1.0 - EARTH_E2 * sin(lat) * sin(lat));
        x = (N + alt) * cos(lat) * cos(lon);
        y = (N + alt) * cos(lat) * sin(lon);
        z = (N * (1 - EARTH_E2) + alt) * sin(lat);
    }

    void ecef_to_enu(double x, double y, double z, double& enu_x, double& enu_y, double& enu_z) {
        double lat = lat_r_ * DEG2RAD;
        double lon = lon_r_ * DEG2RAD;

        double dx = x - x0_;
        double dy = y - y0_;
        double dz = z - z0_;

        enu_x = -sin(lon) * dx + cos(lon) * dy;
        enu_y = -sin(lat) * cos(lon) * dx - sin(lat) * sin(lon) * dy + cos(lat) * dz;
        enu_z = cos(lat) * cos(lon) * dx + cos(lat) * sin(lon) * dy + sin(lat) * dz;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odometer");
    GpsOdometer node;
    ros::spin();
    return 0;
}
