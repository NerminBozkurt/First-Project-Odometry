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
    GpsOdometer(ros::NodeHandle& nh)
      : origin_set_(false)
    {
        // Reference geodetic origin
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
    // ROS interfaces
    ros::Subscriber fix_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher path_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    nav_msgs::Path path_;

    // Geodetic reference
    double lat_ref_, lon_ref_, alt_ref_;
    double Xr_, Yr_, Zr_;

    // Origin and normalization
    bool origin_set_;
    double base_x_, base_y_, base_z_;
    const double norm_x_ = 14.82;
    const double norm_y_ = 0.0;
    const double norm_z_ = 0.0;

    // Last position for filtering and heading
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

        double x_raw, y_raw, z_raw;
        ecefToENU(Xp, Yp, Zp, x_raw, y_raw, z_raw);

        // Set origin on first valid measurement
        if (!origin_set_) {
            base_x_ = x_raw;
            base_y_ = y_raw;
            base_z_ = z_raw;
            origin_set_ = true;
            ROS_INFO("Origin set at ENU raw = {%.3f, %.3f, %.3f}", base_x_, base_y_, base_z_);
        }

        // Normalize so that starting point maps to (14.82, 0, 0)
        double x = x_raw - base_x_ + norm_x_;
        double y = y_raw - base_y_ + norm_y_;
        double z = z_raw - base_z_ + norm_z_;

        // Filter large jumps
        if (has_last_) {
            double dx = x - last_x_;
            double dy = y - last_y_;
            double dz = z - last_z_;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (dist > 100.0) {
                ROS_WARN_THROTTLE(5, "Skipping large odometry jump: %.1f meters", dist);
                return;
            }
        }

        // Estimate heading
        double yaw = 0.0;
        if (has_last_) {
            double dx = x - last_x_;
            double dy = y - last_y_;
            if (std::hypot(dx, dy) > 1e-2) {
                yaw = std::atan2(dy, dx);
            }
            ROS_INFO_THROTTLE(1, "dx=%.2f dy=%.2f → yaw=%.2f deg", dx, dy, yaw * 180.0 / M_PI);
        }

        // Update last for next iteration
        last_x_ = x;
        last_y_ = y;
        last_z_ = z;
        has_last_ = true;

        // Build quaternion
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

    void geodeticToECEF(double lat, double lon, double alt,
                        double& X, double& Y, double& Z) {
        const double a = 6378137.0;
        const double b = 6356752.0;
        const double e_sq = 1 - (b*b)/(a*a);
        
        double phi = lat * M_PI/180.0;
        double lambda = lon * M_PI/180.0;
        double sinphi = sin(phi), cosphi = cos(phi);
        double sinl = sin(lambda), cosl = cos(lambda);
        double N = a / std::sqrt(1 - e_sq*sinphi*sinphi);

        X = (N + alt) * cosphi * cosl;
        Y = (N + alt) * cosphi * sinl;
        Z = (N*(1 - e_sq) + alt) * sinphi;
    }

    void ecefToENU(double Xp, double Yp, double Zp,
                   double& x, double& y, double& z) {
        double phi = lat_ref_ * M_PI/180.0;
        double lambda = lon_ref_ * M_PI/180.0;
        double sinphi = sin(phi), cosphi = cos(phi);
        double sinl = sin(lambda), cosl = cos(lambda);

        double dx = Xp - Xr_;
        double dy = Yp - Yr_;
        double dz = Zp - Zr_;

        x = -sinl*dx + cosl*dy;
        y = -sinphi*cosl*dx - sinphi*sinl*dy + cosphi*dz;
        z = cosphi*cosl*dx + cosphi*sinl*dy + sinphi*dz;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odometer_normalized");
    ros::NodeHandle nh("~");
    GpsOdometer node(nh);
    ros::spin();
    return 0;
}
