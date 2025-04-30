#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <std_msgs/Float64.h>


class OdometerNode {
public:
    OdometerNode(ros::NodeHandle& nh) : nh_(nh), x_(0.0), y_(0.0), theta_(90 * M_PI / 180.0), is_first_msg_(true) {
        nh_.param("wheelbase", L_, 1.765); // default wheelbase = 1.765 m
        nh_.param("steeringfactor", steer_fac_, 30.0);
        nh_.param("integraltype", INTEGRAL_TYPE, 1);
        nh_.param("alpha", alpha_, 0.0);
        nh_.param("windowsize", win_size_double, 0.0);
        nh_.param("variancethreshold", variance_threshold_, 0.0);
        nh_.param("curvethreshold", curve_threshold_, 9.0);
        nh_.param("postcurvesuppressiontime", post_curve_suppression_time_, 2.0);

        window_size_ = static_cast<size_t>(win_size_double);

        sub_ = nh_.subscribe("/speedsteer", 10, &OdometerNode::callback, this);
        pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/odom_path", 10);
        variance_pub = nh_.advertise<std_msgs::Float64>("/variance", 10);
        
        odom.header.frame_id = "odom";
        odom.child_frame_id = "vehicle";
        path_.header.frame_id = "odom";
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "vehicle";

        last_time_ = ros::Time::now();
    }

private:
    bool is_first_msg_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher path_pub_;
    ros::Publisher variance_pub;
    tf::TransformBroadcaster odom_broadcaster_;
    geometry_msgs::TransformStamped odom_tf;
    nav_msgs::Odometry odom;
    nav_msgs::Path path_;
    double x_, y_, theta_; // pose
    double L_; // wheelbase
    double steer_fac_;
    int INTEGRAL_TYPE;

    double smoothed_omega = 0.0;
    double alpha_;  // smoothing factor between 0 and 1 for omega
    ros::Time last_time_;

    // Sliding window data and parameters
    std::vector<double> steering_window_;
    double win_size_double;
    size_t window_size_;
    double variance_threshold_;; // "1" change in steering wheel angle reading

    // Post curve suppression
    ros::Time last_turn_time_;
    bool was_recently_turning_ = false;
    double curve_threshold_;              // e.g. 0.05 rad
    double post_curve_suppression_time_; // e.g. 2.0 sec

    // New method to compute effective steering
    bool isLowVariancePath(double scaled_steering, double* var) {
        // Add the new scaled steering value to the window
        steering_window_.push_back(scaled_steering);
        if (steering_window_.size() > window_size_) {
            steering_window_.erase(steering_window_.begin());
        }
        
        // Calculate mean over the window
        double sum = 0.0;
        for (double s : steering_window_) {
            sum += s;
        }
        double mean = sum / steering_window_.size();
        
        // Calculate variance over the window
        double variance = 0.0;
        for (double s : steering_window_) {
            variance += (s - mean) * (s - mean);
        }
        variance /= steering_window_.size();
        
        *var = variance;

        //ROS_INFO("VAR: %.4f", variance);

        // If variance is low, assume vehicle is going straight and ignore the steering angle.
        if (variance < variance_threshold_) {
            return true;
        } else {
            return false;
        }
    }

    bool shouldSuppressAfterSharpTurn(double delta, const ros::Time& current_time) {
        // Still actively turning
        if (abs(delta) > curve_threshold_) {
            last_turn_time_ = current_time;
            was_recently_turning_ = true;
            return false;
        }
    
        // Recently turned — still in recovery window
        if (was_recently_turning_) {
            double time_since_turn = (current_time - last_turn_time_).toSec();
            if (time_since_turn < post_curve_suppression_time_) {
                return true;
            } else {
                was_recently_turning_ = false;
            }
        }
    
        return false;
    }

    double movingAverage(double omega)
    {
        smoothed_omega = alpha_* omega + (1 - alpha_) * smoothed_omega;
        return smoothed_omega;
    }

    void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        if (is_first_msg_)
        {
            last_time_ = msg->header.stamp;
            is_first_msg_ = false;
            return;  // Skip this one, no dt to integrate
        }

        ros::Time current_time = msg->header.stamp;
        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;

        // Get data
        double v = (0.277778)*(msg->point.y);     // forward speed (m/s)
        double delta = msg->point.x; // get steering wheel angle (in degrees)
        
        // Bicycle model
        double delta_scaled = 3.14/180*delta/steer_fac_; // convert steering wheel angle in deg to wheel angle in rad
        double R = L_ / tan(delta_scaled); // turning radius
        double omega = v / R; //(v / L_) * tan(delta_scaled); // yaw rate
        //ROS_INFO("TRUE OMEGA: %.4f", omega);
        
        // -----------------------------------------------------------------
        // Adaptive Offset and Bicycle Model Again
        /*double delta_corr = adaptiveOffsetSteering(omega, v, delta);
        double delta_corr_scaled = 3.14/180*delta_corr/steer_fac_; // convert steering wheel angle in deg to wheel angle in rad
        R = L_ / tan(delta_corr_scaled); // turning radius
        omega = v / R; //(v / L_) * tan(delta_scaled); // yaw rate*/

        // -----------------------------------------------------------------
        // Compute corrected - filtered steering based on variance
        double variance;
        if(isLowVariancePath(delta, &variance)) // isStraightPath
        {
            omega = 0.0;
        }
        else
        {// Post Curve Suppression
            bool recently_turning = shouldSuppressAfterSharpTurn(delta, current_time);
            if (recently_turning)
                omega = 0.0; // suppress heading drift
            else
                omega = movingAverage(omega);
        };
        // PUBLISHING THE VARIANCE FOR TUNING PURPOSES
        std_msgs::Float64 var_msg;
        var_msg.data = variance;
        variance_pub.publish(var_msg);

        // Integrate pose, euler approx.
        if(INTEGRAL_TYPE == 1)
        {
            x_ += v * cos(theta_) * dt;
            y_ += v * sin(theta_) * dt;
            theta_ += omega * dt;
        }
        else if(INTEGRAL_TYPE == 2)
        { // Runge Kutta 2 Integration
            double mid_theta = theta_ + 0.5 * omega * dt;
            x_ += v * cos(mid_theta) * dt;
            y_ += v * sin(mid_theta) * dt;
            theta_ += omega * dt;
        }
        else if(INTEGRAL_TYPE == 3)
        { // Exact integration for unicycle model
            if (fabs(omega) > 1e-5) {
                x_ += R * (sin(theta_ + omega * dt) - sin(theta_));
                y_ += R * (-cos(theta_ + omega * dt) + cos(theta_));
            } else {
                // Approximate straight motion
                x_ += v * cos(theta_) * dt;
                y_ += v * sin(theta_) * dt;
            }
            theta_ += omega * dt;
        }
        
        // Publish odometry
        odom.header.stamp = current_time;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);

        odom.twist.twist.linear.x = v;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = omega;

        pub_.publish(odom);

        // path drawing
        geometry_msgs::PoseStamped pose;
        pose.header = odom.header;  // use the same timestamp and frame
        pose.pose = odom.pose.pose;    
        path_.poses.push_back(pose);
        path_.header.stamp = ros::Time::now();
        path_pub_.publish(path_);

        // Optional: broadcast TF transform
        odom_tf.header.stamp = current_time;
        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw(theta_);

        odom_broadcaster_.sendTransform(odom_tf);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometer");
    ros::NodeHandle nh;
    OdometerNode odometer(nh);
    ros::spin();
    return 0;
}   
