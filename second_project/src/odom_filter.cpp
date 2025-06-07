#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <vector>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class MedianLowpassOdomFilter
{
public:
  MedianLowpassOdomFilter()
  {
    ros::NodeHandle pnh("~");
    pnh.param("window_size", window_size_, 5);
    pnh.param("alpha", alpha_, 0.3);  // low-pass factor
    pnh.param("input_topic", input_topic_, std::string("/odometry"));
    pnh.param("output_topic", output_topic_, std::string("/odometry_filtered"));

    sub_ = nh_.subscribe(input_topic_, 10, &MedianLowpassOdomFilter::odomCallback, this);
    pub_ = nh_.advertise<nav_msgs::Odometry>(output_topic_, 10);

    // initialize previous orientation to identity
    prev_orientation_.setRPY(0.0, 0.0, 0.0);
    has_prev_orientation_ = false;

    prev_lin_x_ = prev_lin_y_ = prev_lin_z_ = 0.0;
    prev_ang_x_ = prev_ang_y_ = prev_ang_z_ = 0.0;

    ROS_INFO("MedianLowpassOdomFilter: window_size=%d, alpha=%.2f", window_size_, alpha_);
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // buffer velocities for median filter
    pushDeque(lin_x_buf_, msg->twist.twist.linear.x);
    pushDeque(lin_y_buf_, msg->twist.twist.linear.y);
    pushDeque(lin_z_buf_, msg->twist.twist.linear.z);
    pushDeque(ang_x_buf_, msg->twist.twist.angular.x);
    pushDeque(ang_y_buf_, msg->twist.twist.angular.y);
    pushDeque(ang_z_buf_, msg->twist.twist.angular.z);

    if ((int)lin_x_buf_.size() < window_size_)
      return;

    // median velocities
    double mx = median(lin_x_buf_);
    double my = median(lin_y_buf_);
    double mz = median(lin_z_buf_);
    double ax = median(ang_x_buf_);
    double ay = median(ang_y_buf_);
    double az = median(ang_z_buf_);

    // low-pass velocities
    double fx = alpha_ * mx + (1.0 - alpha_) * prev_lin_x_;
    double fy = alpha_ * my + (1.0 - alpha_) * prev_lin_y_;
    double fz = alpha_ * mz + (1.0 - alpha_) * prev_lin_z_;
    double gx = alpha_ * ax + (1.0 - alpha_) * prev_ang_x_;
    double gy = alpha_ * ay + (1.0 - alpha_) * prev_ang_y_;
    double gz = alpha_ * az + (1.0 - alpha_) * prev_ang_z_;

    prev_lin_x_ = fx; prev_lin_y_ = fy; prev_lin_z_ = fz;
    prev_ang_x_ = gx; prev_ang_y_ = gy; prev_ang_z_ = gz;

    // orientation smoothing
    tf2::Quaternion q_curr;
    tf2::fromMsg(msg->pose.pose.orientation, q_curr);

    if (!has_prev_orientation_) {
      prev_orientation_ = q_curr;
      has_prev_orientation_ = true;
    }

    tf2::Quaternion q_filt = prev_orientation_.slerp(q_curr, alpha_);
    prev_orientation_ = q_filt;

    // publish filtered odometry
    nav_msgs::Odometry filt = *msg;
    filt.twist.twist.linear.x = fx;
    filt.twist.twist.linear.y = fy;
    filt.twist.twist.linear.z = fz;
    filt.twist.twist.angular.x = gx;
    filt.twist.twist.angular.y = gy;
    filt.twist.twist.angular.z = gz;
    filt.pose.pose.orientation = tf2::toMsg(q_filt);

    pub_.publish(filt);
  }

  // helper functions
  void pushDeque(std::deque<double>& dq, double v) {
    dq.push_back(v);
    if ((int)dq.size() > window_size_) dq.pop_front();
  }

  double median(const std::deque<double>& buf) {
    std::vector<double> tmp(buf.begin(), buf.end());
    std::sort(tmp.begin(), tmp.end());
    int n = tmp.size();
    return (n % 2 == 1) ? tmp[n/2] : 0.5 * (tmp[n/2 - 1] + tmp[n/2]);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  int window_size_;
  double alpha_;
  std::string input_topic_, output_topic_;

  std::deque<double> lin_x_buf_, lin_y_buf_, lin_z_buf_;
  std::deque<double> ang_x_buf_, ang_y_buf_, ang_z_buf_;

  double prev_lin_x_, prev_lin_y_, prev_lin_z_;
  double prev_ang_x_, prev_ang_y_, prev_ang_z_;
  tf2::Quaternion prev_orientation_;
  bool has_prev_orientation_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "median_lowpass_odom_filter");
  MedianLowpassOdomFilter node;
  ros::spin();
  return 0;
}
