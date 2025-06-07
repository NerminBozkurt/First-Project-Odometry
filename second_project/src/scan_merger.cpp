// src/scan_merge_tf.cpp

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <limits>

class ScanMergerTF
{
public:
  ScanMergerTF()
    : nh_(),
      pnh_("~"),
      tf_buffer_(),
      tf_listener_(tf_buffer_),
      sub_front_(nh_, pnh_.param<std::string>("topic_front", "/scan_front"), 10),
      sub_back_ (nh_, pnh_.param<std::string>("topic_back",  "/scan_back" ), 10),
      sync_(SyncPolicy(10), sub_front_, sub_back_)
  {
    output_topic_ = pnh_.param<std::string>("output_topic", "/scan_merged");
    base_frame_   = pnh_.param<std::string>("base_frame",   "base_link");
    robot_radius_ = pnh_.param<double>("robot_radius", 0.5);

    sync_.registerCallback(
      boost::bind(&ScanMergerTF::scanCallback, this, _1, _2));

    pub_ = nh_.advertise<sensor_msgs::LaserScan>(output_topic_, 5);

    ROS_INFO("ScanMergerTF: merging '%s' + '%s' → '%s' in frame '%s' "
             "(self‐filter radius = %.2fm)",
      sub_front_.getTopic().c_str(),
      sub_back_ .getTopic().c_str(),
      output_topic_.c_str(),
      base_frame_.c_str(),
      robot_radius_);
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
                       sensor_msgs::LaserScan,
                       sensor_msgs::LaserScan>;

  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> sub_front_, sub_back_;
  message_filters::Synchronizer<SyncPolicy> sync_;
  ros::Publisher pub_;
  std::string output_topic_, base_frame_;
  double robot_radius_;

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& front,
                    const sensor_msgs::LaserScan::ConstPtr& back)
  {
    // Wait up to 1.5s for both laser→base_link transforms to appear
    if (!tf_buffer_.canTransform(base_frame_,
                                 front->header.frame_id,
                                 front->header.stamp,
                                 ros::Duration(1.5)) ||
        !tf_buffer_.canTransform(base_frame_,
                                 back->header.frame_id,
                                 back->header.stamp,
                                 ros::Duration(1.5)))
    {
      ROS_WARN_THROTTLE(2.0,
        "Waiting for laser→%s transforms: front='%s', back='%s'",
        base_frame_.c_str(),
        front->header.frame_id.c_str(),
        back->header.frame_id.c_str());
      return;
    }

    // 1) angular resolution check
    double delta = front->angle_increment;
    if (std::abs(delta - back->angle_increment) > 1e-6)
    {
      ROS_WARN_THROTTLE(1.0,
        "Front/back angle_increment differ: %f vs %f",
        delta, back->angle_increment);
    }

    // 2) compute exact beam count for full 360°
    size_t beam_count = static_cast<size_t>(std::round((2.0 * M_PI) / delta));
    double angle_min  = -M_PI;
    double angle_max  = angle_min + delta * (beam_count - 1);

    // 3) prepare merged LaserScan
    sensor_msgs::LaserScan merged;
    merged.header.stamp    = ros::Time::now();
    merged.header.frame_id = base_frame_;
    merged.angle_min       = angle_min;
    merged.angle_max       = angle_max;
    merged.angle_increment = delta;
    merged.time_increment  = 0.0;
    merged.scan_time       = std::max(front->scan_time, back->scan_time);
    merged.range_min       = std::min(front->range_min, back->range_min);
    merged.range_max       = std::max(front->range_max, back->range_max);

    merged.ranges.assign(beam_count,
        std::numeric_limits<float>::infinity());
    merged.intensities.assign(beam_count, 0.0f);

    // 4) lookup transforms (now guaranteed)
    geometry_msgs::TransformStamped tf_front =
      tf_buffer_.lookupTransform(base_frame_,
                                 front->header.frame_id,
                                 front->header.stamp,
                                 ros::Duration(0.1));
    geometry_msgs::TransformStamped tf_back  =
      tf_buffer_.lookupTransform(base_frame_,
                                 back->header.frame_id,
                                 back->header.stamp,
                                 ros::Duration(0.1));

    // 5) project+transform+bin helper
    auto projectAndBin = [&](const sensor_msgs::LaserScan::ConstPtr& scan,
                             const geometry_msgs::TransformStamped& tf)
    {
      geometry_msgs::PointStamped pin, pout;
      pin.header = scan->header;

      for (size_t i = 0; i < scan->ranges.size(); ++i)
      {
        float r = scan->ranges[i];
        if (!std::isfinite(r) ||
            r < scan->range_min ||
            r > scan->range_max)
          continue;

        // point in laser frame
        double ang = scan->angle_min + double(i) * delta;
        pin.point.x = r * std::cos(ang);
        pin.point.y = r * std::sin(ang);
        pin.point.z = 0.0;

        // transform into base_link
        tf2::doTransform(pin, pout, tf);

        // self-filter: drop points within robot_radius_
        double r_base = std::hypot(pout.point.x, pout.point.y);
        if (r_base < robot_radius_)
          continue;

        // convert back to polar in base_link
        double angle_base = std::atan2(pout.point.y, pout.point.x);
        int bin = int(std::floor((angle_base - angle_min)/delta + 0.5));
        if (bin < 0 || bin >= int(beam_count))
          continue;

        // keep closest hit
        if (r_base < merged.ranges[bin])
        {
          merged.ranges[bin]      = r_base;
          merged.intensities[bin] = scan->intensities[i];
        }
      }
    };

    // 6) merge both scans
    projectAndBin(front, tf_front);
    projectAndBin(back,  tf_back);

    // 7) publish merged scan
    pub_.publish(merged);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_merge_tf");
  ScanMergerTF node;
  ros::spin();
  return 0;
}
