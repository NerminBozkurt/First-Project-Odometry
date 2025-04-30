#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <first_project/sector_times.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cmath>

struct GpsPoint {
  double lat, lon;
};

class SectorTimesNode
{
public:
  SectorTimesNode(ros::NodeHandle& nh)
    : nh_(nh)
    //message_filters subscribers
    , speed_sub_(nh_, "/speedsteer", 1)
    , gps_sub_  (nh_, "/swiftnav/front/gps_pose", 1)
    , sync_(SyncPolicy(10), speed_sub_, gps_sub_)
    // init all your state exactly as before
    , current_sector_(1)
    , total_sector_dist_(0.0)
    , current_speed_sum_(0.0)
    , speedCount_(0)
    , inside_reg_(false)
    , is_first_message_(true)
    , saved1_vals_(false)
    , wrap_occurred_(false)
  {
    // load parameters
    nh_.param("sector1_end/lat",       s1_.lat,            0.0);
    nh_.param("sector1_end/lon",       s1_.lon,            0.0);
    nh_.param("sector2_end/lat",       s2_.lat,            0.0);
    nh_.param("sector2_end/lon",       s2_.lon,            0.0);
    nh_.param("sector3_end/lat",       s3_.lat,            0.0);
    nh_.param("sector3_end/lon",       s3_.lon,            0.0);
    nh_.param("finish/lat",            finish_.lat,        0.0);
    nh_.param("finish/lon",            finish_.lon,        0.0);
    nh_.param("defaultmaxdistth",      max_dist_th_default,5.0);
    nh_.param("mindistth",             min_dist_th_,       0.2);
    nh_.param("boundaryradius",        boundary_radius_,   5.0);
    max_dist_th_ = max_dist_th_default;

    //allowed stamp difference 
    sync_.setMaxIntervalDuration(ros::Duration(0.05));  // 50 ms tolerance
    // bind the merged callback
    sync_.registerCallback(
      boost::bind(&SectorTimesNode::syncCb, this, _1, _2)
    );

    pub_   = nh_.advertise<first_project::sector_times>("/sector_times", 10);
    timer_ = nh_.createTimer(ros::Duration(1.0),
                             &SectorTimesNode::timerCb, this);
  }

private:
  typedef geometry_msgs::PointStamped            SpeedMsg;
  typedef sensor_msgs::NavSatFix                 GpsMsg;
  typedef message_filters::sync_policies::ApproximateTime<SpeedMsg, GpsMsg> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy>                              Syncer;

  ros::NodeHandle nh_;
  message_filters::Subscriber<SpeedMsg> speed_sub_;
  message_filters::Subscriber<GpsMsg>   gps_sub_;
  Syncer                                 sync_;
  ros::Publisher                         pub_;
  ros::Timer                             timer_;

  double    max_dist_th_, max_dist_th_default, min_dist_th_, boundary_radius_;
  GpsPoint  s1_, s2_, s3_, finish_;

  int        current_sector_;
  ros::Time  sector_start_time_, wrap_time_;
  double     total_sector_dist_, current_speed_sum_;
  int        speedCount_;
  bool       inside_reg_, is_first_message_;
  geometry_msgs::PointStamped last_gps_;

  double saved1_time_, saved1_speed_sum_;
  int    saved1_speed_count_;
  bool   saved1_vals_;
  bool   wrap_occurred_;

  double gpsDist2(const GpsPoint& a, const GpsPoint& b) {
    const double R = 6.371e6;
    double lat1=a.lat*M_PI/180.0, lon1=a.lon*M_PI/180.0;
    double lat2=b.lat*M_PI/180.0, lon2=b.lon*M_PI/180.0;
    double dlat=lat2-lat1, dlon=lon2-lon1;
    double aa=sin(dlat/2)*sin(dlat/2)+cos(lat1)*cos(lat2)*sin(dlon/2)*sin(dlon/2);
    return R*2*atan2(sqrt(aa),sqrt(1-aa));
  }

  bool insideBoundaryArea(int sector, const GpsPoint& pos) {
    GpsPoint target =
        sector==1 ? s1_
      : sector==2 ? s2_
      : sector==3 ? s3_
                  : finish_;
    return gpsDist2(pos,target) < boundary_radius_;
  }

  void syncCb(const SpeedMsg::ConstPtr& speed_msg,
              const GpsMsg::ConstPtr&   gps_msg)
  {
    // merged speedCb + gpsCb 
    double v = 0.277778 * speed_msg->point.y;
    current_speed_sum_ += v;
    speedCount_++;

    GpsPoint cur{gps_msg->latitude, gps_msg->longitude};
    ros::Time now = gps_msg->header.stamp;

    if (is_first_message_) {
      last_gps_.point.x  = cur.lat;
      last_gps_.point.y  = cur.lon;
      is_first_message_  = false;
      sector_start_time_ = now;
      return;
    }

    double d = gpsDist2(
      GpsPoint{last_gps_.point.x,last_gps_.point.y},cur
    );
    if (d<max_dist_th_ && d>min_dist_th_) {
      max_dist_th_ = max_dist_th_default;
      total_sector_dist_ += d;
      last_gps_.point.x = cur.lat;
      last_gps_.point.y = cur.lon;
    } else {
      max_dist_th_ += 5.0;
      return;
    }

    bool inside = insideBoundaryArea(current_sector_,cur);
    if (inside && !inside_reg_) {
      handleSectorCrossing(now);
    }
    inside_reg_ = inside;
  }

  void handleSectorCrossing(const ros::Time& now) {
    if (current_sector_==1) {
      ros::Duration secT = now - sector_start_time_;
      saved1_time_        = secT.toSec();
      saved1_speed_sum_   = current_speed_sum_;
      saved1_speed_count_ = speedCount_;
      saved1_vals_        = true;
    }
    if (current_sector_!=3) {
      sector_start_time_  = now;
      total_sector_dist_  = 0.0;
      current_speed_sum_  = 0.0;
      speedCount_         = 0;
    } else {
      wrap_time_        = now;
      wrap_occurred_    = true;
      sector_start_time_= now;
      total_sector_dist_= 0.0;
      current_speed_sum_= 0.0;
      speedCount_       = 0;
    }
    current_sector_ = (current_sector_%3)+1;
  }

  void timerCb(const ros::TimerEvent&) {
    ros::Time now = ros::Time::now();
    double elapsed_sec, mean_speed;
    if (current_sector_==1 && saved1_vals_ && wrap_occurred_) {
      double newT = (now-wrap_time_).toSec();
      elapsed_sec = saved1_time_ + newT;
      int totalCount = saved1_speed_count_ + speedCount_;
      double totalSum = saved1_speed_sum_ + current_speed_sum_;
      mean_speed = totalCount>0 ? 3.6*totalSum/totalCount : 0.0;
    } else {
      ros::Duration d = now-sector_start_time_;
      elapsed_sec = d.toSec();
      mean_speed = elapsed_sec>0 ? 3.6*total_sector_dist_/elapsed_sec : 0.0;
    }
    first_project::sector_times msg;
    msg.current_sector            = current_sector_;
    msg.current_sector_time       = elapsed_sec;
    msg.current_sector_mean_speed = mean_speed;
    pub_.publish(msg);
  }
};

int main(int argc,char**argv){
  ros::init(argc,argv,"sector_times");
  ros::NodeHandle nh;
  SectorTimesNode node(nh);
  ros::spin();
  return 0;
}
