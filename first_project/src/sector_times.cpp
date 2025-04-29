#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <first_project/sector_times.h>
#include <cmath>

struct GpsPoint { double lat, lon; };

class SectorTimesNode
{
public:
    SectorTimesNode(ros::NodeHandle& nh)
      : nh_(nh)
      , current_sector_(1)
      , sector_start_time_(ros::Time::now())
      , total_sector_dist_(0.0)
      , current_speed_sum_(0.0)
      , speedCount_(0)
      , inside_reg(false)
      , is_first_message_(true)
      , saved1_vals_(false)
      , wrap_occurred_(false)
    {
        // Load sector boundaries from parameters
        nh_.param("sector1_end/lat", s1_.lat, 0.0);
        nh_.param("sector1_end/lon", s1_.lon, 0.0);
        nh_.param("sector2_end/lat", s2_.lat, 0.0);
        nh_.param("sector2_end/lon", s2_.lon, 0.0);
        nh_.param("sector3_end/lat", s3_.lat, 0.0);
        nh_.param("sector3_end/lon", s3_.lon, 0.0);
        nh_.param("finish/lat", finish_.lat, 0.0);
        nh_.param("finish/lon", finish_.lon, 0.0);
        nh_.param("defaultmaxdistth", max_dist_th_default, 5.0);
        nh_.param("mindistth", min_dist_th_, 0.2);
        nh_.param("boundaryradius", boundary_radius_, 5.0);
        max_dist_th_ = max_dist_th_default;

        speed_sub_ = nh_.subscribe("/speedsteer", 10, &SectorTimesNode::speedCb, this);
        gps_sub_   = nh_.subscribe("/swiftnav/front/gps_pose", 10, &SectorTimesNode::gpsCb, this);
        pub_       = nh_.advertise<first_project::sector_times>("/sector_times", 10);

        timer_ = nh_.createTimer(ros::Duration(1.0), &SectorTimesNode::timerCb, this);
    }

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber speed_sub_, gps_sub_;
    ros::Publisher  pub_;
    ros::Timer      timer_;

    // state
    double max_dist_th_, max_dist_th_default, min_dist_th_, boundary_radius_;
    int    current_sector_;
    ros::Time sector_start_time_, wrap_time_;
    double total_sector_dist_, current_speed_sum_;
    int    speedCount_;
    bool   inside_reg, is_first_message_;
    geometry_msgs::PointStamped last_gps_;

    // snapshot for sector1
    double saved1_time_;
    double saved1_speed_sum_;
    int    saved1_speed_count_;
    bool   saved1_vals_;
    bool   wrap_occurred_;

    // boundaries
    GpsPoint s1_, s2_, s3_, finish_;

    // Haversine
    double gpsDist2(const GpsPoint& a, const GpsPoint& b) {
        const double R = 6371000;
        double lat1=a.lat*M_PI/180.0, lon1=a.lon*M_PI/180.0;
        double lat2=b.lat*M_PI/180.0, lon2=b.lon*M_PI/180.0;
        double dlat=lat2-lat1, dlon=lon2-lon1;
        double aa=sin(dlat/2)*sin(dlat/2)+cos(lat1)*cos(lat2)*sin(dlon/2)*sin(dlon/2);
        return R*2*atan2(sqrt(aa),sqrt(1-aa));
    }

    bool insideBoundaryArea(int sector, const GpsPoint& pos) {
        GpsPoint target = (sector==1? s1_ : sector==2? s2_ : sector==3? s3_ : finish_);
        return gpsDist2(pos,target) < boundary_radius_;
    }

    void speedCb(const geometry_msgs::PointStamped::ConstPtr& msg) {
        current_speed_sum_ += 0.277778*msg->point.y;
        speedCount_++;
    }

    void gpsCb(const sensor_msgs::NavSatFix::ConstPtr& fix) {
        GpsPoint cur{fix->latitude, fix->longitude};
        ros::Time now = fix->header.stamp;

        if (is_first_message_) {
            last_gps_.point.x = cur.lat;
            last_gps_.point.y = cur.lon;
            is_first_message_ = false;
            sector_start_time_ = now;
            return;
        }

        double d=gpsDist2(GpsPoint{last_gps_.point.x,last_gps_.point.y},cur);
        if (d<max_dist_th_ && d>min_dist_th_) {
            max_dist_th_=max_dist_th_default;
            total_sector_dist_+=d;
            last_gps_.point.x=cur.lat;
            last_gps_.point.y=cur.lon;
        } else { max_dist_th_+=5.0; return; }

        bool inside=insideBoundaryArea(current_sector_,cur);
        if (inside && !inside_reg) {
            // 1->2: snapshot sector1
            if (current_sector_==1) {
                ros::Duration secT = now-sector_start_time_;
                saved1_time_=secT.toSec();
                saved1_speed_sum_=current_speed_sum_;
                saved1_speed_count_=speedCount_;
                saved1_vals_=true;
            }
            // reset for 2 and 3
            if (current_sector_!=3) {
                sector_start_time_=now;
                total_sector_dist_=0.0;
                current_speed_sum_=0.0;
                speedCount_=0;
            } else if (current_sector_==3) {
                // on 3->1 wrap
                wrap_time_=now;
                wrap_occurred_=true;
                // reset for new lap
                sector_start_time_=now;
                total_sector_dist_=0.0;
                current_speed_sum_=0.0;
                speedCount_=0;
            }
            // advance
            current_sector_ = (current_sector_%3)+1;
        }
        inside_reg=inside;
    }

    void timerCb(const ros::TimerEvent&) {
        ros::Time now=ros::Time::now();
        double elapsed_sec;
        double mean_speed;
        if (current_sector_==1 && saved1_vals_ && wrap_occurred_) {
            double newT=(now-wrap_time_).toSec();
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
        msg.current_sector=current_sector_;
        msg.current_sector_time=elapsed_sec;
        msg.current_sector_mean_speed=mean_speed;
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
