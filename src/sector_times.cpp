#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <robot_pkg/SectorTimes.h>
#include <cmath>

struct GpsPoint { double lat, lon; };

class SectorTimesNode
{
    public:
        void getsectorBoundaries()
        {
            std::cout << "s1_end lat,lon: (" << s1_.lat << "," << " " << s1_.lon << ")\n";
            std::cout << "s2_end lat,lon: (" << s2_.lat << "," << " " << s2_.lon << ")\n";
        }

        SectorTimesNode(ros::NodeHandle& nh): nh_(nh), current_sector_(1), sector_start_time_(ros::Time::now()), total_sector_dist_(0)
        {
            // Load sector boundaries from parameters
            nh_.param("sector1_end/lat", s1_.lat, 0.0);
            nh_.param("sector1_end/lon", s1_.lon, 0.0);
            nh_.param("sector2_end/lat", s2_.lat, 0.0);
            nh_.param("sector2_end/lon", s2_.lon, 0.0);
            nh_.param("sector3_end/lat", s3_.lat, 0.0);
            nh_.param("sector3_end/lon", s3_.lon, 0.0);
            nh_.param("finish/lat",      finish_.lat, 0.0);
            nh_.param("finish/lon",      finish_.lon, 0.0);
            nh_.param("defaultmaxdistth", max_dist_th_default, 5.0);
            nh_.param("mindistth", min_dist_th_, 0.2);
            nh_.param("boundaryradius", boundary_radius_, 5.0);

            max_dist_th_ = max_dist_th_default;

            speed_sub_ = nh_.subscribe("/speedsteer", 10, &SectorTimesNode::speedCb, this);
            gps_sub_ = nh_.subscribe("/swiftnav/front/gps_pose", 10, &SectorTimesNode::gpsCb, this);
            pub_ = nh_.advertise<robot_pkg::SectorTimes>("/sector_times", 10);
        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber speed_sub_;
        ros::Subscriber gps_sub_;
        ros::Publisher pub_;

        // state
        double max_dist_th_;
        double max_dist_th_default;
        double min_dist_th_;
        double boundary_radius_;
        int current_sector_;
        ros::Time sector_start_time_;
        double total_sector_dist_;
        geometry_msgs::PointStamped last_gps_;
        bool inside_reg = false;
        bool  is_first_message_ = true;
        double current_speed_sum_; // from /speedsteer
        int speedCount_ = 0;
        ros::Time current_time; // from /speedsteer

        double sectorTimes_[3] = {0, 0, 0};
        double sectorDists_[3] = {0, 0, 0};
        double sectorSpeedSums_[3] = {0, 0, 0};
        double sectorSpeedCounts_[3] = {0, 0, 0};

        // Sector Boundaries
        GpsPoint s1_, s2_, s3_, finish_;

        // simple distance approx: (~111 km per degree lat, cos for lon)
        double gpsDist(const GpsPoint& a, const GpsPoint& b)
        {
            double dlat = (b.lat - a.lat) * 111000.0;
            double dlon = (b.lon - a.lon) * 111000.0 * cos(a.lat * M_PI/180.0);
            return sqrt(dlat*dlat + dlon*dlon);
        } 

        double gpsDist2(const GpsPoint& a, const GpsPoint& b) {
            const double R = 6371000; // Earth radius in meters
            double lat1 = a.lat * M_PI / 180.0;
            double lon1 = a.lon * M_PI / 180.0;
            double lat2 = b.lat * M_PI / 180.0;
            double lon2 = b.lon * M_PI / 180.0;
        
            double dlat = lat2 - lat1;
            double dlon = lon2 - lon1;
        
            double a_ = sin(dlat / 2) * sin(dlat / 2) +
                        cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
            double c = 2 * atan2(sqrt(a_), sqrt(1 - a_));
        
            return R * c;  // distance in meters
        }

        // determine if we've crossed the boundary for sector N
        bool insideBoundaryArea(int sector, const GpsPoint& pos) {
            GpsPoint target = (sector == 1 ? s1_ : (sector == 2 ? s2_ : (sector == 3 ? s3_ : finish_)));
            double d = gpsDist2(pos, target);
            return d < boundary_radius_;  // e.g. within 5 m
        }


        void speedCb(const geometry_msgs::PointStamped::ConstPtr& msg)
        {
            // Get data
            //ROS_INFO("SPEED CALLBACK ENTERED!");
            current_time = msg->header.stamp;
            current_speed_sum_ += (0.277778)*(msg->point.y);     // forward speed (m/s)
            speedCount_ += 1;
        }
        
        void gpsCb(const sensor_msgs::NavSatFix::ConstPtr& fix)
        {
            GpsPoint cur{fix->latitude, fix->longitude};
            ros::Time now = fix->header.stamp;

            if (is_first_message_)
            {
                ROS_INFO("GPS CALLBACK FIRST ENTRY!"); // debug purposed
                last_gps_.point.x = fix->latitude;
                last_gps_.point.y = fix->longitude;
                is_first_message_ = false;
                sector_start_time_ = now;
                return;
            }
        
            // Incremental distance computation
            GpsPoint prev{last_gps_.point.x, last_gps_.point.y};
            double d = gpsDist2(prev, cur);
            //ROS_INFO("distance: %.4f", d);
            if (d < max_dist_th_ && d > min_dist_th_) // ignore small noise and skip large jumps
            {
                max_dist_th_ = max_dist_th_default;
                total_sector_dist_ += d;  
                last_gps_.point.x = fix->latitude;
                last_gps_.point.y = fix->longitude;    
            }
            else
            {
                max_dist_th_ += 5.0; // bad points keep coming, incrementally relieve the distance threshold
                ROS_WARN("Skipped point!"); // debug purposed
                return;
            }
            
            // Check if boundary crossed
            // ROS_INFO("PAST POINT QUALITY CHECK!"); // debug purposed
            double sector_time;
            //double mean_speed;
            bool inside = insideBoundaryArea(current_sector_, cur);
            if (inside && !inside_reg) { // rising edge of "inside"
                ROS_WARN("SECTOR %d EXIT!", current_sector_);
                sector_time = (now - sector_start_time_).toSec();
                //mean_speed  = total_sector_dist_ / sector_time;
                
                int sector_idx = (current_sector_-1)%3; // minus 1 mod 3 cause wrap back to sector 1 if sector 4
                sectorTimes_[sector_idx] += sector_time; 
                sectorDists_[sector_idx] += total_sector_dist_;

                robot_pkg::SectorTimes msg;
                msg.current_sector = sector_idx+1;
                msg.current_sector_time = sectorTimes_[sector_idx];
                msg.current_sector_mean_speed = 3.6 * sectorDists_[sector_idx] / sectorTimes_[sector_idx];
                pub_.publish(msg);
                
                sectorSpeedSums_[sector_idx] += current_speed_sum_;
                sectorSpeedCounts_[sector_idx] += speedCount_;
                ROS_WARN("SPEED FROM /SPEEDSTER: %.4f", 3.6*sectorSpeedSums_[sector_idx]/sectorSpeedCounts_[sector_idx]);

                // Advance to next sector
                current_sector_ = (current_sector_ % 4) + 1;
                // reset sector counters
                sector_start_time_ = now;
                total_sector_dist_ = 0.0;
                is_first_message_ = true;  // reset for next sector
                // reset speed counters
                current_speed_sum_ = 0;
                speedCount_ = 0;
            }
            inside_reg = inside;
            //ROS_INFO("inside_reg: %d", inside_reg);
            //ROS_INFO("Distance: %.2f m | Time: %.2f s | Speed: %.2f m/s", total_sector_dist_, sector_time, mean_speed);

    }
};  

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "sector_times");
    ros::NodeHandle nh;
    SectorTimesNode node(nh);
    node.getsectorBoundaries();
    ros::spin();
    return 0;
}
