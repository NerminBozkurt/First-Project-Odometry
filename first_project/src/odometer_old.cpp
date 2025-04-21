#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <tf/transform_broadcaster.h>

class odometer
{
    const double pi = 3.14159265358979323846;
    float pos_x = 0.0;
    float pos_y = 0.0;
    float omega = 0.0;
    float x_steer_in = 0.0;
    float y_speed = 0.0;
    float theta = 0.0;
    float baseline = 1.765;
    float track = 1.30;
    float steering_factor = 32.0;
    float x_steer_out = 0.0;
    float speed_long = 0.0;
    float speed_lat = 0.0;
    float dt = 0.1;
    float pos_x_prev = 0.0;
    float pos_y_prev = 0.0;
    float theta_prev = 0.0;
    nav_msgs::Odometry msg2;
    tf::Transform transform;
    tf::Quaternion q;
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Timer timer1;
    tf::TransformBroadcaster br;

public:
    odometer()
    {
        sub = n.subscribe("/speedsteer", 1, &odometer::callback, this);
        pub = n.advertise<nav_msgs::Odometry>("/odom", 1);
        timer1 = n.createTimer(ros::Duration(dt), &odometer::callback1, this);
    }

    void callback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        x_steer_in = msg->point.x;
        y_speed = msg->point.y;
        x_steer_out = x_steer_in / steering_factor;

        // Use radians for trig functions
        theta = y_speed * sin(x_steer_out) / baseline *dt + theta_prev;
        pos_x = y_speed * cos(theta) * dt + pos_x_prev;
        pos_y = y_speed * sin(theta) * dt + pos_y_prev;

        // Store previous values for integration
        theta_prev = theta;
        pos_x_prev = pos_x;
        pos_y_prev = pos_y;

        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = ros::Time::now();
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";

        tf_msg.transform.translation.x = pos_x;
        tf_msg.transform.translation.y = pos_y;
        tf_msg.transform.translation.z = 0.0;

        tf::Quaternion quat = tf::createQuaternionFromYaw(theta);
        tf_msg.transform.rotation.x = quat.x();
        tf_msg.transform.rotation.y = quat.y();
        tf_msg.transform.rotation.z = quat.z();
        tf_msg.transform.rotation.w = quat.w();

        br.sendTransform(tf_msg);

    }

    void callback1(const ros::TimerEvent&)
    {
        msg2.header.stamp = ros::Time::now();
        msg2.header.frame_id = "odom";
        msg2.child_frame_id = "base_link";

        msg2.pose.pose.position.x = pos_x;
        msg2.pose.pose.position.y = pos_y;
        msg2.pose.pose.orientation  =tf::createQuaternionMsgFromYaw(theta);

        pub.publish(msg2);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "first_node");
    odometer my_odometer;
    ros::spin();
    return 0;
}
