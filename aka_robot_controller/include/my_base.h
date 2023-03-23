#ifndef MY_BASE_H
#define MY_BASE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

class myBase
{
  public:
    myBase();
    void velCallback(const geometry_msgs::Twist& vel);

  private:
    ros::NodeHandle nh_;
    ros::Publisher odom_publisher_;
    ros::Subscriber vel_subscriber_;
    tf2::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_trans;
    tf2_ros::TransformBroadcaster odom_broadcaster_;
    nav_msgs::Odometry odom;

    float linear_vel_x_;
    float linear_vel_y_;
    float angular_vel_z_;
    ros::Time last_vel_time_;
    float vel_dt_;
    float x_pos_;
    float y_pos_;
    float heading_;
};

#endif