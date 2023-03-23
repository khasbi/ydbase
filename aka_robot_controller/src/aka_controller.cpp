#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

float linear_x;
float linear_y;
float angular_z;

double x = 0.0;
double y = 0.0;
double th = 0.0;

void velCallback(const geometry_msgs::Twist& vel)
{
    linear_x = vel.linear.x;
    linear_y = 0;   //diff drive
    angular_z = vel.angular.z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aka_controller");

    ros::NodeHandle nh;
    ros::Subscriber sub_vel = nh.subscribe("raw_vel", 50, velCallback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(1.0);
    while (nh.ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();
        double delta_x = (linear_x * cos(th) - linear_y * sin(th)) * dt;
        double delta_y = (linear_x * sin(th) - linear_y * cos(th)) * dt;
        double delta_th = angular_z * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //publish odometry messages over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = linear_x;
        odom.twist.twist.linear.y = linear_y;
        odom.twist.twist.angular.z = angular_z;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }   
    

    
}