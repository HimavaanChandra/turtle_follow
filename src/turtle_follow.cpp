#include "turtle_follow.h"

//Check message types in cmake and package.xml

//Maybe make a file to auto download dependencies like the ar_tags package

//Receive ar tag positions from ROS subscriber
//Save x position  (y and z not needed) into variables
Pursuit::Pursuit(ros::NodeHandle nh)
    : nh_(nh)
{

    sub1_ = nh_.subscribe("/robot_0/odom", 10, &Pursuit::odomCallback, this);
    sub2_ = nh_.subscribe("/robot_0/base_scan", 10, &Pursuit::laserCallback, this);

    //Topic robot_0 might be different------------------------------------------------------
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1);
}

Pursuit::~Pursuit()
{
}

void Pursuit::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    geometry_msgs::Pose pose = msg->pose.pose; //Make class variable
    robot_.mtx_.lock();
    robot_.x_ = pose.position.x;
    robot_.y_ = pose.position.y;
    robot_.angle_ = tf::getYaw(pose.orientation);
    robot_.mtx_.unlock();
}

void robotControl()
{
    robot_.control_.linear.x = 0;
    robot_.control_.angular.z = 0;
    cmd_vel_pub_.publish(robot_.control_);
}

//0 is centre

//Detect distance from center

//Use car code to determine steering direction.
//Might need to change speed depending on turning amount

//Use lidar to check distance to robot //compare to ar tag distance withing tolerance. //This might be dodgy

//If within a certain range then stop

//If lower than range then reverse.

//Send steering commands to ROS

//-----Should implement-----
//Use orientation to keep robot perpendicular

//For locating bot in the first place --
//Use image detection + lidar
//If any lidar values change then:
//Predict velocity
//Predict future location
//Turn to location
//Image detection

//Use ar tag pose + lidar to path plan to ar tag
//pure pursuit to follow