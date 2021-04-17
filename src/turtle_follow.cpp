#include "turtle_follow.h"

//Check message types and addons in cmake and package.xml

//Maybe make a file to auto download dependencies like the ar_tags package
//Need to make a launch file for sim and real robots

TurtleFollow::TurtleFollow(ros::NodeHandle nh)
    : nh_(nh)
{
    //need to initialise all the variable values

    //Topic robot_0 might be different------------------------------------------------------
    //Passing by reference (&TurtleFollow) might be a problem-------------------------------
    odomSub_ = nh_.subscribe("/robot_0/odom", 10, &TurtleFollow::odomCallback, this);
    laserSub_ = nh_.subscribe("/robot_0/base_scan", 10, &TurtleFollow::laserCallback, this);
    tagSub_ = nh_.subscribe("/ar_pose_marker", 10, &TurtleFollow::robotControl, this);

    //Topic robot_0 might be different------------------------------------------------------
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1);
}

TurtleFollow::~TurtleFollow()
{
}

void TurtleFollow::tagCallback(const ar_track_alvar::AlvarMarkerConstPtr &msg)
{
    tagPose = msg->pose.pose;
}

void TurtleFollow::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    robot_.ranges_ = msg->ranges;
}

void TurtleFollow::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    robotPose = msg->pose.pose;
    // robot_.mtx_.lock(); //Might need to implement mutexs-------------------------------
    robot_.x_ = pose.position.x; //Might not be needed------------------------------------------------------
    robot_.y_ = pose.position.y; //Might not be needed------------------------------------------------------
    robot_.angle_ = tf::getYaw(pose.orientation); //Might not be needed------------------------------------------------------
    // robot_.mtx_.unlock();
}

void TurtleFollow::robotControl()
{
    // turn_amount = 0.0
    // speed = 0.0

    // auto_speed = 0
    // auto_steering = 127
    // auto_braking = 127

    // target_x = centre_path[0][0] #-------------------------------------------------------------------------------------------------------------------
    // distance_from_centre = target_x - columns/2
    // turn_amount = np.round(np.interp(distance_from_centre, [-200, 200], [0.6, -0.6]), 3)
    // control.drive.acceleration = speed
    // control.drive.steering_angle = turn_amount

    //This needs numpy so probs can't implement unless in cmath or openCV
    // auto_steering = int(np.interp(distance_from_centre, [-200, 200], [0, 255]))

    //Extremities
    // if auto_steering > 255:
    //     auto_steering = 255
    // if auto_steering < 0:
    //     auto_steering = 0



//Psuedo code
    //0 is centre

    //Detect distance from center

    //Use car code to determine steering direction.
    //Might need to change speed depending on turning amount

    //Use lidar to check distance to robot //compare to ar tag distance withing tolerance. //This might be dodgy

    //If within a certain range then stop

    //If lower than range then reverse.

    //Send steering commands to ROS

    robot_.control_.linear.x = 0;
    robot_.control_.angular.z = 0;
    cmd_vel_pub_.publish(robot_.control_);
}

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