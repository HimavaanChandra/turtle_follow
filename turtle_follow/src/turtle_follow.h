/*! @file
 * 
 * @brief Detects an AR tag and uses velocity control and appropriate control algorithms to navigate towards it
 * 
 * @author Himavaan Chandra and Chloe Mallinson
 * @date 11-05-2021
*/
#include "ros/ros.h"

#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>
#include <condition_variable>
#include <vector>

/**
 * @class TurtleFollow
 * @brief Detects and follows an AR tag
 * @details TurtleFollow uses velocity control to navigate towards an AR tag.
 * TurtleFollow subscribes to a series of ROS topics that return AR tag data and laser scans.
 * TurtleFollow uses a combination of Pure Pusuit and a basic position controller to do so.
 */
class TurtleFollow
{
public:
    /**
     * @brief Construct a new Turtle Follow object
     * 
     * @param nh ROS node handle
     */
    TurtleFollow(ros::NodeHandle nh);

    /**
     * @brief Destroy the Turtle Follow object
     * 
     */
    ~TurtleFollow();

    /**
     * @brief Calls the control functions to move robot towards target
     * 
     */
    void robotControl();

    ros::NodeHandle nh_; //!< The ROS node

private:
    /**
     * @brief Callback function for AR tag data
     * 
     * @param msg 
     */
    void tagCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &msg);

    /**
     * @brief Callback function for Laser Scan data
     * 
     * @param msg 
     */
    void laserCallback(const sensor_msgs::LaserScanConstPtr &msg);

    /**
     * @brief Callback function for Robot odometry     * 
     * @param msg 
     */
    void TurtleFollow::odomCallback(const nav_msgs::OdometryConstPtr &msg)

        /**
     * @brief Function to check for obstruction for the safety of the robot
     * 
     * @return true 
     * @return false 
     */
        bool obstructionDetection();

    /**
     * @brief Function to detect AR tag
     * 
     */
    void detection(void);

    /**
     * @brief Basic controller function to navigate towards AR tag when detected
     * 
     * @param centreDistance 
     */
    void basicController(double centreDistance);

    /**
     * @brief Pure pursuit controller function to navigate towards AR tag when detected
     * 
     * @param centreDistance 
     * @param range 
     */
    void purePursuit(double centreDistance, double range);

    /**
     * @brief Control based on image based visual servoing to navigate towards the AR tag when detected
     * 
     * @param 
     */
    void visServo(geometry_msgs::Pose tag_pose_);

    ros::Subscriber laser_sub_;     //!< ROS subscriber variable for Laser Scans
    ros::Subscriber tag_sub_;       //!< ROS subscriber variable for AR Tag data
    ros::Publisher cmd_vel_pub_;    //!< ROS publisher variable for Velocity commands
    geometry_msgs::Pose tag_pose_;  //!< Pose variable for AR tag
    geometry_msgs::Pose pose        //!< Pose variable for Robot Pose
    bool tag_;                      //!< Bool for tag detection

    //!< Struct to capture all Turtlebot3 Waffle data
    struct Robot
    {
        const double radius_ = 0.22;   //!< Radius of the robot
        const double max_linv_ = 0.22; //!< Maximum linear velocity of the robot
        const double max_rotv_ = 2.84; //!< Maximum angular velocity of the robot
        float closest_range_;          //!< Float for quantity of closest range variable
        bool obstacle_;                //!< Bool for whether there is an obstacle ahead or not
        geometry_msgs::Twist twist_{}; //!< Geometry messages twist variable
        std::vector<float> ranges_;    //!< Vector of floats to hold all ranges recorded
    };

    Robot robot_; //!< Robot item
};