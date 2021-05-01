#include "ros/ros.h"

//Remove ones that are not used
#include "ar_track_alvar/AlvarMarker.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

//Probs not needed---------------------------------------------
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//Cleanup--------------------------------------------------------------
#include <iostream>
#include <string>
// #include <thread>
// #include <chrono>
// #include <mutex>
#include <random>
#include <cmath>
// #include <condition_variable>
#include <vector>

//Delete what is not used-------------------------------------------------
class TurtleFollow
{
public:
    TurtleFollow(ros::NodeHandle nh);
    ~TurtleFollow();

    void robotControl(); //might need to go in private functions-----------------------------

    ros::NodeHandle nh_;

private:
    void tagCallback(const ar_track_alvar::AlvarMarkerConstPtr &msg);
    void laserCallback(const sensor_msgs::LaserScanConstPtr &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    bool obstructionDetection();
    void detection(void);
    void basicController(double centreDistance, double range);
    void purePursuit(double centreDistance, double range);
    void visServo(double centreDistance);

    ros::Subscriber odom_sub_;
    ros::Subscriber laser_sub_;
    ros::Subscriber tag_sub_;
    ros::Publisher cmd_vel_pub_;
    geometry_msgs::Pose tag_pose_;
    

    struct Robot
    {
        const double radius_ = 0.22;
        const double max_vel_ = 0.22;
        bool obstacle_;
        geometry_msgs::Twist control_;
        std::vector<float> ranges_;
        std::mutex mtx_; //Do we need to multithread?-------------------------------
    };

    Robot robot_;
}