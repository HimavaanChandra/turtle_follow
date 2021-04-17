#include "ros/ros.h"

//Remove ones that are not used
//Move these to .h maybe?
#include "ar_track_alvar/AlvarMarkers.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

//Probs not needed
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//Cleanup
// #include <sstream>
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
  /*! @brief Pursuit constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
  TurtleFollow(ros::NodeHandle nh);
  ~TurtleFollow();

  void pursuit(void);
  void purePursuit(void);
  void checkGoal(void);
  bool pointChecker(double x_target, double y_target);

public:
  ros::NodeHandle nh_;

private:
  void laserCallback(const sensor_msgs::LaserScanConstPtr &msg);
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);

  double robotx_; /*!< robot x coord in pixel coords (100)*/
  double roboty_; /*!< robot y coord in pixel coords (100)*/
  ros::Subscriber sub1_;
  ros::Subscriber sub2_;
  ros::Publisher cmd_vel_pub_; /*!< Velocity publisher */

  struct Robot
  {
    double x_; /*!< robot's x coordinate*/
    double y_; /*!< robot's y coordinate*/
    double angle_; /*!< robot's angle */
    bool obstacle_; /*!< boolean if there is an object 0.2m in front of the robot*/
    geometry_msgs::Twist control_; /*!< Controller of the robot*/
    std::vector<float> ranges_; /*!< Laser sensor data*/
    std::mutex mtx_; /*!< mutex to lock data*/
  };

  Robot robot_; /*!< structure of robot's data*/
}