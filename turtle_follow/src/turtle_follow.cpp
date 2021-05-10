#include "turtle_follow.h"

//Check message types and addons in cmake and package.xml

//Maybe make a file to auto download dependencies like the ar_tags package
//Need to make a launch file for sim and real robots

TurtleFollow::TurtleFollow(ros::NodeHandle nh)
    : nh_(nh)
{
  //need to initialise all the variable values-------------------------------------------------------------
  //Passing by reference (&TurtleFollow) might be a problem-------------------------------
  odom_sub_ = nh_.subscribe("/odom", 10, &TurtleFollow::odomCallback, this); //Probs delete------
  laser_sub_ = nh_.subscribe("/scan", 10, &TurtleFollow::laserCallback, this);
  tag_sub_ = nh_.subscribe("/ar_pose_marker", 10, &TurtleFollow::tagCallback, this);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  tag_ = false;
}

TurtleFollow::~TurtleFollow()
{
}

void TurtleFollow::tagCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &msg)
{
  tag_ = false;
  // const struct ar_track_alvar_msgs::AlvarMarkers_<std::allocator<void> >
  // ar_track_alvar_msgs::AlvarMarker[] markers = msg->markers;
  // std::vector<ar_track_alvar_msgs::AlvarMarker> test = msg->markers;
  if (msg->markers.size() > 0)
  {
    tag_pose_ = msg->markers.at(0).pose.pose;
    tag_ = true;
  }
  // tag_pose_ = msg->pose.pose;
}

void TurtleFollow::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
  robot_.ranges_ = msg->ranges;
}

//May not need------------------------------------------------------------------------
void TurtleFollow::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  geometry_msgs::Pose pose = msg->pose.pose;
}

bool TurtleFollow::obstructionDetection()
{
  robot_.closest_range_ = 100; //Set high so can be overwritten
  robot_.obstacle_ = false;
  if (robot_.ranges_.size() > 0)
  {
    for (unsigned int i = 0; i < robot_.ranges_.size(); i++)
    {
      //The +-15 degrees is the window of detection for a forward obstruction
      if (robot_.ranges_.at(i) < robot_.radius_ && robot_.ranges_.at(i) > 0)
      {
        if (((i >= (robot_.ranges_.size() - 15)) && i <= robot_.ranges_.size() - 1) || (i >= 0 && i <= 15))
        {
          ROS_INFO_STREAM("Obstruction Ahead!");
          robot_.obstacle_ = true;
          if (robot_.ranges_.at(i) < robot_.closest_range_)
          {
            robot_.closest_range_ = robot_.ranges_.at(i);
          }
        }
      }
    }
  }
  else
  {
    robot_.obstacle_ = false;
  }
  return robot_.obstacle_;
}

void TurtleFollow::basicController(double centreDistance)
{
  // Maximum translational velocity	Burger = 0.22 m/s	Waffle = 0.26 m/s
  // Maximum rotational velocity	Burger = 2.84 rad/s (162.72 deg/s)	Waffle = 1.82 rad/s (104.27 deg/s)

  //Adjust tolerances
  //Within tolerance
  if (centreDistance >= -0.1 && centreDistance <= 0.1)
  {
    robot_.twist_.linear.x = 0.10;
    robot_.twist_.angular.z = 0;
  }
  //To the left
  else if (centreDistance < -0.1)
  {
    robot_.twist_.linear.x = 0.11;
    robot_.twist_.angular.z = 0.5;
  }
  //To the right
  else if (centreDistance > 0.1)
  {
    robot_.twist_.linear.x = 0.11;
    robot_.twist_.angular.z = -0.5;
  }
  //Do nothing
  else
  {
    robot_.twist_.linear.x = 0;
    robot_.twist_.angular.z = 0;
  }
}

void TurtleFollow::purePursuit(double centreDistance, double range)
{
  // Maximum translational velocity	Burger = 0.22 m/s	Waffle = 0.26 m/s
  // Maximum rotational velocity	Burger = 2.84 rad/s (162.72 deg/s)	Waffle = 1.82 rad/s (104.27 deg/s)
  double gamma = (2 * std::sin(centreDistance)) / std::pow(range, 2);
  // double linear_velocity_ = std::sqrt((6 * std::pow(9.81, 2)) / std::abs(gamma));
  double linear_velocity_ = 0.22;
  double angular_velocity_ = linear_velocity_ * gamma * 5;
  if (gamma < 0)
  {
    if (angular_velocity_ < 0)
    {
      angular_velocity_ = -angular_velocity_;
    }
  }
  else
  {
    if (angular_velocity_ > 0)
    {
      angular_velocity_ = -angular_velocity_;
    }
  }
  //Remove cout -----------------------------------------------------------------------------------------
  std::cout << "gamma: " << gamma << " linear: " << linear_velocity_ << " angular: " << angular_velocity_ << std::endl;
  if (linear_velocity_ > robot_.max_linv_)
  {
    linear_velocity_ = robot_.max_linv_;
  }

  while (linear_velocity_ > robot_.max_linv_ || angular_velocity_ > robot_.max_rotv_)
  {
    angular_velocity_ *= 0.99;
    linear_velocity_ *= 0.99;
  }

  robot_.twist_.linear.x = linear_velocity_;
  robot_.twist_.angular.z = angular_velocity_;
}

void TurtleFollow::visServo(double centreDistance)
{
  // // Visual Servoing
  // double linear_velocity_ = 0;
  // double angular_velocity_ = 0;
  // double lambda = 0.01;
  // // 1.93
  // // cv::Mat<double> focalLength = [1408.83, 1409.15];
  // std::vector<double> focalLength = [1408.83, 1409.15];
  // // cv::Mat<double> prinPoint = [980.52, 521.50];
  // std::vector<double> prinPoint = [980.52, 521.50];

  // // Find the centre of the AR tag
  // double x = tag_pose_.position.z;
  // double y = tag_pose_.position.x;
  // double z = tag_pose_.position.y;
  // double Z = 0.2;
  // // cv::Mat<double> arPose = (x, y, z);
  // std::vector<double> arPose = [x, y, z];
  // double oriW = tag_pose_.orientation.w;
  // double oriX = tag_pose_.orientation.x;
  // double oriY = tag_pose_.orientation.y;
  // double oriZ = tag_pose_.orientation.z;
  // tf::Quaternion q(oriX, oriY, oriZ, oriW);
  // tf::Matrix3x3 m(q);
  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  // // When tag pose x = 0, AR tag is in centre of screen
  // // Tag pose x is y camera
  // // Tag pose z is x camera
  // // cv::Mat<double> target = (x, y, Z, 1); //Might need to change Z to lowercase z if followiing is not working ----------------------------------------
  // std::vector<double> target = (x, y, z, 1);

  // // Transform ros ar to camera frame --------------------------------------------------------------------------------------------------------

  // // Find linear and angular velocity to navigate centre of AR tag to the centre of camera frame using visual servoing
  // // cv::Mat<double> imTarget = (target - prinPoint) / focalLength;
  // std::vector<double> imTarget = (target - prinPoint) / focalLength;
  // // cv::Mat<double> ar3D = (arPose - prinPoint) / focalLength;
  // std::vector<double> ar3D = (arPose - prinPoint) / focalLength;

  // // Calculate velocity matrix/feature Jacobian
  // cv::Mat<double> Lxi(2, 6); // Is this how you set the size of a matrix ?? I'm struggling to find an example that I understand --------------------
  // std::vector<double> Lxi;
  // std::vector<std::vector<double>> Lx;
  // int n = size(imTarget);
  // for (int i = 0; i < n; i+1)
  // {
  //   // You are accessing positions that arent defined/set yet --------------------------------------------------------------------------------
  //   Lxi[1, 1] = -1 / Z;
  //   Lxi[1, 2] = 0;
  //   Lxi[1, 3] = x / Z;
  //   Lxi[1, 4] = x * y;
  //   Lxi[1, 5] = -(1 + pow(x, 2));
  //   Lxi[1, 6] = y;

  //   Lxi[2, 1] = 0;
  //   Lxi[2, 2] = -1 / Z;
  //   Lxi[2, 3] = y / Z;
  //   Lxi[2, 4] = 1 + pow(y, 2);
  //   Lxi[2, 5] = -x * y;
  //   Lxi[2, 6] = -x;

  //   Lx.push_back(Lxi);
  // }

  // // Calculate position error
  // // cv::Mat<double> error2 = ar3D - imTarget;
  // std::vector<double> error2 = ar3D - imTarget;
  // // cv::Mat<double> err;
  // std::vector<double> err;
  // err = error2.resize(1, 6); // I'm not sure if this is right still tbh ??? -------------------------------------------------------------
  // // cv::Mat<double> deltaError = -err * lambda;
  // std::vector<double> deltaError = -err * lambda;

  // // Calculate velocity matrix
  // // cv::Mat<double> Lx2;
  // std::vector<double> Lx2;
  // Lx2 = pow((Lx.t() * Lx), -1) * Lx.t();
  // // cv::Mat<double> velocity;
  // std::vector<double> velocity;
  // velocity = -(lambda) * Lx2 * err;
  // linear_velocity_ = velocity.at(1, 1);
  // angular_velocity_ = velocity.at(2, 1);

  // // Published to ros in robotControl
  // robot_.twist_.linear.x = linear_velocity_;
  // robot_.twist_.angular.z = angular_velocity_;
}

void TurtleFollow::robotControl()
{
  ros::Rate rate(10);
  while (ros::ok())
  {
    std::cout << "Robot Control" << std::endl;
    if (tag_)
    {
      std::cout << "Tag Detected" << std::endl;
      // basicController(tag_pose_.position.x);
      purePursuit(tag_pose_.position.x, tag_pose_.position.z); //-------------------------------------------
    }
    else
    {
      std::cout << "Turning to find tag" << std::endl;
      robot_.twist_.linear.x = 0;
      robot_.twist_.angular.z = 0.5;
    }

    if (obstructionDetection())
    {
      std::cout << "Obstruction" << std::endl;
      robot_.twist_.linear.x = 0;
      robot_.twist_.angular.z = 0;
      if (robot_.closest_range_ < robot_.radius_ - 0.1)
      {
        std::cout << "Reverse" << std::endl;
        purePursuit(tag_pose_.position.x, tag_pose_.position.z);
        robot_.twist_.linear.x = -robot_.twist_.linear.x/5;
      }
    }
    //Need to add reversing if too close------------------------------------------------
    std::cout << "Linear: " << robot_.twist_.linear.x << std::endl;
    std::cout << "Angular: " << robot_.twist_.angular.z << std::endl;

    cmd_vel_pub_.publish(robot_.twist_);
    rate.sleep();
  }
}