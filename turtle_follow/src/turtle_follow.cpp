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
  robot_.obstacle_ = false;
  if (robot_.ranges_.size() > 0)
  {
    for (unsigned int i = 0; i < robot_.ranges_.size(); i++)
    {
      //The +-25 is the window of detection -- Can move this to the for loop above to make neater------------------------------------------------
      if (robot_.ranges_.at(i) < robot_.radius_ && robot_.ranges_.at(i) > 0)
      {
        if (((i >= (robot_.ranges_.size() - 15)) && i <= robot_.ranges_.size() - 1) || (i >= 0 && i <= 15))
        {
          ROS_INFO_STREAM("Obstruction Ahead!");
          robot_.obstacle_ = true;
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
    robot_.control_.linear.x = 0.10;
    robot_.control_.angular.z = 0;
  }
  //To the left
  else if (centreDistance < -0.1)
  {
    robot_.control_.linear.x = 0.11;
    robot_.control_.angular.z = 0.5;
  }
  //To the right
  else if (centreDistance > 0.1)
  {
    robot_.control_.linear.x = 0.11;
    robot_.control_.angular.z = -0.5;
  }
  //Do nothing
  else
  {
    robot_.control_.linear.x = 0;
    robot_.control_.angular.z = 0;
  }
}

void TurtleFollow::purePursuit(double centreDistance, double range)
{
  // Maximum translational velocity	Burger = 0.22 m/s	Waffle = 0.26 m/s
  // Maximum rotational velocity	Burger = 2.84 rad/s (162.72 deg/s)	Waffle = 1.82 rad/s (104.27 deg/s)
  double rotv = 2.84;
  double gamma = (2 * range * std::sin(centreDistance)) / std::pow(range, 2);
  double linear_velocity_ = std::sqrt((6 * std::pow(9.81, 2)) / std::abs(gamma));
  double angular_velocity_ = linear_velocity_ * gamma;

  if (linear_velocity_ > robot_.max_vel_)
  {
    linear_velocity_ = robot_.max_vel_;
  }

  while (linear_velocity_ > robot_.max_vel_ || angular_velocity_ > rotv)
  {
    angular_velocity_ *= 0.99;
    linear_velocity_ *= 0.99;
  }

  robot_.control_.linear.x = linear_velocity_;
  robot_.control_.angular.z = angular_velocity_;

  //PMS code
  // double lookahead = 10;
  // target_range_ = target_bogie.at(1);
  // double gamma = (2 * (target_range_ + lookahead) * std::sin(target_bogie[0])) / std::pow(target_range_, 2);

  // std::lock_guard<std::mutex> lock(access_);
  // linear_velocity_ = std::sqrt((6 * std::pow(9.81, 2)) / std::abs(gamma));
  // angular_velocity_ = linear_velocity_ * gamma;

  // if (linear_velocity_ < sim->V_TERM)
  // {
  //     linear_velocity_ = sim->V_TERM;
  // }
  // else if (linear_velocity_ > sim->MAX_V)
  // {
  //     linear_velocity_ = sim->MAX_V;
  // }

  // double g_force = std::abs(linear_velocity_ * angular_velocity_ / 9.81);

  // while (std::abs(g_force) > sim->MAX_G)
  // {
  //     angular_velocity_ *= 0.99;
  //     linear_velocity_ *= 0.99;
  //     g_force = linear_velocity_ * angular_velocity_ / 9.81;
  // }
  ///////////
}

void TurtleFollow::visServo(double centreDistance)
{
  // Visual Servoing
  double linear_velocity_ = 0;
  double angular_velocity_ = 0;
  double lambda = 0.01;
  // 1.93
  cv::Mat focalLength = [1408.83; 1409.15];
  cv::Mat prinPoint = [980.52; 521.50];

  // Find the centre of the AR tag
  double x = tag_pose_.position.z;
  double y = tag_pose_.position.x;
  double z = tag_pose_.position.y;
  double Z = 0.2;
  cv::Mat arPose = [x y z];
  double oriW = tag_pose_.orientation.w;
  double oriX = tag_pose_.orientation.x;
  double oriY = tag_pose_.orientation.y;
  double oriZ = tag_pose_.orientation.z;
  tf::Quaternion q(oriX, oriY, OriZ, oriW);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  // When tag pose x = 0, AR tag is in centre of screen
  // Tag pose x is y camera
  // Tag pose z is x camera
  cv::Mat target = [x; y; Z; 1]; //Might need to change Z to lowercase z if followiing is not working ----------------------------------------

  // Transform ros ar to camera frame --------------------------------------------------------------------------------------------------------

  // Find linear and angular velocity to navigate centre of AR tag to the centre of camera frame using visual servoing
  cv::Mat imTarget = (target - prinPoint) / focalLength;
  cv::Mat ar3D = (arPose - prinPoint) / focalLength;

  // Calculate velocity matrix/feature Jacobian
  cv::Mat Lxi;
  cv::Mat Lx; //Make vector of matricies then just to a pushback -----------------------------------------------------------------------------
  double n = size(imTarget);
  for (int i = 0; i < n; i++)
  {
    // You are accessing positions that arent defined/set yet --------------------------------------------------------------------------------
    Lxi[1, 1] = -1 / Z;
    Lxi[1, 2] = 0;
    Lxi[1, 3] = x / Z;
    Lxi[1, 4] = x * y;
    Lxi[1, 5] = -(1 + x ^ 2);
    Lxi[1, 6] = y;

    Lxi[2, 1] = 0;
    Lxi[2, 2] = -1 / Z;
    Lxi[2, 3] = y / Z;
    Lxi[2, 4] = 1 + y ^ 2;
    Lxi[2, 5] = -x * y;
    Lxi[2, 6] = -x;

    Lx = [Lx; Lxi];
  }

  // Calculate position error
  cv::Mat error2 = ar3D - imTarget;
  cv::Mat error = cv::reshape(error2.t(), [], 1); //Look up the function ----------------------------------------------------------------------
  cv::Mat deltaError = -error * lambda;

  // Calculate velocity matrix
  cv::Mat Lx2;
  cv::Mat Lx2 = pow((Lx.t() * Lx), -1) * Lx.t();
  cv::Mat velocity;
  cv::Mat velocity = -lambda * Lx2 * error;
  double linear_velocity_ = velocity[1, 1];
  double angular_velocity_ = velocity[2, 1];

  // Published to ros in robotControl
  robot_.control_.linear.x = linear_velocity_;
  robot_.control_.angular.z = angular_velocity_;
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
      basicController(tag_pose_.position.x);
      // purePursuit(tag_pose_.position.x, tag_pose_.position.z); //-------------------------------------------
    }
    else
    {
      std::cout << "Turning to find tag" << std::endl;
      robot_.twist_.linear.x = 0;
      robot_.twist_.angular.z = 0.1;
    }

    if (obstructionDetection())
    {
      std::cout << "Obstruction" << std::endl;
      robot_.twist_.linear.x = 0;
      robot_.twist_.angular.z = 0;
    }
    //Need to add reversing if too close------------------------------------------------
    std::cout << "Linear: " << robot_.twist_.linear.x << std::endl;
    std::cout << "Angular: " << robot_.twist_.angular.z << std::endl;

    cmd_vel_pub_.publish(robot_.twist_);
    rate.sleep();
  }
}