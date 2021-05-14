#include "turtle_follow.h"

TurtleFollow::TurtleFollow(ros::NodeHandle nh)
    : nh_(nh)
{
  // Subscribe to ros topics and set variables
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
  if (msg->markers.size() > 0)
  {
    tag_pose_ = msg->markers.at(0).pose.pose;
    tag_ = true;
  }
}

void TurtleFollow::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
  robot_.ranges_ = msg->ranges;
}

bool TurtleFollow::obstructionDetection()
{
  // Closest range is set to a high value so it can be over written later on
  robot_.closest_range_ = 100;
  robot_.obstacle_ = false;
  if (robot_.ranges_.size() > 0)
  {
    for (unsigned int i = 0; i < robot_.ranges_.size(); i++)
    {
      // The +-15 degrees is the window of detection for a forward obstruction
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
  // AR tag is within 0.1 tolerance of view
  if (centreDistance >= -0.1 && centreDistance <= 0.1)
  {
    robot_.twist_.linear.x = 0.10;
    robot_.twist_.angular.z = 0;
  } 
  // Tag to the left 
  else if (centreDistance < -0.1)
  {
    robot_.twist_.linear.x = 0.11;
    robot_.twist_.angular.z = 0.5;
  }
  // Tag to the right
  else if (centreDistance > 0.1)
  {
    robot_.twist_.linear.x = 0.11;
    robot_.twist_.angular.z = -0.5;
  }
  // If there is no relevant input, do nothing
  else
  {
    robot_.twist_.linear.x = 0;
    robot_.twist_.angular.z = 0;
  }
}

void TurtleFollow::purePursuit(double centreDistance, double range)
{
  // Calculating maximum angular velocity for velocity control
  double gamma = (2 * std::sin(centreDistance)) / std::pow(range, 2);
  double linear_velocity = 0.22;
  double angular_velocity = linear_velocity * gamma * 5;
  if (gamma < 0)
  {
    if (angular_velocity < 0)
    {
      angular_velocity = -angular_velocity;
    }
  }
  else
  {
    if (angular_velocity > 0)
    {
      angular_velocity = -angular_velocity;
    }
  }

  if (linear_velocity > robot_.max_linv_)
  {
    linear_velocity = robot_.max_linv_;
  }

  while (linear_velocity > robot_.max_linv_ || angular_velocity > robot_.max_rotv_)
  {
    angular_velocity *= 0.99;
    linear_velocity *= 0.99;
  }

  robot_.twist_.linear.x = linear_velocity;
  robot_.twist_.angular.z = angular_velocity;
}

void TurtleFollow::visServo(double centreDistance)
{
  // Get AR and Robot pose in 3D cords
  // Set tracking distance
  // Set heading angle tolerance
  // Calculate tracking pose    
  // Find angle to point to tracking point
    // if abs(ang) > ang_tot
      // Turn
    // else
      // lin_vel = max_lin_vel 

  // Published to ros in robotControl
  robot_.twist_.linear.x = linear_velocity_;
  robot_.twist_.angular.z = angular_velocity_;
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
      purePursuit(tag_pose_.position.x, tag_pose_.position.z);
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
    std::cout << "Linear: " << robot_.twist_.linear.x << std::endl;
    std::cout << "Angular: " << robot_.twist_.angular.z << std::endl;

    cmd_vel_pub_.publish(robot_.twist_);
    rate.sleep();
  }
}