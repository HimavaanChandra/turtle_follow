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
    odom_sub_ = nh_.subscribe("/odom", 10, &TurtleFollow::odomCallback, this);
    laser_sub_ = nh_.subscribe("/base_scan", 10, &TurtleFollow::laserCallback, this);
    tag_sub_ = nh_.subscribe("/ar_pose_marker", 10, &TurtleFollow::tagCallback, this);

    //Topic robot_0 might be different------------------------------------------------------
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

TurtleFollow::~TurtleFollow()
{
}

void TurtleFollow::tagCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &msg)
{
  // const struct ar_track_alvar_msgs::AlvarMarkers_<std::allocator<void> >
  // ar_track_alvar_msgs::AlvarMarker[] markers = msg->markers;
  // std::vector<ar_track_alvar_msgs::AlvarMarker> test = msg->markers;
  tag_pose_ = msg->markers.at(0).pose.pose;
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
  if (robot_.ranges_.size() > 0)
  {
    for (unsigned int i = 0; i < robot_.ranges_.size(); i++)
    {
      //The +-25 is the window of detection -- Can move this to the for loop above to make neater------------------------------------------------
      if ((robot_.ranges_.at(i) < robot_.radius_ + 0.1) && (i >= ((robot_.ranges_.size() / 2) - 25) && i <= ((robot_.ranges_.size() / 2) + 25)))
      {
        ROS_INFO_STREAM("Obstruction Ahead!");
        robot_.obstacle_ = true;
      }
    }
  }
  else
  {
    robot_.obstacle_ = false;
  }
  return robot_.obstacle_;
}

void TurtleFollow::detection(void)
{
	//Convert image
	//BGR2HSV
  cv::Mat image_ = cv::imread("/home/ros/catkin_ws/src/maze_navigating_robot/imageTuning/image2.jpg"); //Remove and replace with topic/imagecallback
  // cv::Mat image_ = &image; How do I access &image?-----------------------------------------------------------------------------
  cv::Mat hsv; //Make class variable?/Only use one vartaiable to save time for all hsv redMask edges etc--------------------------
	cv::cvtColor(image_, hsv, cv::COLOR_BGR2HSV);

	//https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
	//Threshold to isolate Red
	cv::Mat redMask;
	//Sim
	// cv::inRange(hsv, cv::Scalar(0, 127, 50), cv::Scalar(6, 255, 255), redMask);
	//Real Robot
	cv::inRange(hsv, cv::Scalar(0, 127, 50), cv::Scalar(6, 255, 255), redMask);

	//Draw box around red blob
	cv::Mat edges;
	cv::Canny(redMask, edges, 400, 1400, 3);

	//Find contours: https://docs.opencv.org/master/d4/d73/tutorial_py_contours_begin.html
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(edges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	//Draw contours
	cv::drawContours(image_, contours,  0, cv::Scalar(0, 255, 0), 2);

	//Get centre position of blob
	//Get moments of contours
	cv::Moments m = cv::moments(contours[0], true);
	//centre of blob: https://www.pyimagesearch.com/2016/02/01/opencv-center-of-contour/
	int cx = m.m10 / m.m00;
	int cy = m.m01 / m.m00;
  cv::Point pt(cx, cy);
  cv::circle(image_, pt, 3, CV_RGB(0, 255, 0), 1);

	double area = cv::contourArea(contours[0]);

  cv::Size size = image_.size();
  double frameArea = size.width*size.height;
  std::cout << "Contour Area: " << area << std::endl; //--------------------------------------------------------
  std::cout << "Frame Area: " << frameArea << std::endl; //-----------------------------------------------------
  //Do ratio comparison then initiate takeover?
  double ratio  = area/frameArea;
  int cutoff=0;//Adjust------------------------------
  if (ratio > cutoff)
  {
    //Take over control till ratio is certain amount. - This might need to be in higher loop so that values can be recalculated or not

    //Set linear and angular velocity override
    geometry_msgs::Twist twist{};
    twist.angular.z = 0.;
    twist.linear.x = 0;
    // cmd_vel_pub_.publish(twist); //Needs to be redefined?------------------------------

  }
}




void TurtleFollow::basicController(double centreDistance)
{
    // Maximum translational velocity	Burger = 0.22 m/s	Waffle = 0.26 m/s
    // Maximum rotational velocity	Burger = 2.84 rad/s (162.72 deg/s)	Waffle = 1.82 rad/s (104.27 deg/s)
    
    //Adjust tolerances
    //Within tolerance
    if (centreDistance >= -0.1 && centreDistance <= 0.1)
    {
        robot_.control_.linear.x = 0.22;
        robot_.control_.angular.z = 0;
    }
    //To the left
    else if (centreDistance < -0.1)
    {
        robot_.control_.linear.x = 0.11;
        robot_.control_.angular.z = 1.4;
    }
    //To the right
    else if (centreDistance > 0.1)
    {
        robot_.control_.linear.x = 0.11;
        robot_.control_.angular.z = -1.4;
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
    
}

void TurtleFollow::visServo(double centreDistance)
{
    // Visual Servoing 
    // double linear_velocity_ = 0;
    // double angular_velocity_ = 0;

    // // Find the centre of the AR tag 
    // x = tag_pose_.x;
    // z = tag_pose_.z; 

    // // Find linear and angular velocity to navigate centre of AR tag to the centre of camera frame using visual servoing

    // // Published to ros in robotControl
    // robot_.control_.linear.x = linear_velocity_;
    // robot_.control_.angular.z = angular_velocity_;
}


void TurtleFollow::robotControl()
{
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


    //Only use 1
    //Basic controller
    basicController(tag_pose_.position.x);

    //Pure pursuit controller //Tag pose z might be dodgy might have to check tolerance with lidar----------------
    purePursuit(tag_pose_.position.x, tag_pose_.position.z);

    //Use car code to determine steering direction.
    //Might need to change speed depending on turning amount


    

    //Use lidar to check distance to robot //compare to ar tag distance withing tolerance. //This might be dodgy

    //If within a certain range then stop

    //If lower than range then reverse.

    //Send steering commands to ROS

    // robot_.control_.linear.x = 0;
    // robot_.control_.angular.z = 0;
    if (obstructionDetection())
    {
        robot_.control_.linear.x = 0;
        robot_.control_.angular.z = 0;
    }
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