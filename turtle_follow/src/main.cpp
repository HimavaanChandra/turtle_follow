#include "ros/ros.h"
#include "turtle_follow.h"

int main(int argc, char **argv)
{
  // Initialise ros node and handle
  ros::init(argc, argv, "turtle_follow");
  ros::NodeHandle nh;

  std::shared_ptr<TurtleFollow> turtleFollow(new TurtleFollow(nh));
  std::thread t(&TurtleFollow::robotControl, turtleFollow);
  
  ros::spin();

  ros::shutdown();
  t.join();

  return 0;
}
