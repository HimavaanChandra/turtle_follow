#include "ros/ros.h"
#include "turtle_follow.h"
//Check message types in cmake and package.xml

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_follow");

  ros::NodeHandle nh;

  std::shared_ptr<Pursuit> gc(new Pursuit(nh));
//   std::thread t(&Pursuit::imagePublish, gc);



  ros::spin();

  ros::shutdown();
//   t.join();



  return 0;
}

