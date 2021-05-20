# turtle_follow
## Turtlebot Following Task
This project controls a Turtlebot robot with the task of following another Turtlebot.This will be done through the development of a program with the application of sensing, path planning and control.A  Gazebo  simulation  will  also  be  used  allowing  for  a  plethora  of  testing  to  be  conducted  in  a  safe  and controlled environment. This will be donebefore introducing real world testing that could pose a risk to student and staff safety as well as damageto the Turtlebot and surrounding environment. Sensors that will be utilized for this project include an RGB-D sensor for the detection of the other robot and a LiDAR for close range peripheral sensing.


This program and simulation will then be able to be utilized as a template for future following, sensing and movement projects. This involves projects in the automated manufacturing, autonomous mining and farming, public transport,and private civilian transport industries.The application of real-world sensors will also provide experience and knowledge that will prove to bebeneficial for future projects.

## The Turtle_Follow project is made up of:
* Main
* Turtle Follow
* Gazebo Package

## How to run Gazebo Simulation
* Prerequisites: ar_track_alvar package
1. Git clone to your local drive
2. Make symbolic link to your catkin_ws/src folder
3. Copy the cam.launch file into your catkin_ws/src/ar_track_alvar/launch folder
4. Navigate to catkin_ws in your terminal
5. catkin build or catkin_make in terminal (depending on you catkin set up)
6. Open 4 terminals to run the gazebo simulation. In the 4 terminals run:

    i. roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
    
    ii. roslaunch ar_tag spawn_ar_tracking.launch

    iii. roslaunch ar_track_alvar cam.launch

    iv. rosrun turtle_follow turtle_follow

## How to run real Turtlebot
* Prerequisites: ar_track_alvar package
1. Git clone to your local drive
2. Make symbolic link of the turtle_follow folder to your catkin_ws/src folder
3. Copy the cam.launch file into your catkin_ws/src/ar_track_alvar/launch folder
4. Navigate to catkin_ws in your terminal
5. catkin build or catkin_make in terminal (depending on you catkin set up)
6. roslaunch all core turtlebot movement, lidar and camera launch files
7. roslaunch ar_track_alvar cam.launch
8. rosrun turtle_follow turtle_follow

## Contributors & Maintainers
- Himavaan Chandra
    - AR Tag
    - Control
    - Package Set Up
- Chloe Mallinson
    - Image Based Visual Servoing
    - Gazebo Set Up
- Hayden McManus
    - Gazebo Simulation Package