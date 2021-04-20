# AR Tag Spawning

##Credit and Acknowledgements
All the credit for the logic for spawning the AR tag belongs to Miguel Angel. The video tutorial I followed is linked below:
https://www.youtube.com/watch?v=WDhIaVOUwsk&t=1s

##Prerequistes
* Gazebo installed. Preferrably with turtlebot packages installed
* Possibily catkin_tools

##Procedure
1. Copy the folders "ar\_tag" and "spawn\_robot\_tools_pkg" into the src folder in the workspace that you intend to run Gazebo on. 
2. Open terminal and navigate to the workspace and run the command "catkin build"
   1. catkin_make might work but I haven't tested
3. Ensure that the packages "ar\_tag" and "spawn\_robot\_tools\_pkg" are referenced in the terminal after running the command to ensure that the packages are recognised and built.
4. Run the following command to update the workspace "source /(workspace_location)/devel/setup.bash"
   1. To automate this you can run the following command to run the setup when the terminal is opened 'echo "source /opt/ros/melodic/setup.bash"'
5. Run the command "rospack profile"
6. At this point the packages should be installed and the AR tag should be spawnable. Now open an instance of Gazebo. For the purposes of this assignment, we are using a turtlebot so I used the following command to spawn a turtlebot in an empty world "roslaunch turtlebot3\_gazebo turtlebot3\_empty_world.launch".
7. Run the command "roslaunch ar\_tag spawn\_ar_tracking.launch". This should spawn the AR tag in the world. 
