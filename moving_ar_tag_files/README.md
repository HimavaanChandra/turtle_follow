# Sensors-and-Control
   
1. To get the AR tag moving in Gazebo we will need to move the following files. Sudo permissions are required so if you running "sudo nautilus" will open a sudo instance of file manager to make this a bit easier.
   
   1.1)
   Look for the file "turtlebot3\_teleop\_key.launch" and move to /opt/ros/melodic/share/turtlebot3_teleop/launch (replace file inside folder).
   
   1.2)
   Look for the file "pr2\_indiv.launch" will move to /opt/ros/melodic/share/ar\_track\_alvar/launch (and replace the file with the same name in this folder).
   
   1.3)
   Look for the files "spawn\_sdf.launch" and "turtlebot3\_multi\_ar.launch" and move them to (catkin workspace)/src/turtlebot3\_simulations/turtlebot3_gazebo/launch

   
   1.4)
   Look for the folder "turtlebot3\_arleader" should be moved to /src/turtlebot3\_simulations/turtlebot3_gazebo/models

   
2. In the models folder --> turtlebot3_arleader, open model.sdf in a text editor then
   'ctrl+f' and search for "hayden" change all 5 iterations to the name of your user/home directory name and save 

3. In a terminal: 

   roslaunch turtlebot3\_gazebo turtlebot3\_multi_ar.launch

   roslaunch ar\_track\_alvar pr2_indiv.launch

   roslaunch turtlebot3\_teleop turtlebot3\_teleop_key.launch 

                  

