//Receive ar tag positions from ROS subscriber

//Save x position  (y and z not needed) into variables

//0 is centre

//Detect distance from center

//Use car code to determine steering direction.
//Might need to change speed depending on turning amount

//Use lidar to check distance to robot

//If within a certain range then stop

//If lower than range then reverse.

//Send steering commands to ROS


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