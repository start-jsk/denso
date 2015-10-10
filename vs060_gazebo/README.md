A v060 package in Gazebo.

##Dependencies
Please make sure that all the dependencies are up to date

`sudo apt-get update`

Install ros_control from debian packages 

`sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers`

## How to run it
1) Launch the robot into gazebo.

`roslaunch vs060_gazebo vs060_gazebo.launch`

2) If you want to command the robot and see the performance, you can run the following line

`roslaunch vs060_gazebo vs060_rqt.launch`
