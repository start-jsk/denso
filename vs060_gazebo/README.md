A v060 package in Gazebo.

##Dependencies
Please make sure that all the dependencies are up to date

`sudo apt-get update`

Install ros_control from debian packages 

`sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers`

## How to run it
1) Launch the robot into gazebo.

`roslaunch vs060_gazebo vs060_gazebo.launch`

![first](https://cloud.githubusercontent.com/assets/12606874/10413473/897cdfda-6fb6-11e5-91e0-c80c8ffdd996.png)


2) If you want to command the robot and see the performance, you can run the following line

`roslaunch vs060_gazebo vs060_rqt.launch`

You can command each joint seperately. Simply mark the joint on the right-side of the following figure. To see the changes of the signals you can change the tabe according to the joint number from the below sidbar, as the highlighed circle shows.
![rqt](https://cloud.githubusercontent.com/assets/12606874/10413389/eac86604-6fb3-11e5-9966-1c2eabebc263.png)
By selecting all the commands from the right-side of the previous picture (where all joints are set to 1 rad), you will see the following picture.
![settingcommands](https://cloud.githubusercontent.com/assets/12606874/10413390/f0bf67c4-6fb3-11e5-8daa-acc9bc80c174.png)
