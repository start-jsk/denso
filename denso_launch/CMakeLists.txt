cmake_minimum_required(VERSION 2.8.3)
project(denso_launch)

find_package(catkin REQUIRED COMPONENTS denso_controller control_msgs vs060_moveit_config rostest)

catkin_package(
  DEPENDS
  CATKIN-DEPENDS denso_controller control_msgs vs060_moveit_config
  INCLUDE_DIRS
  LIBRARIES
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_rostest(test/vs060.test)