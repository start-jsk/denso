cmake_minimum_required(VERSION 2.8.3)
project(denso_launch)

find_package(catkin REQUIRED)

catkin_package(
  DEPENDS
  CATKIN-DEPENDS
  INCLUDE_DIRS
  LIBRARIES
)

install(DIRECTORY launch test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/vs060.test)
endif()