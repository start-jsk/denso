# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(vs060)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roslang roscpp moveit_commander moveit_ros_planning)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roslang roscpp moveit_commander moveit_ros_planning
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(publish_scene_from_text src/publish_scene_from_text.cpp)
target_link_libraries(publish_scene_from_text ${catkin_LIBRARIES})
if (${CMAKE_VERSION} VERSION_GREATER "3.1.3")
  target_compile_features(publish_scene_from_text PRIVATE cxx_range_for)
endif()

install(TARGETS publish_scene_from_text
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY model DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
