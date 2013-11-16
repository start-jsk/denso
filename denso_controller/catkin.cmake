# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(denso_controller)

find_package(catkin REQUIRED COMPONENTS roscpp open_controllers_interface)

catkin_package(
  DEPENDS
  CATKIN-DEPENDS roscpp open_controllers_interface
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})

install(TARGETS main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES denso_controllers.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



