cmake_minimum_required(VERSION 3.0.2)
project(homework1)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs nav_msgs dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(robotVelocity src/robotVelocity.cpp)
add_dependencies(robotVelocity ${catkin_EXPORTED_TARGETS})
target_link_libraries(robotVelocity ${catkin_LIBRARIES})

add_executable(robotOdometry src/robotOdometry.cpp) 
add_dependencies(robotOdometry ${PROJECT_NAME}_gencfg)
target_link_libraries(robotOdometry ${catkin_LIBRARIES})
