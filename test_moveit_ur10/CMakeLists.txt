cmake_minimum_required(VERSION 2.8.3)
project(test_moveit_ur10)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
             COMPONENTS
             moveit_core
             roscpp
             moveit_ros_planning
             moveit_ros_planning_interface
	           moveit_visual_tools
             pluginlib
             cmake_modules
             geometric_shapes
             geometry_msgs
	           joy
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    roscpp
    moveit_ros_planning_interface
    interactive_markers
)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(robot_pose src/robot_pose.cpp)
target_link_libraries(robot_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS robot_pose DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(scene src/joy.cpp)
target_link_libraries(scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS scene DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

