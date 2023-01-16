cmake_minimum_required(VERSION 3.0.2)
project(traineeship)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gazebo_ros
  # interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

# INCLUDE_DIRS include/${PROJECT_NAME}/
#  LIBRARIES traineeship
#  CATKIN_DEPENDS franka_ros moveit panda_moveit_config roscpp std_msgs
#  DEPENDS
#    roscpp
#    gazebo_ros
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(logical_to_moveit src/logical_to_moveit.cpp)
target_link_libraries(logical_to_moveit ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
# add_dependencies(logical_to_moveit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(gzlistener src/gzlistener.cpp)
target_link_libraries(gzlistener ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(stone_mover src/stone_mover.cpp)
target_link_libraries(stone_mover ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
