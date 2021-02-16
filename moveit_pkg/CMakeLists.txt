cmake_minimum_required(VERSION 2.8.3)
project(moveit_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_core
    geometry_msgs
    
)

catkin_package(CATKIN_DEPENDS
    roscpp
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_core
    geometry_msgs
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(move_group_node src/move_group_interface.cpp)
add_dependencies(move_group_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_group_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
