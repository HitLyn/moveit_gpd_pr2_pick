cmake_minimum_required(VERSION 2.8.3)
project(moveit_gpd_pr2_pick)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gpd
  message_filters
  moveit_core
  moveit_ros_planning_interface
  sensor_msgs
  pcl_ros
  roscpp
  std_msgs
  tf
)


catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_pick_object_with_gripper src/pick_object_with_gripper.cpp)
target_link_libraries(${PROJECT_NAME}_pick_object_with_gripper ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_plan_gripper_grasp_service src/plan_gripper_grasp_service.cpp)
target_link_libraries(${PROJECT_NAME}_plan_gripper_grasp_service ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_pick_object_with_shadow src/pick_object_with_shadow.cpp)
target_link_libraries(${PROJECT_NAME}_pick_object_with_shadow ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_plan_shadow_grasp_service src/shadow_grasp_service.cpp)
target_link_libraries(${PROJECT_NAME}_plan_shadow_grasp_service ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_pick src/grasp.cpp)
target_link_libraries(${PROJECT_NAME}_pick ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_service src/service.cpp)
target_link_libraries(${PROJECT_NAME}_service ${catkin_LIBRARIES})
