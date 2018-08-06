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
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(${PROJECT_NAME}_pick_object_with_gripper src/pick_object_with_gripper.cpp)
#target_link_libraries(${PROJECT_NAME}_pick_object_with_gripper ${catkin_LIBRARIES})

#add_executable(${PROJECT_NAME}_plan_gripper_grasp_service src/plan_gripper_grasp_service.cpp)
#target_link_libraries(${PROJECT_NAME}_plan_gripper_grasp_service ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_pick_object_with_shadow src/pick_object_with_shadow.cpp)
target_link_libraries(${PROJECT_NAME}_pick_object_with_shadow ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_plan_shadow_grasp_service src/shadow_grasp_service.cpp)
target_link_libraries(${PROJECT_NAME}_plan_shadow_grasp_service ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_pick src/grasp.cpp)
target_link_libraries(${PROJECT_NAME}_pick ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_service src/service.cpp)
target_link_libraries(${PROJECT_NAME}_service ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_shadow_action_grasp src/pick_place_action.cpp)
target_link_libraries(${PROJECT_NAME}_shadow_action_grasp ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_test_action_grasp src/test_action_grasp.cpp)
target_link_libraries(${PROJECT_NAME}_test_action_grasp ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_gripper_grasp_action src/gripper_grasp_action.cpp)
target_link_libraries(${PROJECT_NAME}_gripper_grasp_action ${catkin_LIBRARIES})
