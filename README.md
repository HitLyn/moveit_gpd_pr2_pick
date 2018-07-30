# moveit_gpd_pr2_pick
# Using gpd message to grasp on Trixi


## Working Process1 (with gpd package):
* roslaunch openni_launch openni.launch
* roslaunch tams_pr2_moveit_config demo.launch
* rosrun tf static 0 0 1.26 0 0 0 1 odom_combined camera_link
* roslaunch gpd shadow_grasp.launch/gripper_grasp.launch
* roslaunch moveit_gpd_pr2_pick pick_object_demo_loop.launch/ shadow_grasp_service.launch
* (rosrun moveit_gpd_pr2_pick moveit_gpd_pr2_pick_pick_object_with_shadow)

## Working Process2 (with dex_grasp_pose topic published)
* rosrun moveit_gpd_pr2_pick moveit_gpd_pr2_pick_test_action_grasp to publish grasp pose messages we need
* rosrun moveit_gpd_pr2_pick moveit_gpd_pr2_pick_shadow_action_grasp/moveit_gpd_pr2_pick_gripper_grasp_action
