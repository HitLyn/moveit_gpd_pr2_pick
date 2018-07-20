# moveit_gpd_pr2_pick
Using gpd message to grasp on Trixi

ROS Version: Kinetic
Working Process:
1 roslaunch openni_launch openni.launch
2 roslaunch tams_pr2_moveit_config demo.launch
3 rosrun tf static 0 0 1.26 0 0 0 1 odom_combined camera_link
4 roslaunch gpd shadow_grasp.launch/gripper_grasp.launch
5 roslaunch moveit_gpd_pr2_pick pick_object_demo_loop.launch/ shadow_grasp_service.launch
6 (rosrun moveit_gpd_pr2_pick moveit_gpd_pr2_pick_pick_object_with_shadow)
