#ifndef PICK_PLACE_ACTION_
#define PICK_PLACE_ACTION_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit_msgs/CollisionObject.h>

#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class PR2_Shadow_Grasp
{
public:

  PR2_Shadow_Grasp(ros::NodeHandle& node);

  void planPickCallback(const geometry_msgs::PoseStamped& msg);
  void executePick();

  void spawnObject(const geometry_msgs::PoseStamped pose_);


  std::vector<moveit_msgs::Grasp> generateGrasp(const geometry_msgs::PoseStamped pose_);

private:

  void jointValuesToJointTrajectory(std::map<std::string, double> target_values, ros::Duration duration,
        trajectory_msgs::JointTrajectory &grasp_pose);

  // ros::NodeHandle n_;
  ros::Subscriber plan_pick_sub_;
  ros::Subscriber execute_pick_sub_;

  actionlib::SimpleActionClient<moveit_msgs::PickupAction> pick_action_client_;

  moveit::planning_interface::MoveGroupInterface *arm_;
  moveit::planning_interface::MoveGroupInterface *hand_;
  moveit::planning_interface::PlanningSceneInterface *psi_;
  moveit_msgs::CollisionObject object_;

  std::vector<moveit_msgs::RobotTrajectory> current_trajectories_;
  std::string current_object_;
  bool grasp_central_;

};

#endif
