#include <iostream>
#include <stdio.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_matrix.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PickupActionResult.h>

#include <manipulation_msgs/GraspPlanning.h>

class PickUpObject
{
public:
  moveit::planning_interface::MoveGroupInterface arm;
  moveit::planning_interface::MoveGroupInterface arm_with_hand;
  moveit::planning_interface::MoveGroupInterface hand;
  moveit::planning_interface::PlanningSceneInterface psi;

  PickUpObject() : arm_with_hand("right_arm_and_hand"), hand("right_hand"), arm("right_arm")
  {
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;
  }

  bool executePick()
  {
    arm_with_hand.setPlanningTime(20.0);
    return arm_with_hand.planGraspsAndPick() == moveit_msgs::MoveItErrorCodes::SUCCESS;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_object_demo");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  PickUpObject po;

  bool success = false;

  ROS_INFO("Picking Object");
  while(ros::ok())
  {
    if(success)
    {
      po.arm.setNamedTarget("right_arm_side");
      while(!po.arm.move())
        ROS_ERROR("moving to side pose failed.");
      po.hand.setNamedTarget("open");
      while(!po.hand.move())
        ROS_ERROR("opening hand failed.");
      // po.arm.setNamedTarget("home");
      // while(!po.arm.move())
      //   ROS_ERROR("moving home failed.");
      ros::Duration(5).sleep();
      success = false;
    }
    success = po.executePick();
  }

  return 0;
}
