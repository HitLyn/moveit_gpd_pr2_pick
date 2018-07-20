#include <iostream>
#include <stdio.h>
#include <mutex>

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
private:
  ros::NodeHandle node_handle;
  ros::ServiceClient planning_scene_diff_client;
  ros::ServiceClient get_planning_scene_client;
  ros::Subscriber planning_scene_sub_;
  bool planning_srv_called_ = true;
  bool have_planning_scene_ = false;
  moveit_msgs::PlanningScene planning_scene_;

public:
  moveit::planning_interface::MoveGroupInterface right_arm_and_hand;
  moveit::planning_interface::MoveGroupInterface right_hand;

  PickUpObject() : right_arm_and_hand("right_arm_and_hand"), right_hand("right_hand")
  {
    planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();
    get_planning_scene_client = node_handle.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
    get_planning_scene_client.waitForExistence();
  }


  void getPlanningScene()
  {
    planning_scene_sub_ = node_handle.subscribe("planning_scene_for_gpd", 1, &PickUpObject::planningSceneCallback, this);
    ros::spinOnce();
  }

  bool getHavePlanningSceneAndSrv()
  {
    return have_planning_scene_;
  }

  void planningSceneCallback(const moveit_msgs::PlanningScene& msg)
  {
    moveit_msgs::ApplyPlanningScene srv;
    planning_scene_ = msg;
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;

    // clear the detached object from last time

    request.components.components = 4;
    get_planning_scene_client.call(request, response);
    if(response.scene.robot_state.attached_collision_objects.size() > 0)
    {
      moveit_msgs::AttachedCollisionObject detach_object;
      detach_object.object.id = "object";
      detach_object.object.operation = detach_object.object.REMOVE;
      planning_scene_.robot_state.attached_collision_objects.clear();
      planning_scene_.robot_state.attached_collision_objects.push_back(detach_object);
    }

    srv.request.scene = planning_scene_;

    have_planning_scene_ = planning_scene_diff_client.call(srv);
    if (have_planning_scene_)
    {
      ROS_INFO("planning scene got successfully. shutdown subscriber.");
      planning_scene_sub_.shutdown();
    }
  }

  bool executePick()
  {
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    request.components.components = 16;
    get_planning_scene_client.call(request, response);

    bool execute_success = false;
    execute_success = bool(right_arm_and_hand.planGraspsAndPick(response.scene.world.collision_objects[0]));
    have_planning_scene_ = false;

    if(execute_success)
    {
      // moveit_msgs::CollisionObject remove_object;
      // remove_object.id = "object";
      // remove_object.operation = remove_object.REMOVE;
      //
      // moveit_msgs::AttachedCollisionObject detach_object;
      // detach_object.object.id = "object";
      // detach_object.object.operation = detach_object.object.REMOVE;
      //
      ROS_INFO("picking object successfully! removing object.");
      // planning_scene_.world.collision_objects.clear();
      // planning_scene_.world.collision_objects.push_back(remove_object);
      //
      // planning_scene_.robot_state.attached_collision_objects.clear();
      // planning_scene_.robot_state.attached_collision_objects.push_back(detach_object);

      // have_planning_scene_ = false;
      ROS_INFO("planning scene changed.");

    }
    return execute_success;
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_object_demo");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle n;

  PickUpObject po;

  bool success = false;
  po.getPlanningScene();
  // std::cout << po.getHavePlanningSceneAndSrv();

  while(ros::ok())
  {
    if(success)
    {
      ROS_INFO("picking successfully, changing position.");
      // po.arm.setNamedTarget("right_arm_side");
      // while(!po.arm.move())
      //   ROS_ERROR("moving to side pose failed.");
      // po.hand.setNamedTarget("open");
      // while(!po.hand.move())
      //   ROS_ERROR("opening hand failed.");
      // po.arm.setNamedTarget("home");
      // while(!po.arm.move())
      //   ROS_ERROR("moving home failed.");

      success = false;
    }
    if (!po.getHavePlanningSceneAndSrv())
    {
      ROS_INFO("no planning scene.");
    }
    else
      ROS_INFO("planning scene exists.");

    while(!po.getHavePlanningSceneAndSrv())
    {
      po.getPlanningScene();
    }
    if (!po.getHavePlanningSceneAndSrv())
    {
      ROS_INFO("no planning scene.");
    }
    else
      ROS_INFO("planning scene exists.");

    success = po.executePick();
    ros::Duration(1).sleep();

  }

  return 0;
}
