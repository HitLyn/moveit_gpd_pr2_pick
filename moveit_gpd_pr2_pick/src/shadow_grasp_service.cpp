#include <ros/ros.h>
#include <moveit_msgs/GraspPlanning.h>
#include <moveit_msgs/CollisionObject.h>
#include <gpd/GraspConfigList.h>
#include <gpd/GraspConfig.h>
#include <mutex>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/Grasp.h>

#include <manipulation_msgs/GraspPlanning.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <stdio.h>

class PlanGPDGrasp
{
public:
  PlanGPDGrasp(ros::NodeHandle& n)
  {
    clustered_grasps_sub_ = n.subscribe("/detect_grasps/clustered_grasps", 10, &PlanGPDGrasp::clustered_grasps_callback, this);
    grasps_visualization_pub_ = n.advertise<geometry_msgs::PoseArray>("grasps_visualization", 10);

    planning_scene_diff_client_ = n.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client_.waitForExistence();
    get_planning_scene_client_ = n.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
    get_planning_scene_client_.waitForExistence();

    planning_scene_diff_publisher_ = n.advertise<moveit_msgs::PlanningScene>("planning_scene_for_gpd", 1);

    ros::NodeHandle pn("~");

    pn.param("bound_frame", bound_frame_, std::string("odom_combined"));
    pn.param("x_bound", x_bound_, 1.0);
    pn.param("y_bound", y_bound_, 1.0);
    pn.param("z_bound", z_bound_, 2.0);
    pn.param("x_bound_offset", x_bound_offset_, 0.0);
    pn.param("y_bound_offset", y_bound_offset_, 0.0);
    pn.param("z_bound_offset", z_bound_offset_, 0.29);

    pn.param("grasp_offset", grasp_offset_, -0.1);

    pn.param("grasp_cache_time_threshold", grasp_cache_time_threshold_, 5.0);

    std::string move_group_arm;
    std::string move_group_hand;
    pn.param("move_group_arm", move_group_arm, std::string("right_arm_and_hand"));
    pn.param("move_group_hand", move_group_hand, std::string("right_hand"));

    moveit::planning_interface::MoveGroupInterface right_arm_and_hand(move_group_arm);
    moveit::planning_interface::MoveGroupInterface right_hand(move_group_hand);

    // Setting variables in grasp_candidate_ that are the same for every grasp
    grasp_candidate_.id = "grasp";

    grasp_candidate_.pre_grasp_approach.min_distance = 0.08;
    grasp_candidate_.pre_grasp_approach.desired_distance = 0.1;

    grasp_candidate_.post_grasp_retreat.min_distance = 0.13;
    grasp_candidate_.post_grasp_retreat.desired_distance = 0.15;
    // grasp_candidate_.post_grasp_retreat.direction.header.frame_id = right_arm_and_hand.getEndEffectorLink();
    grasp_candidate_.post_grasp_retreat.direction.header.frame_id = right_arm_and_hand.getEndEffectorLink();
    // grasp_candidate_.post_grasp_retreat.direction.header.frame_id = bound_frame_;
    // ROS_INFO(grasp_candidate_.post_grasp_retreat.direction.header.frame_id);

    jointValuesToJointTrajectory(right_hand.getNamedTargetValues("open"), ros::Duration(2.0), grasp_candidate_.pre_grasp_posture);
    jointValuesToJointTrajectory(right_hand.getNamedTargetValues("pack"), ros::Duration(2.0), grasp_candidate_.grasp_posture);
  }

  moveit_msgs::CollisionObject spawnObject(const gpd::GraspConfigList::ConstPtr& msg)
  {
    moveit_msgs::CollisionObject object;
    object.header.frame_id = frame_id_;
    // object.header.frame_id = bound_frame_;
    object.id = "object";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.03;
    primitive.dimensions[1] = 0.03;
    primitive.dimensions[2] = 0.03;

    geometry_msgs::Pose pose;
    for(gpd::GraspConfig grasp : msg->grasps)
    {
      pose.position.x+= grasp.top.x;
      pose.position.y+= grasp.top.y;
      pose.position.z+= grasp.top.z;
    }
    pose.orientation.w = 1;
    pose.position.x = pose.position.x/(msg->grasps.size());
    pose.position.y = pose.position.y/(msg->grasps.size());
    pose.position.z = pose.position.z/(msg->grasps.size());
    // pose.position.x = 0.6;
    // pose.position.y = -0.178;
    // pose.position.z = 1.29;

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);

    // add object to scene
    object.operation = object.ADD;
    return object;
  }

  void clustered_grasps_callback(const gpd::GraspConfigList::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(m_);

    // clear all grasp candidates every time received the new message from "detect grasps"
    grasp_candidates_.clear();

    // plan the planning scene and publish
    planning_scene_.is_diff = true;
    planning_scene_.robot_state.is_diff = true;

    // generate grasp candidates
    ros::Time grasp_stamp = msg->header.stamp;
    frame_id_ = msg->header.frame_id;
    grasp_candidate_.grasp_pose.header.frame_id = frame_id_;
    grasp_candidate_.pre_grasp_approach.direction.header.frame_id = frame_id_;

    moveit_msgs::CollisionObject object = spawnObject(msg);
    planning_scene_.world.collision_objects.push_back(object);
    planning_scene_diff_publisher_.publish(planning_scene_);

    for(gpd::GraspConfig grasp : msg->grasps)
    {
      // shift the grasp according to the offset parameter
      grasp.top.x = grasp.top.x + grasp_offset_ * grasp.approach.x;
      grasp.top.y = grasp.top.y + grasp_offset_ * grasp.approach.y;
      grasp.top.z = grasp.top.z + grasp_offset_ * grasp.approach.z;

      if(grasp_boundry_check(grasp))
      {
        grasp_candidate_.grasp_pose.pose = gpd_grasp_to_pose(grasp);

        grasp_candidate_.grasp_quality = grasp.score.data;
        grasp_candidate_.pre_grasp_approach.direction.vector.x = grasp.approach.x;
        grasp_candidate_.pre_grasp_approach.direction.vector.y = grasp.approach.y;
        grasp_candidate_.pre_grasp_approach.direction.vector.z = grasp.approach.z;

        grasp_candidates_.push_front(std::make_pair(grasp_candidate_, grasp_stamp));
      }
    }
    ROS_INFO_STREAM("grasp candidates added, totally " << grasp_candidates_.size() << " candidates ready.");
  }

  bool grasp_boundry_check(const gpd::GraspConfig &grasp)
  {
    geometry_msgs::PointStamped grasp_point;
    geometry_msgs::PointStamped transformed_grasp_point;
    grasp_point.header.frame_id = frame_id_;
    bool result;

    grasp_point.point = grasp.top;
    // transform the grasp point into the frame of the bound to make the boundary check easier
    try
    {
      listener_.transformPoint(bound_frame_, grasp_point, transformed_grasp_point);
    }
    catch( tf::TransformException ex)
    {
      ROS_ERROR("transfrom exception : %s",ex.what());
    }

    result = (transformed_grasp_point.point.x < 1
         && transformed_grasp_point.point.x > -1
         && transformed_grasp_point.point.y < 1
         && transformed_grasp_point.point.y > -1
         && transformed_grasp_point.point.z < 1.5
         && transformed_grasp_point.point.z > 0);

    // std::cout << result << std::endl;

    return result;
  }

  geometry_msgs::Pose gpd_grasp_to_pose(const gpd::GraspConfig& grasp)
  {
    geometry_msgs::Pose pose;

    tf::Matrix3x3 orientation(-grasp.binormal.x, -grasp.approach.x, -grasp.axis.x,
                              -grasp.binormal.y, -grasp.approach.y, -grasp.axis.y,
                              -grasp.binormal.z, -grasp.approach.z, -grasp.axis.z);


    // tf::Matrix3x3 orientation(-grasp.axis.x, -grasp.approach.x, grasp.binormal.x,
    //                           -grasp.axis.y, -grasp.approach.y, grasp.binormal.y,
    //                           -grasp.axis.z, -grasp.approach.z, grasp.binormal.z); // normal
    //
    // tf::Matrix3x3 orientation(grasp.approach.x, grasp.binormal.x, grasp.axis.x,
    //                           grasp.approach.y, grasp.binormal.y, grasp.axis.y,
    //                           grasp.approach.z, grasp.binormal.z, grasp.axis.z);



    tf::Quaternion orientation_quat;
    orientation.getRotation(orientation_quat);
    tf::quaternionTFToMsg(orientation_quat, pose.orientation);

    pose.position = grasp.top;

    return pose;
  }

  bool plan_gpd_grasp(moveit_msgs::GraspPlanning::Request &req, moveit_msgs::GraspPlanning::Response &res)
  {
    geometry_msgs::PoseArray grasps_visualization;
    grasps_visualization.header.frame_id = frame_id_;

    {
      std::lock_guard<std::mutex> lock(m_);
      for (auto grasp_candidate:grasp_candidates_)
      {
        res.grasps.push_back(grasp_candidate.first);
        grasps_visualization.poses.push_back(grasp_candidate.first.grasp_pose.pose);
      }
      ROS_INFO_STREAM(res.grasps.size() << " proper grasps found, and all former candidates cleared.");
    }

    if(res.grasps.empty())
    {
      ROS_INFO("No valid grasp found.");
      res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    else
    {
      ROS_INFO("%ld grasps found.", res.grasps.size());
      res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    grasps_visualization_pub_.publish(grasps_visualization);

    return true;
  }

private:
  void jointValuesToJointTrajectory(std::map<std::string, double> target_values, ros::Duration duration,
        trajectory_msgs::JointTrajectory &grasp_pose)
  {
    grasp_pose.joint_names.reserve(target_values.size());
    grasp_pose.points.resize(1);
    grasp_pose.points[0].positions.reserve(target_values.size());

    for(std::map<std::string, double>::iterator it = target_values.begin(); it != target_values.end(); ++it)
    {
      grasp_pose.joint_names.push_back(it->first);
      grasp_pose.points[0].positions.push_back(it->second);
    }
    grasp_pose.points[0].time_from_start = duration;
  }

  moveit_msgs::Grasp grasp_candidate_;
  std::deque<std::pair<moveit_msgs::Grasp, ros::Time>> grasp_candidates_;
  std::mutex m_;
  ros::Subscriber clustered_grasps_sub_;
  tf::TransformListener listener_;
  ros::Publisher grasps_visualization_pub_;

  ros::ServiceClient planning_scene_diff_client_;
  ros::ServiceClient get_planning_scene_client_;
  moveit_msgs::PlanningScene planning_scene_;
  ros::Publisher planning_scene_diff_publisher_;

  // frame of the grasp
  std::string frame_id_;

  // frame of the bound that determines which grasps are valid
  std::string bound_frame_;

  // measurements of the bound that determines which grasps are valid
  double x_bound_;
  double y_bound_;
  double z_bound_;

  // offset of the bounds from the 0 position
  double x_bound_offset_;
  double y_bound_offset_;
  double z_bound_offset_;

  // offset of the grasp along the approach vector
  double grasp_offset_;

  // grasps older than this threshold are not considered anymore
  double grasp_cache_time_threshold_;
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "shadow_grasp_service");
  ros::NodeHandle n;

  PlanGPDGrasp gpd_grasp(n);

  ros::ServiceServer ss = n.advertiseService("plan_grasps", &PlanGPDGrasp::plan_gpd_grasp, &gpd_grasp);
  ros::spin();

  return 0;
}
