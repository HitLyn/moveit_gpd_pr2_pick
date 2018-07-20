#include <ros/ros.h>
#include <moveit_msgs/GraspPlanning.h>

#include <moveit/move_group_interface/move_group_interface.h>

void jointValuesToJointTrajectory(std::map<std::string, double> target_values, ros::Duration duration,
        trajectory_msgs::JointTrajectory &grasp_pose)
{
  grasp_pose.joint_names.reserve(target_values.size());
  grasp_pose.points.resize(1);
  grasp_pose.points[0].positions.reserve(target_values.size());

  for(std::map<std::string, double>::iterator it = target_values.begin(); it != target_values.end(); ++it){
    grasp_pose.joint_names.push_back(it->first);
    grasp_pose.points[0].positions.push_back(it->second);
  }
  grasp_pose.points[0].time_from_start = duration;
}

bool serviceCB(moveit_msgs::GraspPlanning::Request &req, moveit_msgs::GraspPlanning::Response &res)
{
  moveit::planning_interface::MoveGroupInterface arm("right_arm_and_hand");
  moveit::planning_interface::MoveGroupInterface hand("right_hand");

  moveit_msgs::Grasp grasp;
  grasp.id = "grasp";

  jointValuesToJointTrajectory(hand.getNamedTargetValues("open"), ros::Duration(1.0), grasp.pre_grasp_posture);
  jointValuesToJointTrajectory(hand.getNamedTargetValues("pack"), ros::Duration(2.0), grasp.grasp_posture);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "odom_combined";
  pose.pose.orientation.x = -0.215860639118;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = -0.976424182658;
  pose.pose.orientation.w = 0;

  // pose.header.frame_id = "odom_combined";
  // pose.pose.orientation.x = -0.0;
  // pose.pose.orientation.y = -0.887929895957;
  // pose.pose.orientation.z = -0.0;
  // pose.pose.orientation.w = 0.459978803714;
  //
  pose.pose.position.x = 0.6;
  pose.pose.position.y = -0.178;
  pose.pose.position.z = 1.29;
  //
  // pose.pose.position.x = 0.460948845436;
  // pose.pose.position.y = 0.188;
  // pose.pose.position.z = 1.42612504162;

  grasp.grasp_pose = pose;

  grasp.pre_grasp_approach.min_distance = 0.08;
  grasp.pre_grasp_approach.desired_distance = 0.1;
  grasp.pre_grasp_approach.direction.header.frame_id = "odom_combined";
  grasp.pre_grasp_approach.direction.vector.y = 1.0;

  grasp.post_grasp_retreat.min_distance = 0.08;
  grasp.post_grasp_retreat.desired_distance = 0.1;
  grasp.post_grasp_retreat.direction.header.frame_id = "odom_combined";
  grasp.post_grasp_retreat.direction.vector.y = -1.0;

  res.grasps.push_back(grasp);
  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_grasps_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("plan_grasps", serviceCB);
  ros::spin();

  return 0;
}
