#include <moveit_gpd_pr2_pick/pick_place_action.h>
#include <tf/transform_datatypes.h>
#include <termio.h>

int scanKeyboard()
{
int in;
struct termios new_settings;
struct termios stored_settings;
tcgetattr(0,&stored_settings);
new_settings = stored_settings;
new_settings.c_lflag &= (~ICANON);
new_settings.c_cc[VTIME] = 0;
tcgetattr(0,&stored_settings);
new_settings.c_cc[VMIN] = 1;
tcsetattr(0,TCSANOW,&new_settings);

in = getchar();

tcsetattr(0,TCSANOW,&stored_settings);
return in;
}

PR2_Shadow_Grasp::PR2_Shadow_Grasp(ros::NodeHandle& node) : pick_action_client_(node, "pickup", false)
{
  arm_ = new moveit::planning_interface::MoveGroupInterface("right_arm_and_hand");
  hand_ = new moveit::planning_interface::MoveGroupInterface("right_hand");
  psi_ = new moveit::planning_interface::PlanningSceneInterface();

  pick_action_client_.waitForServer();

  // Subscribe to the grasp candidates topic to get grasping information
  plan_pick_sub_ = node.subscribe("dex_grasp_pose", 1, &PR2_Shadow_Grasp::planPickCallback, this);
}

void PR2_Shadow_Grasp::spawnObject(const geometry_msgs::PoseStamped pose_)
{
  moveit_msgs::CollisionObject object;
  object.header.frame_id = pose_.header.frame_id;
  object.id = "object";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.03;
  primitive.dimensions[1] = 0.03;
  primitive.dimensions[2] = 0.03;

  geometry_msgs::Pose pose = pose_.pose;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);

  object.operation = object.ADD;
  object_ = object;
  psi_->applyCollisionObject(object);
}

void PR2_Shadow_Grasp::planPickCallback(const geometry_msgs::PoseStamped& msg)
{
  ROS_INFO("Planning pick...");
  moveit_msgs::PickupGoal goal;

  // Spawn object
  spawnObject(msg);

  // Set goal
  goal.target_name = object_.id;
  goal.group_name = arm_->getName();
  goal.end_effector = hand_->getName();
  goal.allowed_planning_time = 30;
  goal.allow_gripper_support_collision = true;
  goal.planner_id = "RRTstartkConfigDefault";

  goal.possible_grasps = generateGrasp(msg);
  if (goal.possible_grasps.size() == 0)
  {
    ROS_ERROR("No possible grasps found");
    return;
  }

  goal.planning_options.plan_only = true;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  // send goal
  // ROS_INFO_STREAM(goal);
  pick_action_client_.sendGoal(goal);
  pick_action_client_.waitForResult();

  moveit_msgs::PickupResult result = *(pick_action_client_.getResult());

  // If we plan successfully we can decide whether to pick or just give up.
  if(result.error_code.val == 1)
  {
    ROS_INFO_STREAM("Plan found for object " << object_.id);
    current_trajectories_.clear();
    current_object_ = object_.id;

    for(int i = 0; i < result.trajectory_stages.size(); i++)
    {
      current_trajectories_.push_back(result.trajectory_stages[i]);
    }

    ROS_INFO("Enter 'p' for 'Pick' or other keys for 'Give Up' the plan");

    if (scanKeyboard() == 112)
    {
      executePick();
      ROS_INFO("Picking complete, moving aside...");

      // moving aside
      moveit::planning_interface::MoveGroupInterface arm("right_arm");
      arm.setNamedTarget("right_arm_to_side");
      arm.move();
      // open hand and drop the object
      moveit::planning_interface::MoveGroupInterface hand("right_hand");
      hand.setNamedTarget("open");
      hand.move();
      // drop object
      moveit_msgs::AttachedCollisionObject drop_object;
      drop_object.link_name = "rh_palm";
      drop_object.object = object_;
      drop_object.object.operation = drop_object.object.REMOVE;
      drop_object.touch_links = hand_->getLinkNames();

      psi_->applyAttachedCollisionObject(drop_object);

      //moving to home position


    }
    else
    {
      ROS_INFO("Give up the plan and wait for next object.");
      current_trajectories_.clear();
    }


  }
  // No plan successfully
  else
  {
    ROS_INFO_STREAM("No plan found for object " << object_.id);
    ROS_ERROR_STREAM(result.error_code);
  }
}

std::vector<moveit_msgs::Grasp> PR2_Shadow_Grasp::generateGrasp(const geometry_msgs::PoseStamped pose_)
{
  // check if the object is spawned correctly
  std::map<std::string, moveit_msgs::CollisionObject> objects =
    psi_->getObjects(std::vector<std::string>{object_.id});

  if(objects.size() == 0)
  {
    ROS_ERROR_STREAM("Object " << object_.id << " not found.");
    return std::vector<moveit_msgs::Grasp>{};
  }

  // set grasp
  moveit_msgs::Grasp grasp;
  grasp.id = "grasp";

  jointValuesToJointTrajectory(hand_->getNamedTargetValues("open"), ros::Duration(2.0), grasp.pre_grasp_posture);
  jointValuesToJointTrajectory(hand_->getNamedTargetValues("pack"), ros::Duration(2.0), grasp.grasp_posture);

  // Set the offset to grasp_pose
  double offset = -0.1;
  geometry_msgs::PoseStamped pose;
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose_.pose.orientation, q);
  tf::Matrix3x3 orientation(q);
  tf::Vector3 direction = orientation.getColumn(2);
  pose = pose_;
  pose.pose.position.x = pose.pose.position.x + offset*direction.getX();
  pose.pose.position.y = pose.pose.position.y + offset*direction.getY();
  pose.pose.position.z = pose.pose.position.z + offset*direction.getZ();

  grasp.grasp_pose = pose;

  grasp.pre_grasp_approach.min_distance = 0.08;
  grasp.pre_grasp_approach.desired_distance = 0.1;
  grasp.pre_grasp_approach.direction.header.frame_id = arm_->getEndEffectorLink();
  grasp.pre_grasp_approach.direction.vector.y = -1.0;

  grasp.post_grasp_retreat.min_distance = 0.08;
  grasp.post_grasp_retreat.desired_distance = 0.1;
  grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  grasp.post_grasp_retreat.direction.vector.z = 1.0;

  return std::vector<moveit_msgs::Grasp>{grasp};
}

void PR2_Shadow_Grasp::jointValuesToJointTrajectory(std::map<std::string, double> target_values, ros::Duration duration,
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


void PR2_Shadow_Grasp::executePick()
{
  ROS_INFO("Execute pick");

  if(current_trajectories_.size() > 0)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // pre grasp pose
    plan.trajectory_ = current_trajectories_[0];
    if(!arm_->execute(plan))
    {
      ROS_ERROR("Movement to pre grasp pose failed.");
      return;
    }
    // open gripper
    plan.trajectory_ = current_trajectories_[1];
    if(!arm_->execute(plan))
    {
      ROS_ERROR("Open gripper failed.");
      return;
    }
    // grasp pose
    plan.trajectory_ = current_trajectories_[2];
    if(!arm_->execute(plan))
    {
      ROS_ERROR("Movement to grasp pose failed.");
      return;
    }
    // close gripper
    plan.trajectory_ = current_trajectories_[3];
    if(!arm_->execute(plan))
    {
      ROS_ERROR("Close gripper failed.");
      return;
    }

    // attach object
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "rh_palm";
    attached_object.object = object_;
    attached_object.touch_links = hand_->getLinkNames();

    psi_->applyAttachedCollisionObject(attached_object);

    // retreat
    plan.trajectory_ = current_trajectories_[4];
    if(!arm_->execute(plan))
    {
      ROS_ERROR("Retreat failed.");
      return;
    }


  }
  current_trajectories_.clear();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PR2_Shadow_Grasp");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  PR2_Shadow_Grasp pr2_shadow_grasp(n);
  ros::waitForShutdown();

  return 0;
}
