#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_action_grasp");
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<geometry_msgs::PoseStamped>("dex_grasp_pose", 1);

  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "odom_combined";
  msg.pose.position.x = 0.67;
  msg.pose.position.y = 0.05;
  msg.pose.position.z = 1.03;
  msg.pose.orientation.x = 0.37;
  msg.pose.orientation.y = -0.37;
  msg.pose.orientation.z = -0.6;
  msg.pose.orientation.w = 0.6;

  ros::Rate loop_rate(0.1);
  ros::Rate check_rate(10);

  while(ros::ok())
  {
    while(publisher.getNumSubscribers() == 0)
    {
      check_rate.sleep();
    }
    ROS_INFO("Publish home position.");
    publisher.publish(msg);
    loop_rate.sleep();
  }
  return 0;
}
