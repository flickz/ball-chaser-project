#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher cmd_vel_pub;

bool handle_drive_cb(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res)
{
  ROS_INFO("DriveToTargetRequest received - linear_x: %.2f, angular_z: %.2f", float(req.linear_x), float(req.angular_z));
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = req.linear_x;
  cmd_vel_msg.angular.z = req.angular_z;

  cmd_vel_pub.publish(cmd_vel_msg);

  res.msg_feedback = "Velocity linear_x: " + std::to_string(req.linear_x) + " , angular_z " + std::to_string(req.angular_z);
  ROS_INFO_STREAM(res.msg_feedback);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_bot");
  ros::NodeHandle n;

  // Create a service for ball_chaser/command_robot
  ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_cb);

  // Create a publisher
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  ros::spin();

  return 0;
}