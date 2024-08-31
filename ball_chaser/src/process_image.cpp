#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "ball_chaser/DriveToTarget.h"

ros::ServiceClient cmd_robot_client;

void handle_drive_bot(float linear_x, float angular_z)
{
  ROS_INFO_STREAM("Moving the car towards the ball");
  ball_chaser::DriveToTarget req_msg;
  req_msg.request.angular_z = angular_z;
  req_msg.request.linear_x = linear_x;

  if (!cmd_robot_client.call(req_msg))
  {
    ROS_ERROR("Failed to call service /baller_chaser/command_robot");
  }
}

void image_processor_cb(sensor_msgs::Image img)
{
  int white_pixel = 255;
  bool ball_found = false;
  int ball_position = -1;
  int img_width = img.width;

  for (int i = 0; i < img.height * img.step; i += 3)
  {
    if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel)
    {
      ball_found = true;
      ball_position = (i / 3) % img_width;
      break;
    }
  }

  if (ball_found)
  {
    float linear_x = 0.5; // Forward speed
    float angular_z = 0.0;

    if (ball_position < img_width / 3)
    {
      angular_z = 0.5; // Turn left
    }
    else if (ball_position > 2 * img_width / 3)
    {
      angular_z = -0.5; // Turn right
    }

    handle_drive_bot(linear_x, angular_z);
  }
  else
  {
    // No ball is found
    handle_drive_bot(0.0, 0.0);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "process_image");

  ros::NodeHandle n;

  ros::Subscriber image_sub = n.subscribe("/camera/rgb/image_raw", 10, image_processor_cb);

  cmd_robot_client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  ros::spin();

  return 0;
}