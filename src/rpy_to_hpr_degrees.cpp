#include <ros/ros.h>
#include <std_msgs/Float64.h>


ros::Publisher heading_pub, pitch_pub, roll_pub;

void roll_callback(const std_msgs::Float64::ConstPtr msg)
{
  std_msgs::Float64 f;
  f.data = msg->data*180.0/M_PI;
  roll_pub.publish(f);
}

void pitch_callback(const std_msgs::Float64::ConstPtr msg)
{
  std_msgs::Float64 f;
  f.data = msg->data*180.0/M_PI;
  pitch_pub.publish(f);
}

void yaw_callback(const std_msgs::Float64::ConstPtr msg)
{
  std_msgs::Float64 f;
  f.data = 90.0-msg->data*180.0/M_PI;
  heading_pub.publish(f);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rpy_to_hpr_degrees");
  ros::NodeHandle nh;
  
  ros::Subscriber roll_sub = nh.subscribe("roll", 10, roll_callback);
  ros::Subscriber pitch_sub = nh.subscribe("pitch", 10, pitch_callback);
  ros::Subscriber yaw_sub = nh.subscribe("yaw", 10, yaw_callback);

  heading_pub = nh.advertise<std_msgs::Float64>("heading_degrees", 10);
  pitch_pub = nh.advertise<std_msgs::Float64>("pitch_degrees", 10);
  roll_pub = nh.advertise<std_msgs::Float64>("roll_degrees", 10);

  ros::spin();
  return 0;
}