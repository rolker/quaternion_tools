#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

ros::Publisher transform_pub;
tf2_ros::Buffer tfBuffer;

std::string source_frame = "map";
std::string target_frame = "base_link";

void timer_callback(const ros::TimerEvent& event)
{
  try
  {
    transform_pub.publish(tfBuffer.lookupTransform(target_frame, source_frame, event.current_real, ros::Duration(1.0)));
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_transform");
  ros::NodeHandle nh;

  double period = 0.2;
  ros::param::param("~period", period, period);

  ros::param::param("~source_frame", source_frame, source_frame);
  ros::param::param("~target_frame", target_frame, target_frame);
  
  tf2_ros::TransformListener tfListener(tfBuffer);

  transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 10);
  
  ros::Timer timer = nh.createTimer(ros::Duration(period), timer_callback);

  ros::spin();
  return 0;
}
