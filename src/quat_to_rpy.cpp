#include "ros_type_introspection/ros_introspection.hpp"
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>

using namespace RosIntrospection;
using topic_tools::ShapeShifter;

ros::Publisher length_pub, angle_pub, axis_pub, roll_pub, pitch_pub, yaw_pub;

void publishQuaternion(const RosIntrospection::ROSType&, Span<uint8_t>& buffer)
{
  geometry_msgs::Quaternion quaternion;
  ros::serialization::IStream is( buffer.data(), buffer.size() );
  ros::serialization::deserialize(is, quaternion);

  tf2::Quaternion q;
  tf2::fromMsg(quaternion, q);


  std_msgs::Float64 length;
  length.data = q.length();
  length_pub.publish(length);

  std_msgs::Float64 angle;
  angle.data = q.getAngle();
  angle_pub.publish(angle);

  axis_pub.publish(tf2::toMsg(q.getAxis()));

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  std_msgs::Float64 f;
  f.data = roll;
  roll_pub.publish(f);
  f.data = pitch;
  pitch_pub.publish(f);
  f.data = yaw;
  yaw_pub.publish(f);
};


void callback(const ShapeShifter::ConstPtr& msg, RosIntrospection::Parser& parser)
{
  const std::string&  datatype   =  msg->getDataType();
  const std::string&  definition =  msg->getMessageDefinition();

  parser.registerMessageDefinition("input", RosIntrospection::ROSType(datatype), definition);
  
  std::vector<uint8_t> buffer( msg->size() );
  ros::serialization::OStream stream(buffer.data(), buffer.size());
  msg->write(stream);
  
  Span<uint8_t> buffer_span(buffer);

  const RosIntrospection::ROSType quaternion_type( ros::message_traits::DataType<geometry_msgs::Quaternion>::value() ) ;
  parser.applyVisitorToBuffer("input", quaternion_type, buffer_span, publishQuaternion);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quat_to_rpy");
  ros::NodeHandle nh;
  
  Parser parser;

  boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback_function;
  callback_function  = [&parser](const topic_tools::ShapeShifter::ConstPtr& msg) -> void
  {
      callback(msg, parser) ;
  };
  ros::Subscriber subscriber = nh.subscribe("input", 10, callback_function);

  length_pub = nh.advertise<std_msgs::Float64>("length", 10);
  angle_pub = nh.advertise<std_msgs::Float64>("angle", 10);
  axis_pub = nh.advertise<geometry_msgs::Vector3>("axis", 10);
  roll_pub = nh.advertise<std_msgs::Float64>("roll", 10);
  pitch_pub = nh.advertise<std_msgs::Float64>("pitch", 10);
  yaw_pub = nh.advertise<std_msgs::Float64>("yaw", 10);

  ros::spin();
  return 0;
}