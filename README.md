# Quaternion tools

## quat_to_rpy

Listens to messages from the input topic and publishes angle/axis, length and roll/pitch/yaw from Quaternion member.

Example:

ROS_NAMESPACE=debug_quat rosrun quaternion_tools quat_to_rpy input:=/ben/sensors/posmv/orientation

## rpy_to_hpr_degrees

Republishes roll, pitch and yaw in radians to heading, pitch and roll in degrees.
