#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>



int main(int argc, char **argv)
{
  ros::init(argc,argv, "my_static_tf2_broadcaster");
 
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "map";
  static_transformStamped.child_frame_id = "vive_odom";
  static_transformStamped.transform.translation.x = 0.3;
  static_transformStamped.transform.translation.y = 0.3;
  static_transformStamped.transform.translation.z = 0.399;
  
  static_transformStamped.transform.rotation.x = 0;
  static_transformStamped.transform.rotation.y = 0;
  static_transformStamped.transform.rotation.z = 0;
  static_transformStamped.transform.rotation.w = 1;
  static_broadcaster.sendTransform(static_transformStamped);
  ros::Duration(20).sleep();
  return 0;
};