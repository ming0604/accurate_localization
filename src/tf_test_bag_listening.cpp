#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2/LinearMath/Transform.h"


int main(int argc, char **argv)
{
  ros::init(argc,argv, "my_tf2_listener");

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
    
  ros::Rate rate(10.0);
  while (ros::ok()){
    geometry_msgs::TransformStamped transformStamped;
    tf2::Transform viveodom_to_map_tf2;
    try{
      transformStamped = tfBuffer.lookupTransform("map", "vive_odom",ros::Time(0));
      tf2::fromMsg(transformStamped.transform,viveodom_to_map_tf2);
      //print vive_odom pose on map
      double x,y,yaw;
      x = viveodom_to_map_tf2.getOrigin().x();
      y = viveodom_to_map_tf2.getOrigin().y();
      yaw = tf2::getYaw(viveodom_to_map_tf2.getRotation());
      ROS_INFO("vive inaitial pose: x=%lf, y=%lf, yaw=%lf\n",x,y,yaw);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }


    rate.sleep();
  }
  return 0;
};      