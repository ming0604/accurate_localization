#include <ros/ros.h>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/GetMap.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "tf2/LinearMath/Transform.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <sensor_msgs/PointCloud2.h>


using namespace std;
class base_gt
{
private:
    ros::NodeHandle _nh;
    ros::Subscriber vive_gt_sub;
    ros::Publisher base_gt_pub;
    ros::Publisher base_gt_path_pub;

    tf2::Transform vive_to_viveodom_transform;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    tf::TransformListener listener;
    tf::StampedTransform base_to_vive_tf;
    tf::StampedTransform vive_odom_to_map_tf;
    tf::Transform base_to_map_tf;
    tf::Transform vive_to_viveodom_transform_tf;
    double vive_x;
    double vive_y;
    double vive_z;
    double vive_yaw;
    bool getvivetf;

    geometry_msgs::TransformStamped base_to_vive;
    geometry_msgs::TransformStamped vive_odom_to_map;
    tf2::Transform base_to_vive_tf2;
    tf2::Transform vive_odom_to_map_tf2;
    tf2::Transform base_to_map_tf2;
    geometry_msgs::Transform base_to_map;

    geometry_msgs::PoseStamped base_gt_pose;
    nav_msgs::Path base_gt_path;

    tf::Transform base_to_vive_gt_tf;
    double base_gt_offset_x;
    double base_gt_offset_y;
public:
    base_gt(ros::NodeHandle nh): tfListener(tfBuffer)
    {
        _nh = nh;
        vive_gt_sub = _nh.subscribe("/gt", 5, &base_gt::vivecallback, this);
        base_gt_pub = _nh.advertise<geometry_msgs::PoseStamped>("/base_gt",1);
        base_gt_path_pub = _nh.advertise<nav_msgs::Path>("/base_gt_path", 1);
        getvivetf = false;
    }
    ~base_gt()
    {}

    tf::Transform createTf_from_XYyaw(double x, double y, double theta)
    {   
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, theta);
        transform.setRotation(q);
        return transform;
    }

    void vivecallback(const nav_msgs::Odometry::ConstPtr& viveMsg)
    {   
        /*
        if(!getvivetf)
        {
            try
            {
                // get "target_frame" to "source_frame" transformation
                base_to_vive = tfBuffer.lookupTransform("vive_pose", "base_link", ros::Time(0));
                vive_odom_to_map = tfBuffer.lookupTransform("map", "vive_odom", ros::Time(0));
                tf2::convert(base_to_vive.transform,base_to_vive_tf2);
                tf2::convert(vive_odom_to_map.transform,vive_odom_to_map_tf2);
                getvivetf = true;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
            }
        }
        
        tf2::convert(viveMsg->pose.pose , vive_to_viveodom_transform);
        base_to_map_tf2 = vive_odom_to_map_tf2*vive_to_viveodom_transform*base_to_vive_tf2;
        tf2::convert(base_to_map_tf2 ,base_to_map);

        

        tf2::toMsg(base_to_map_tf2, base_gt_pose.pose);
        base_gt_pose.header.stamp = viveMsg->header.stamp;
        base_gt_pose.header.frame_id = "map";
        base_gt_pub.publish(base_gt_pose);
        */
       
        if(!getvivetf)
        {
            try
            {
                listener.lookupTransform("vive_pose", "base_link", ros::Time(0),base_to_vive_tf);
                listener.lookupTransform("map", "vive_odom", ros::Time(0),vive_odom_to_map_tf);
                getvivetf = true;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
            }
        }
        
        vive_x = viveMsg->pose.pose.position.x;
        vive_y = viveMsg->pose.pose.position.y;
        vive_yaw = tf::getYaw(viveMsg->pose.pose.orientation);
        vive_to_viveodom_transform_tf = createTf_from_XYyaw(vive_x,vive_y,vive_yaw);
        
        /*
        vive_x = viveMsg->pose.pose.position.x;
        vive_y = viveMsg->pose.pose.position.y;
        vive_z = viveMsg->pose.pose.position.z;
        vive_to_viveodom_transform_tf.setOrigin(tf::Vector3(vive_x,vive_y,vive_z));
        tf::Quaternion rotation(viveMsg->pose.pose.orientation.x, viveMsg->pose.pose.orientation.y, viveMsg->pose.pose.orientation.z, viveMsg->pose.pose.orientation.w);
        vive_to_viveodom_transform_tf.setRotation(rotation);
        cout << "x:" << vive_to_viveodom_transform_tf.getOrigin().x() << " y:" << vive_to_viveodom_transform_tf.getOrigin().y()
        << " z:" << vive_to_viveodom_transform_tf.getOrigin().z() << endl;
        double roll, pitch, yaw;;
        tf::Matrix3x3 matrix(vive_to_viveodom_transform_tf.getRotation());
        matrix.getRPY(roll, pitch, yaw);
        cout<< "pitch: " << pitch << " roll: " << roll << " yaw: " << yaw << endl;
        */

        base_to_map_tf = vive_odom_to_map_tf*vive_to_viveodom_transform_tf*base_to_vive_tf;
        
        base_gt_pose.pose.position.x = base_to_map_tf.getOrigin().x();
        base_gt_pose.pose.position.y = base_to_map_tf.getOrigin().y();
        base_gt_pose.pose.position.z = base_to_map_tf.getOrigin().z();
        tf::Quaternion q;
        q = base_to_map_tf.getRotation();
        geometry_msgs::Quaternion orientation_msg;
        tf::quaternionTFToMsg(q, orientation_msg);
        
        base_gt_pose.pose.orientation = orientation_msg;
        base_gt_pose.header.stamp = viveMsg->header.stamp;
        base_gt_pose.header.frame_id = "map";
        base_gt_pub.publish(base_gt_pose);
        
        //publish base_gt path
        base_gt_path.header.frame_id = "map";
        base_gt_path.header.stamp = viveMsg->header.stamp;
        base_gt_path.poses.push_back(base_gt_pose);
        base_gt_path_pub.publish(base_gt_path);
        /*
        //validation
        base_to_vive_gt_tf = vive_to_viveodom_transform_tf.inverse()*vive_odom_to_map_tf.inverse()*base_to_map_tf;
        base_gt_offset_x = base_to_vive_gt_tf.getOrigin().x();
        base_gt_offset_y = base_to_vive_gt_tf.getOrigin().y();
        cout << "true offset : " << "x= " << base_to_vive_tf.getOrigin().x() <<", y= " << base_to_vive_tf.getOrigin().y() << endl;
        cout << "estimated offset : " << "x= " << base_gt_offset_x << ", y= " << base_gt_offset_y << endl;
        */
    }
};


int main(int argc, char** argv) 
{
    ros::init (argc, argv, "base_link_gt");
    ros::NodeHandle nh;
    base_gt base_gt(nh);
    ros::spin();
    return 0;
}