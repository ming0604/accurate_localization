#include <ros/ros.h>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/GetMap.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>

using namespace std;

class scan_to_pc
{
private:
    float amcl_pose_x;
    float amcl_pose_y;
    float amcl_pose_yaw;
    nav_msgs::OccupancyGrid grid_map;
    float laser_angle_min;
    float laser_angle_max;
    float laser_angle_increment;
    vector<float> ranges;
    vector<float> intensities;

    ros::NodeHandle _nh;

    // 订阅激光扫描数据
    ros::Subscriber laser_scan_sub;
    //subscibe amcl pose
    ros::Subscriber poseSub; 

    ros::Publisher scan_pc_pub;


public:
    scan_to_pc(ros::NodeHandle nh)
    {   
        _nh = nh;
        laser_scan_sub = _nh.subscribe("/scan", 100, &scan_to_pc::laserScanCallback,this);
        //subscibe amcl pose
        poseSub = _nh.subscribe("/amcl_pose", 1, &scan_to_pc::amclposeCallback,this);

        scan_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/scan_pc", 100);

    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr create_vitual_scan_pc()
    {
        
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr create_scan_pc()
    {   
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
        ROS_INFO("start convert");
        float angle,x,y;
        pcl::PointXYZ point;
        cout << "ranges.size = " << ranges.size() <<endl;
        // go through all the angles
        for (int i = 0; i < ranges.size(); i++)
        {   
            // 计算当前激光点的角度
            angle = laser_angle_min + i * laser_angle_increment;

            // 计算当前激光点在极坐标下的坐标
            x = ranges[i] * cos(angle);
            y = ranges[i] * sin(angle);

            // 添加点到 PointCloud 中
            point.x = x;
            point.y = y;
            point.z = 0.0;  
            pc->push_back(point);
            ROS_INFO("scan to pc %d complete",i);
        }

        ROS_INFO("scan to pc complete");
        return pc;
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
    {
        // 在这里处理激光扫描数据
        // 示例：输出激光扫描的角度范围
        laser_angle_min = laser_scan->angle_min;
        laser_angle_max = laser_scan->angle_max;
        laser_angle_increment = laser_scan->angle_increment;
        ("Received Laser Scan with angle min: %f, angle max: %f", laser_scan->angle_min, laser_scan->angle_max);
        ranges = laser_scan->ranges;
        //cout << "Ranges size: " << laser_scan->ranges.size()<<endl;
        //ROS_INFO("Received Laser Scan ranges size=%d", ranges.size());
    }

    void amclposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg)
    {
        // 处理接收到的姿态消息
        // 可以在这里执行你的逻辑操作
        amcl_pose_x = poseMsg->pose.pose.position.x;
        amcl_pose_y = poseMsg->pose.pose.position.y;
        amcl_pose_yaw = tf::getYaw(poseMsg->pose.pose.orientation);
        ROS_INFO("Received amcl Pose: x=%f, y=%f, theta=%f",amcl_pose_x,amcl_pose_y,amcl_pose_yaw);

        //get grid map use mapserver service
        ros::NodeHandle nh;
        ros::ServiceClient map_client = nh.serviceClient<nav_msgs::GetMap>("static_map");

        nav_msgs::GetMap mapsrv;
        if (map_client.call(mapsrv))
        {
            // 在这里处理获取到的栅格地图
            grid_map = mapsrv.response.map;
            // 可以在这里执行你的逻辑操作
            ROS_INFO("Received Map: width=%d, height=%d, resolution=%.3f\n", grid_map.info.width, grid_map.info.height, grid_map.info.resolution);
        }
        else
        {
            ROS_ERROR("Failed to call static_map service");
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc = create_scan_pc();
        ROS_INFO("Received scan_pc");
        sensor_msgs::PointCloud2 pc_msg;
        pcl::toROSMsg(*scan_pc, pc_msg);
        pc_msg.header.stamp = ros::Time::now();
        pc_msg.header.frame_id = "laser";
        scan_pc_pub.publish(pc_msg);
        ROS_INFO("scan_pc published");



    }
};

int main(int argc, char** argv) 
{
    ros::init (argc, argv, "scan");
    ros::NodeHandle nh;
    scan_to_pc scan_to_pc(nh);

    ros::spin();
    return 0;
}
