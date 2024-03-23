#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/GetMap.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>

ros::Publisher pc_map_pub;

void save_grid_as_pointcloud(std::string map_filename)
{   
    ros::NodeHandle nh_;
    ros::ServiceClient map_client = nh_.serviceClient<nav_msgs::GetMap>("static_map");
    
    nav_msgs::OccupancyGrid grid_map;
    double grid_origin_x;
    double grid_origin_y;
    nav_msgs::GetMap mapsrv;

    while(ros::ok())
    {
        if(map_client.call(mapsrv))
        {
            //get grid map data
            grid_map = mapsrv.response.map;
            grid_origin_x = grid_map.info.origin.position.x;
            grid_origin_y = grid_map.info.origin.position.y;
            ROS_INFO("Received Map: width=%d, height=%d, resolution=%.3f\n", grid_map.info.width, grid_map.info.height, grid_map.info.resolution);
            ROS_INFO("Received Map: origin_x=%.3f, origin_y=%.3f\n", grid_origin_x, grid_origin_y);
             

            // 遍歷地圖中的每個格子，將非空格子轉換為點
            pcl::PointCloud<pcl::PointXYZ>::Ptr map_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointXYZ point;
            for (int i = 0; i < grid_map.info.width * grid_map.info.height; i++) 
            {
                int occupancy = grid_map.data[i];
                //std::cout << occupancy << std::endl;
                if (occupancy == 100) {  // 牆壁     -1:地圖灰色部份 , 0:地圖白色部份，也就是地板 , 100:地圖牆壁部份(黑色的)
                    point.x = (i % grid_map.info.width) * grid_map.info.resolution + grid_map.info.resolution/2 + grid_origin_x;
                    point.y = (i / grid_map.info.width) * grid_map.info.resolution + grid_map.info.resolution/2 + grid_origin_y;
                    point.z = 0.0;  // 假設地圖是二維的，將 z 軸設置為 0

                    map_pointcloud->push_back(point);
                }
            }
            // 將點雲保存為PCD文件
            pcl::io::savePCDFileASCII(map_filename, *map_pointcloud);

            //publish the point cloud map
            sensor_msgs::PointCloud2 map_pc_msg;
            pcl::toROSMsg(*map_pointcloud, map_pc_msg);
            map_pc_msg.header.stamp = ros::Time::now();
            map_pc_msg.header.frame_id = "map";
            ros::Rate rate_1hz(1);
            while(ros::ok())
            {
                pc_map_pub.publish(map_pc_msg); 
                ros::spinOnce();
                rate_1hz.sleep();
               //ROS_INFO("map_pc has been published");
            }
            
            
            break;
        }

        else
        {
            ROS_ERROR("Failed to call static_map service, trying again");
            ros::Duration(0.5).sleep();
        } 
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_to_pc_map");
    ros::NodeHandle nh("~");
    std::string map_file_name;
    nh.param<std::string>("map_name", map_file_name, "/home/mingzhun/lab_localization/point_cloud_map.pcd");
    std::cout << map_file_name << std::endl;
    pc_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_pc", 10);

    save_grid_as_pointcloud(map_file_name);


    ros::spin();
    return 0;
}