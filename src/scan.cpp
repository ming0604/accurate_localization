#include <ros/ros.h>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/GetMap.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
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
    vector<vector<int>> map_2D;
    float grid_origin_x;
    float grid_origin_y;
    int grid_height;
    int grid_width;
    float laser_angle_min;
    float laser_angle_max;
    float laser_angle_increment;
    double laser_x;
    double laser_y;
    int laser_x_grid;
    int laser_y_grid;
    double laser_yaw;
    int occ_threshold=65;
    vector<float> ranges;
    vector<float> intensities;

    ros::NodeHandle _nh;

    // subscibe laser scan
    ros::Subscriber laser_scan_sub;
    //subscibe amcl pose
    ros::Subscriber poseSub; 

    ros::Publisher scan_pc_pub;
    ros::Publisher virtual_pc_pub;
    tf::TransformListener listener;
    tf::StampedTransform laser_map_transform;
public:
    scan_to_pc(ros::NodeHandle nh)
    {   
        _nh = nh;
        laser_scan_sub = _nh.subscribe("/scan", 100, &scan_to_pc::laserScanCallback,this);
        //subscibe amcl pose
        poseSub = _nh.subscribe("/amcl_pose", 1, &scan_to_pc::amclposeCallback,this);

        scan_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/scan_pc", 100);
        virtual_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/virtual_pc", 100);

    }

    //change map from 1D arry into 2D vector
    vector<vector<int>> convertTo2DVector(vector<signed char> data, int rows, int cols) {
        vector<vector<int>> result;
        int index = 0;

        for (int i = 0; i < rows; i++) 
        {
            vector<int> row_data(data.begin() + index, data.begin()+index+cols-1);
            result.push_back(row_data);
            index += cols;
        }
        return result;
    }

    void tf_listener()
    {
        try{
            //"map is target,laser is source"
            listener.lookupTransform("map", "laser", ros::Time(0),laser_map_transform);
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            // 处理异常
            return;
        }
        
        laser_x = laser_map_transform.getOrigin().x(),
        laser_y = laser_map_transform.getOrigin().y(),
        laser_yaw = tf::getYaw(laser_map_transform.getRotation());
        ROS_INFO("get lidar pose:  x: %f, y: %f, yaw: %f",laser_x,laser_y,laser_yaw);
    }

    float line_fx(int x,float m)
    {
        float y;
        y = m*(x-laser_x_grid)+laser_y_grid;

        return y;
    }

    float line_fy(int y,float m)
    {
        float x;
        x = (1/m)*(y-laser_y_grid)+laser_x_grid;

        return x;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr create_vitual_scan_pc()
    {   
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ point;
        tf_listener();
        laser_x_grid = (laser_x-grid_origin_x)/grid_map.info.resolution;
        laser_y_grid = (laser_y-grid_origin_y)/grid_map.info.resolution;
        cout << "laser_x_grid = " << laser_x_grid << ",laser_y_grid = " << laser_y_grid << ", laser_yaw="<< laser_yaw << endl;
        float distance_virtual;
        int index = 0;
        
        for(int i=0; i<ranges.size(); i++)
        {   
            int x_grid = laser_x_grid;
            int y_grid = laser_y_grid;
            float angle,x_map,y_map,m;
            bool find_vir_flag=false;
            angle = laser_yaw + laser_angle_min + i * laser_angle_increment;
            
            if(angle>2.0*M_PI)
            {
                angle = angle - 2.0*M_PI;
            }
            
            m = tan(angle);
            cout << "angle:" << angle <<"m: "<<m<< endl;
            //for 1＆2 Quadrant
            if(angle>=0 && angle<=M_PI)
            {   
                
                if(m>=0 && m<=1)
                {   
                    ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {
                        x_grid = x_grid+1;
                        y_grid = round(line_fx(x_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_origin_y;
                            find_vir_flag = true;
                            ROS_INFO("find virtual point! x=%f y=%f",x_map,y_map);
                            break;
                        }
                    }
                }
                else if(m>1)
                {   
                    
                    ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {
                        y_grid = y_grid+1;
                        x_grid = round(line_fy(y_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_origin_y;
                            find_vir_flag = true;
                            ROS_INFO("find virtual point! x=%f y=%f",x_map,y_map);
                            break;
                        }

                    }
                }
                else if(m<-1)
                {   
                    ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {
                        y_grid = y_grid+1;
                        x_grid = round(line_fy(y_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_origin_y;
                            find_vir_flag = true;
                            ROS_INFO("find virtual point! ");
                            break;
                        }

                    }
                }
                else if(m<0 && m>=-1)
                {   
                    ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {
                        x_grid = x_grid-1;
                        y_grid = round(line_fx(x_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_origin_y;
                            find_vir_flag = true;
                            ROS_INFO("find virtual point! ");
                            break;
                        }
                    }
                }
            }

            else
            {
                if(m>=0 && m<=1)
                {   
                    ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {
                        x_grid = x_grid-1;
                        y_grid = round(line_fx(x_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_origin_y;
                            find_vir_flag = true;
                            ROS_INFO("find virtual point! ");
                            break;
                        }
                    }
                }
                else if(m>1)
                {   
                    ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {
                        y_grid = y_grid-1;
                        x_grid = round(line_fy(y_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_origin_y;
                            find_vir_flag = true;
                            ROS_INFO("find virtual point! ");
                            break;
                        }
                    }
                }
                else if(m<-1)
                {   ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {   
                        
                        y_grid = y_grid-1;
                        x_grid = round(line_fy(y_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_origin_y;
                            find_vir_flag = true;
                            ROS_INFO("find virtual point! ");
                            break;
                        }
                    }
                }
                else if(m<0 && m>=-1)
                {   
                    ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {
                        x_grid = x_grid+1;
                        y_grid = round(line_fx(x_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_origin_y;
                            find_vir_flag = true;
                            ROS_INFO("find virtual point! x=%f y=%f",x_map,y_map);
                            break;
                        }
                    }
                }
            }

            //change into point cloud in map frame
            if(find_vir_flag)
            {
                point.x = x_map;
                point.y = y_map;
                point.z = 0.0;  
                pc->push_back(point);
            }
        }
        
        ROS_INFO("virtual pc has been created");
        return pc;
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
            //ROS_INFO("scan to pc %d complete",i);
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
            grid_origin_x = grid_map.info.origin.position.x;
            grid_origin_y = grid_map.info.origin.position.y;
            grid_height = grid_map.info.height;
            grid_width = grid_map.info.width;
            ROS_INFO("Received Map: width=%d, height=%d, resolution=%.3f\n", grid_map.info.width, grid_map.info.height, grid_map.info.resolution);
            ROS_INFO("Received Map: origin_x=%.3f, origin_y=%.3f\n", grid_origin_x, grid_origin_y);
        }
        else
        {
            ROS_ERROR("Failed to call static_map service");
        }

        map_2D = convertTo2DVector(grid_map.data, grid_map.info.height, grid_map.info.width);
        cout << "map to 2D vector complete, size=" <<map_2D.size()<< endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc = create_scan_pc();
        ROS_INFO("Received scan_pc");
        sensor_msgs::PointCloud2 scan_pc_msg;
        pcl::toROSMsg(*scan_pc, scan_pc_msg);
        scan_pc_msg.header.stamp = ros::Time::now();
        scan_pc_msg.header.frame_id = "laser";
        scan_pc_pub.publish(scan_pc_msg);
        ROS_INFO("scan_pc published");

        pcl::PointCloud<pcl::PointXYZ>::Ptr virtual_pc = create_vitual_scan_pc();
        ROS_INFO("Received virtual_pc");
        sensor_msgs::PointCloud2 virtual_pc_msg;
        pcl::toROSMsg(*virtual_pc, virtual_pc_msg);
        virtual_pc_msg.header.stamp = ros::Time::now();
        virtual_pc_msg.header.frame_id = "map";
        virtual_pc_pub.publish(virtual_pc_msg);
        ROS_INFO("virtual_pc published");
        
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
