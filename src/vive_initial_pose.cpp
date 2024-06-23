#include <ros/ros.h>
#include <algorithm>
#include <vector>

#include <sensor_msgs/LaserScan.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>

#include "tf2/LinearMath/Transform.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>

using namespace std;

typedef struct ICP_result
{
    double score;
    ros::Time timestamp;
    Eigen::Matrix4f result;
}ICP_result;

bool icp_score_compare(const ICP_result& a, const ICP_result& b)
{
    return a.score < b.score;
}

class initializer
{   
    private:
        ros::NodeHandle _nh;

        ros::Subscriber scan_sub;
        ros::Publisher final_init_pose_pub;
        ros::Publisher final_pc_pub;

        float laser_angle_min;
        float laser_angle_max;
        float laser_angle_increment;
        float laser_time_increment;
        float range_max;
        float range_min;
        double init_x, init_y, init_yaw;
        double max_corres_distance;

        vector<float> ranges;
        vector<ICP_result> results;

        ros::Time scan_time_stamp;

        pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc;
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc;
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc_at_base;
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_pc_at_map;
        
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        tf2_ros::StaticTransformBroadcaster static_br;
        geometry_msgs::TransformStamped laser_to_base;
        geometry_msgs::TransformStamped vive_pose_to_base;
        tf2::Transform laser_to_base_tf2;
        tf2::Transform vive_pose_to_base_tf2;
        tf2::Transform vive_pose_to_map_tf2;
        Eigen::Isometry3d laser_to_base_eigen;
        Eigen::Matrix4f init_guess;
        Eigen::Matrix4f icp_best_result_transform;

        std::string pc_map_path;

        int cnt;
        bool pc_map_received;
        bool laser_to_base_received;

    public:
        initializer(ros::NodeHandle nh);
        ~initializer();

        void scan_callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan);
        void load_pc_map();
        void tf2_listener_laser();
        void tf2_listener_tracker(); 
        void vive_odom_calibration_tf_publish(tf2::Transform icp_result_tf2);
        void final_init_pose_publish(const geometry_msgs::Pose& init);
        Eigen::Matrix4f set_init_guess(float x, float y, float yaw);
        void aligned_pc_publish(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc, ros::Time t);
        //transform scan to point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr create_scan_pc();
        
        




};

initializer::initializer(ros::NodeHandle nh):tfListener(tfBuffer)
{
    _nh = nh;
    scan_sub = _nh.subscribe("/scan", 1 , &initializer::scan_callback, this);
    final_init_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("ICP_init_pose",1);
    final_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/final_init_pc", 10);

    //get parameters
    _nh.param<string>("pc_map_path", pc_map_path ,"/Default/path");
    _nh.param("initial_pose_x", init_x , 0.0);
    _nh.param("initial_pose_y", init_y , 0.0);
    _nh.param("initial_pose_yaw", init_yaw, 0.0);
    _nh.param("ICP_max_corres_distance", max_corres_distance,1.0);

    map_pc.reset(new pcl::PointCloud<pcl::PointXYZ>);
    scan_pc.reset(new pcl::PointCloud<pcl::PointXYZ>);
    scan_pc_at_base.reset(new pcl::PointCloud<pcl::PointXYZ>);
    final_pc_at_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
    //set initial pose at map origin
    init_guess = set_init_guess(static_cast<float>(init_x), static_cast<float>(init_y), static_cast<float>(init_yaw));
    //init_guess.setIdentity();

    cnt = 0;
    pc_map_received = false;
    laser_to_base_received = false;

    ROS_INFO("ICP_max_corres_distance: %lf\n",max_corres_distance);

}

initializer::~initializer()
{
    ROS_WARN("Exit vive_initializer");
}

void initializer::load_pc_map()
{
    while(pcl::io::loadPCDFile<pcl::PointXYZ>(pc_map_path, *map_pc) == -1) 
    {
        ROS_ERROR("Couldn't read file your_point_cloud.pcd");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("point cloud map is loaded successfully");
}

void initializer::tf2_listener_laser() 
{
    try{
        //"base_link is target,laser is source"
        laser_to_base = tfBuffer.lookupTransform("base_link", "laser", ros::Time(0));
        tf2::fromMsg(laser_to_base.transform,laser_to_base_tf2);
        laser_to_base_eigen = tf2::transformToEigen(laser_to_base.transform);
        laser_to_base_received = true;
    }
    catch (tf2::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        // exception
        return;
    }
    
}

void initializer::tf2_listener_tracker() 
{
    try{
        //"base_link is target,tracker is source"
        vive_pose_to_base = tfBuffer.lookupTransform("base_link", "vive_pose", ros::Time(0));
        tf2::fromMsg(vive_pose_to_base.transform,vive_pose_to_base_tf2);
    }
    catch (tf2::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        // exception
        return;
    }
    
}

Eigen::Matrix4f initializer::set_init_guess(float x, float y, float yaw)
{   
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix(0, 0) = cos(yaw);
    matrix(0, 1) = -sin(yaw);
    matrix(0, 3) = x;

    matrix(1, 0) = sin(yaw);
    matrix(1, 1) = cos(yaw);
    matrix(1, 3) = y;
    return matrix;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr initializer::create_scan_pc()
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO("start converting scan to pc");
    float angle,x,y;
    pcl::PointXYZ point;
    cout << "ranges.size = " << ranges.size() <<endl;
    // go through all the angles
    for (int i = 0; i < ranges.size(); i++)
    {   
        // count the angle od the scan
        angle = laser_angle_min + i * laser_angle_increment;

        // change into Cartesian coordinates
        x = ranges[i] * cos(angle);
        y = ranges[i] * sin(angle);

        // add the point into the points cloud
        point.x = x;
        point.y = y;
        point.z = 0.0;  
        pc->push_back(point);
        //ROS_INFO("scan to pc %d complete",i);
    }

    ROS_INFO("scan to pc complete");
    return pc;
}

void initializer::final_init_pose_publish(const geometry_msgs::Pose& init)
{
    geometry_msgs::PoseStamped final_init_pose;
    final_init_pose.pose = init;
    final_init_pose.header.frame_id = "map";
    final_init_pose.header.stamp = ros::Time::now();
    final_init_pose_pub.publish(final_init_pose);

    //print out the initial pose
    double x,y,yaw;
    x = final_init_pose.pose.position.x;
    y = final_init_pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(final_init_pose.pose.orientation,q);
    yaw = tf2::getYaw(q);
    
    ROS_INFO("final inaitial pose: x=%lf, y=%lf, yaw=%lf\n",x,y,yaw);
    
}

void initializer::vive_odom_calibration_tf_publish(tf2::Transform icp_result_tf2)
{   
    //get tracker to base tf
    tf2_listener_tracker();
    //compute the tracker to map tf as correct vive initial point
    tf2::Transform vive_odom_to_map_calibration_tf2;

    //icp result is initial base to map tf
    vive_odom_to_map_calibration_tf2 = icp_result_tf2*vive_pose_to_base_tf2;

    //print vive_odom pose on map
    double x,y,yaw;
    x = vive_odom_to_map_calibration_tf2.getOrigin().x();
    y = vive_odom_to_map_calibration_tf2.getOrigin().y();
    yaw = tf2::getYaw(vive_odom_to_map_calibration_tf2.getRotation());
    ROS_INFO("vive inaitial pose: x=%lf, y=%lf, yaw=%lf\n",x,y,yaw);
    //publish new tracker initial pose tf (vive_odom_to_map_calibration_tf)
    geometry_msgs::TransformStamped vive_odom_to_map_static_transformStamped;

    vive_odom_to_map_static_transformStamped.header.stamp = ros::Time::now();
    vive_odom_to_map_static_transformStamped.header.frame_id = "map";
    vive_odom_to_map_static_transformStamped.child_frame_id = "vive_odom";
    tf2::convert(vive_odom_to_map_calibration_tf2,vive_odom_to_map_static_transformStamped.transform);

    static_br.sendTransform(vive_odom_to_map_static_transformStamped);
    ROS_INFO("vive_origin tf has been published after calibration");
}

void initializer::aligned_pc_publish(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc, ros::Time t)
{
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*pc,pc_msg);
    pc_msg.header.stamp = t;
    pc_msg.header.frame_id = "map";
    final_pc_pub.publish(pc_msg);
    ROS_INFO("final initial aligned point cloud is published successfully");
}

void initializer::scan_callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
{   
    laser_angle_min = laser_scan->angle_min;
    laser_angle_max = laser_scan->angle_max;
    laser_angle_increment = laser_scan->angle_increment;
    laser_time_increment = laser_scan->time_increment;
    ROS_INFO("Received Laser Scan with angle min: %f, angle max: %f", laser_scan->angle_min, laser_scan->angle_max);
    
    ranges = laser_scan->ranges;
    range_max = laser_scan->range_max;
    range_min = laser_scan->range_min;
    scan_time_stamp = laser_scan->header.stamp;
    ROS_INFO("Received Laser Scan timestamp: %f", scan_time_stamp.toSec());
    
    //load pointcloud map
    if(!pc_map_received)
    {
        load_pc_map();
        pc_map_received = true;
    }

    //get laser to base transformation
    if(!laser_to_base_received)
    {
        tf2_listener_laser();
    }

    //create scan pc
    scan_pc = create_scan_pc();
    //transform scan points from laser to base_link
    pcl::transformPointCloud(*scan_pc,*scan_pc_at_base,laser_to_base_eigen.matrix());
    for (size_t i=0; i<scan_pc_at_base->size(); i++) 
    {
        scan_pc_at_base->points[i].z = 0;
    }

    //do ICP
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance (max_corres_distance);
    icp.setMaximumIterations (300);
    icp.setTransformationEpsilon (1e-5);
    icp.setEuclideanFitnessEpsilon(1e-5);
    //set source pc as radar point cloud, and target pc as map point cloud
    icp.setInputSource(scan_pc_at_base);
    icp.setInputTarget(map_pc);
    //run the ICP, then get the transformation matrix after icp as new base_link
    icp.align(*output_pc, init_guess);

    if(icp.hasConverged())
    {   
        ICP_result result;
        result.result = icp.getFinalTransformation();
        result.score = icp.getFitnessScore();
        result.timestamp = scan_time_stamp;
        ROS_INFO("ICP converged, fitness score: %lf\n", result.score);
        //result of this time icp is the initial guess of next icp
        init_guess = result.result;
        results.push_back(result);
        cnt++;
    }
    else
    {
        ROS_WARN("ICP not converged");
    }

    //after performing 20 times icp, we choose the best score ones
    if(cnt == 20)
    {   
        ICP_result best_result;
        
        //find the best score(smallest score) ones as the best result
        best_result = *(std::min_element(results.begin(), results.end(),icp_score_compare));
        icp_best_result_transform = best_result.result;
        ROS_INFO("After doing 20 times ICP, best result's fitness score: %lf\n", best_result.score);

        //publish the aligned point cloud
        pcl::transformPointCloud(*scan_pc_at_base,*final_pc_at_map,icp_best_result_transform);
        aligned_pc_publish(final_pc_at_map, best_result.timestamp);
        //transform icp best result data type into tf2::Transform
        Eigen::Matrix4d icp_best_result_transform_double;
        geometry_msgs::Pose icp_best_result_msg;
        tf2::Transform icp_best_result_tf2;
        tf2::Quaternion q;
        double result_yaw;

        icp_best_result_transform_double = icp_best_result_transform.cast<double>();
        Eigen::Isometry3d T(icp_best_result_transform_double);
        
        icp_best_result_msg = tf2::toMsg(T);
        //set z=0 and roll&pitch = 0
        icp_best_result_msg.position.z = 0;
        result_yaw = tf2::getYaw(icp_best_result_msg.orientation);
        q.setRPY(0,0,result_yaw);
        icp_best_result_msg.orientation = tf2::toMsg(q);
        
        tf2::convert(icp_best_result_msg,icp_best_result_tf2);
        final_init_pose_publish(icp_best_result_msg);
        //compute vive odom to map tf after calibration and then publish it
        
        vive_odom_calibration_tf_publish(icp_best_result_tf2);
        ros::shutdown();
    }

}


int main(int argc, char** argv) 
{
    ros::init (argc, argv, "initializer");
    ros::NodeHandle nh("~");
    initializer initializer(nh);

    // subscribe scan at 2 Hz 
    ros::Rate loop_rate(2);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
