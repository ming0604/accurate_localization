#include <ros/ros.h>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/GetMap.h"
#include <nav_msgs/Path.h>
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
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/ndt_2d.h>
#include <pcl_ros/transforms.h>
#include <map>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>
#include <csm/csm_all.h> 
using namespace CSM ;
using namespace std;

enum ICP_method{
    ICP,
    GICP,
    NDT,
    PLICP
};

class ICP_with_AMCL
{
private:
    float amcl_pose_x;
    float amcl_pose_y;
    float amcl_pose_yaw; 
    double PLICP_pose_x;
    double PLICP_pose_y;
    double PLICP_pose_yaw; 
    nav_msgs::Path amcl_path;
    nav_msgs::Path PLICP_path;
    nav_msgs::Path ICP_path;
    double init_cov_[3];
    double cov_xx;
    double cov_yy;
    double cov_yawyaw;
    int re_init_count = 0;
    bool map_received;
    bool laser_to_base_received;
    bool pc_map_received;
    //map<ros::Time, sensor_msgs::LaserScan> scan_dic;
    sensor_msgs::LaserScan scan_data;

    bool amcl_init = false;
    bool scan_matching_init = false;
    ros::Time scan_time_stamp;
    ros::Time amcl_time_stamp;
    ros::WallTime scan_matching_start_time;
    ros::WallTime amcl_receive_now;
    ros::WallTime last_amcl_time;
    ros::WallTime scan_matching_end_time;
    ros::WallTime last_scan_matching_time;
    ros::WallTime scan_matching_pub_now;
    ros::WallDuration scan_matching_time_used; 
    ros::WallDuration amcl_pub_duration;
    ros::WallDuration scan_matching_pub_duration; 
   

    nav_msgs::OccupancyGrid grid_map;
    vector<vector<int>> map_2D;
    float grid_origin_x;
    float grid_origin_y;
    int grid_height;
    int grid_width;

    float laser_angle_min;
    float laser_angle_max;
    float laser_angle_increment;
    float laser_time_increment;
    float range_max;
    float range_min;

    double laser_x;
    double laser_y;
    int laser_x_grid;
    int laser_y_grid;
    double laser_yaw;
    int occ_threshold=65;
    vector<float> ranges;
    vector<float> intensities;

    vector<float> virtual_ranges;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc_at_base;
    pcl::PointCloud<pcl::PointXYZ>::Ptr virtual_pc;
    sensor_msgs::LaserScan  virtual_scan;

    sm_params input_;
    sm_result output_;
    double laser_x_plicp;
    double laser_y_plicp;
    double laser_yaw_plicp;


    ros::NodeHandle _nh;
    ros::NodeHandle private_node_;

    // subscibe laser scan
    ros::Subscriber laser_scan_sub;
    //subscibe amcl pose
    ros::Subscriber poseSub; 

    ros::Publisher scan_pc_pub;
    ros::Publisher virtual_pc_pub;
    ros::Publisher virtual_scan_pub;
    ros::Publisher amcl_path_pub;
    ros::Publisher PLICP_path_pub;
    ros::Publisher PLICP_pose_pub;
    ros::Publisher amcl_init_pub;
    ros::Publisher map_pc_pub;
    ros::Publisher ICP_pose_pub;
    ros::Publisher ICP_path_pub;
    ros::Publisher aligned_pc_pub;
    //use message_filters to ensure that the scan and pose of AMCL is at the same time
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub_;
    message_filters::TimeSynchronizer<sensor_msgs::LaserScan, geometry_msgs::PoseWithCovarianceStamped>* sync_;

    //tf::TransformListener listener;
    //tf::TransformBroadcaster br;
    tf2_ros::TransformBroadcaster br;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    geometry_msgs::TransformStamped laser_to_base;
    geometry_msgs::TransformStamped odom_to_base;
    geometry_msgs::Transform new_laser_to_map_msgs;
    tf2::Transform odom_to_base_tf2;
    tf2::Transform laser_to_base_tf2;
    tf2::Transform amcl_base_to_map_tf2;
    tf2::Transform laser_to_map_tf2;
    tf2::Transform base_to_laser_tf2;
    tf2::Transform new_base_to_map;
    tf2::Transform new_laser_to_map;
    Eigen::Isometry3d new_laser_to_map_eigen;
    Eigen::Isometry3d laser_to_base_eigen;
    Eigen::Matrix4f init_guess;
    Eigen::Matrix4f icp_result_transform;

    std::string amcl_time_save_path;
    std::string scan_matching_time_save_path;
    std::string scan_matching_pose_time_save_path;
    std::string scan_match_method_Str;
    std::string pc_map_path;
    std::ofstream file_amcl_time;
    std::ofstream file_scan_matching_time;
    std::ofstream file_scan_matching_pose_time;

    ICP_method method;

public:
    ICP_with_AMCL(ros::NodeHandle nh): tfListener(tfBuffer)
    {   
        _nh = nh;
        InitParams();
        init_cov_[0] = 0.5 * 0.5;
        init_cov_[1] = 0.5 * 0.5;
        init_cov_[2] = (M_PI/12.0) * (M_PI/12.0);
        map_received = false;
        pc_map_received = false;
        laser_to_base_received = false;
        init_guess.setIdentity();
        map_pc.reset(new pcl::PointCloud<pcl::PointXYZ>);
        scan_pc_at_base.reset(new pcl::PointCloud<pcl::PointXYZ>);
        scan_pc.reset(new pcl::PointCloud<pcl::PointXYZ>);
        virtual_pc.reset(new pcl::PointCloud<pcl::PointXYZ>);
        
        //laser_scan_sub = _nh.subscribe("/scan", 2, &scan_to_pc::laserScanCallback,this);
        //laser_scan_sub = _nh.subscribe("/amcl_scan", 2, &scan_to_pc::laserScanCallback,this);
        //subscibe amcl pose
        //poseSub = _nh.subscribe("/amcl_pose", 1, &scan_to_pc::amclposeCallback,this);
        scan_sub_.subscribe(_nh, "/amcl_scan", 1);
        pose_sub_.subscribe(_nh, "/amcl_pose", 1);
        sync_ = new message_filters::TimeSynchronizer<sensor_msgs::LaserScan, geometry_msgs::PoseWithCovarianceStamped>(scan_sub_, pose_sub_, 10);
        sync_->registerCallback(boost::bind(&ICP_with_AMCL::amclsyncCallback, this, _1, _2));

        scan_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/scan_pc", 10);
        virtual_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/virtual_pc", 10);
        virtual_scan_pub = _nh.advertise<sensor_msgs::LaserScan >("/virtual_scan", 10);
        amcl_path_pub = _nh.advertise<nav_msgs::Path>("/amcl_path", 1);
        PLICP_path_pub = _nh.advertise<nav_msgs::Path>("/PLICP_path", 1);
        PLICP_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("/PLICP_pose", 1);
        ICP_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("/ICP_pose", 2);
        ICP_path_pub = _nh.advertise<nav_msgs::Path>("/ICP_path", 2);
        amcl_init_pub = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
        map_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/map_pc", 10);
        aligned_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/aligned_pc", 10);

        _nh.param<string>("amcl_time_save_path", amcl_time_save_path,"/Default/path");
        _nh.param<string>("scan_matching_time_save_path", scan_matching_time_save_path,"/Default/path");
        _nh.param<string>("scan_matching_pose_time_save_path", scan_matching_pose_time_save_path,"/Default/path");
        _nh.param("re_initial_cov_xx", cov_xx , init_cov_[0]);
        _nh.param("re_initial_cov_yy", cov_yy, init_cov_[1]);
        _nh.param("re_initial_cov_aa", cov_yawyaw, init_cov_[2]);
        _nh.param<string>("scan_match_method", scan_match_method_Str ,"PLICP");
        _nh.param<string>("pc_map_path", pc_map_path ,"/Default/path");
        
        if(scan_match_method_Str == "ICP")
        {
            method = ICP;
        }
        else if(scan_match_method_Str == "GICP")
        {
            method = GICP;
        }
        else if(scan_match_method_Str == "NDT")
        {
            method = NDT;
        }
        else if(scan_match_method_Str == "PLICP")
        {
            method = PLICP;
        }
        else
        {
            ROS_WARN("Unknown laser model type \"%s\"; defaulting to likelihood_field model",
            scan_match_method_Str .c_str());
            method = PLICP;
        }
        cout << scan_match_method_Str << endl;
        file_amcl_time.open(amcl_time_save_path);
        file_amcl_time << "last time, this time, pub duration\n";
        file_scan_matching_time.open(scan_matching_time_save_path);
        file_scan_matching_time << "start time, end time, time duration\n";
        file_scan_matching_pose_time.open(scan_matching_pose_time_save_path);
        file_scan_matching_pose_time << "last time, this time, pub duration\n";
    }
    
    ~ICP_with_AMCL()
    {
        ROS_WARN("Exit Localization");
        file_amcl_time.close();
        file_scan_matching_time.close();
        file_scan_matching_pose_time.close();
    }

    void InitParams()
    {
        // **** CSM parameter - comments copied from algos.h (by Andrea Censi)

        // Maximum angular displacement between scans
        if (!private_node_.getParam("max_angular_correction_deg", input_.max_angular_correction_deg))
        { 
            input_.max_angular_correction_deg = 90;
        }   

        // Maximum translation between scans (m)
        if (!private_node_.getParam("max_linear_correction", input_.max_linear_correction))
        {
             input_.max_linear_correction = 3;
        }   

        // Maximum ICP cycle iterations
        if (!private_node_.getParam("max_iterations", input_.max_iterations))
        {
             input_.max_iterations = 10;
        }   

        // A threshold for stopping (m)
        if (!private_node_.getParam("epsilon_xy", input_.epsilon_xy))
        {
             input_.epsilon_xy = 0.000001;
        }   

        // A threshold for stopping (rad)
        if (!private_node_.getParam("epsilon_theta", input_.epsilon_theta))
        {
            input_.epsilon_theta = 0.000001;
        }    

        // Maximum distance for a correspondence to be valid
        if (!private_node_.getParam("max_correspondence_dist", input_.max_correspondence_dist))
        {
            input_.max_correspondence_dist = 3.0;
        }    

        // Noise in the scan (m)
        if (!private_node_.getParam("sigma", input_.sigma))
        {
            input_.sigma = 0.010;
        }    

        // Use smart tricks for finding correspondences.
        if (!private_node_.getParam("use_corr_tricks", input_.use_corr_tricks))
        {
            input_.use_corr_tricks = 1;
        }    

        // Restart: Restart if error is over threshold
        if (!private_node_.getParam("restart", input_.restart))
        {
            input_.restart = 0;
        }    

        // Restart: Threshold for restarting
        if (!private_node_.getParam("restart_threshold_mean_error", input_.restart_threshold_mean_error))
        {
            input_.restart_threshold_mean_error = 0.01;
        }    

        // Restart: displacement for restarting. (m)
        if (!private_node_.getParam("restart_dt", input_.restart_dt))
        {
            input_.restart_dt = 1.0;
        }    

        // Restart: displacement for restarting. (rad)
        if (!private_node_.getParam("restart_dtheta", input_.restart_dtheta))
        {
            input_.restart_dtheta = 0.1;
        }    

        // Max distance for staying in the same clustering
        if (!private_node_.getParam("clustering_threshold", input_.clustering_threshold))
        {
            input_.clustering_threshold = 0.25;
        }    

        // Number of neighbour rays used to estimate the orientation
        if (!private_node_.getParam("orientation_neighbourhood", input_.orientation_neighbourhood))
        {
            input_.orientation_neighbourhood = 20;
        }    

        // If 0, it's vanilla ICP
        if (!private_node_.getParam("use_point_to_line_distance", input_.use_point_to_line_distance))
        {
            input_.use_point_to_line_distance = 1;
        }    

        // Discard correspondences based on the angles
        if (!private_node_.getParam("do_alpha_test", input_.do_alpha_test))
        {
            input_.do_alpha_test = 0;
        }    

        // Discard correspondences based on the angles - threshold angle, in degrees
        if (!private_node_.getParam("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
        {
            input_.do_alpha_test_thresholdDeg = 20.0;
        }    

        // Percentage of correspondences to consider: if 0.9,
        // always discard the top 10% of correspondences with more error
        if (!private_node_.getParam("outliers_maxPerc", input_.outliers_maxPerc))
        {
            input_.outliers_maxPerc = 0.95;
        }    

        // Parameters describing a simple adaptive algorithm for discarding.
        //  1) Order the errors.
        //  2) Choose the percentile according to outliers_adaptive_order.
        //     (if it is 0.7, get the 70% percentile)
        //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
        //     with the value of the error at the chosen percentile.
        //  4) Discard correspondences over the threshold.
        //  This is useful to be conservative; yet remove the biggest errors.
        if (!private_node_.getParam("outliers_adaptive_order", input_.outliers_adaptive_order))
        {
            input_.outliers_adaptive_order = 0.7;
        }    

        if (!private_node_.getParam("outliers_adaptive_mult", input_.outliers_adaptive_mult))
        {
            input_.outliers_adaptive_mult = 2.0;
        }   

        // If you already have a guess of the solution, you can compute the polar angle
        // of the points of one scan in the new position. If the polar angle is not a monotone
        // function of the readings index, it means that the surface is not visible in the
        // next position. If it is not visible, then we don't use it for matching.
        if (!private_node_.getParam("do_visibility_test", input_.do_visibility_test))
        {
            input_.do_visibility_test = 0;
        }    

        // no two points in laser_sens can have the same corr.
        if (!private_node_.getParam("outliers_remove_doubles", input_.outliers_remove_doubles))
        {
            input_.outliers_remove_doubles = 1;
        }   

        // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
        if (!private_node_.getParam("do_compute_covariance", input_.do_compute_covariance))
        {
            input_.do_compute_covariance = 0;
        }    

        // Checks that find_correspondences_tricks gives the right answer
        if (!private_node_.getParam("debug_verify_tricks", input_.debug_verify_tricks))
        {
            input_.debug_verify_tricks = 0;
        }    

        // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
        // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
        if (!private_node_.getParam("use_ml_weights", input_.use_ml_weights))
        {
            input_.use_ml_weights = 0;
        }    
        // If 1, the field 'readings_sigma' in the second scan is used to weight the
        // correspondence by 1/sigma^2
        if (!private_node_.getParam("use_sigma_weights", input_.use_sigma_weights))
        {
            input_.use_sigma_weights = 0;
        }    
    }

    void load_pc_map()
    {
        while(pcl::io::loadPCDFile<pcl::PointXYZ>(pc_map_path, *map_pc) == -1) 
        {
            ROS_ERROR("Couldn't read file your_point_cloud.pcd");
            ros::Duration(0.5).sleep();
        }
        ROS_INFO("point cloud map is loaded successfully");
    }
    void publish_pc_map()
    {   
        sensor_msgs::PointCloud2 map_pc_msg;
        pcl::toROSMsg(*map_pc,map_pc_msg);
        map_pc_msg.header.stamp = ros::Time::now();
        map_pc_msg.header.frame_id = "map";
        map_pc_pub.publish(map_pc_msg);
        ROS_INFO("point cloud map is published successfully");
    }
    void tf2_listener_laser() 
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
            // 处理异常
            return;
        }
        
    }

    void tf2_listener_odom(ros::Time t)
    {
        try{
            //"base_link is target,odom is source"
            odom_to_base = tfBuffer.lookupTransform("base_link", "odom_frame", t);
            tf2::fromMsg(odom_to_base.transform,odom_to_base_tf2);
        }
        catch (tf2::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            // 处理异常
            return;
        }
    
        ROS_INFO("get odom_to_map");
    }

    void get_laser_pose(geometry_msgs::Pose amcl_base_on_map)
    {   
        if(!laser_to_base_received)
        {
            tf2_listener_laser();
        }

        tf2::fromMsg(amcl_base_on_map,amcl_base_to_map_tf2);
        //get laser to map transform
        laser_to_map_tf2 = amcl_base_to_map_tf2*laser_to_base_tf2;
        laser_x = laser_to_map_tf2.getOrigin().x();
        laser_y = laser_to_map_tf2.getOrigin().y();
        laser_yaw = tf2::getYaw(laser_to_map_tf2.getRotation());
        ROS_INFO("get lidar pose:  x: %f, y: %f, yaw: %f",laser_x,laser_y,laser_yaw);
    }
   
    /*
    void load_corres_scan(ros::Time t)
    {
        auto iter = scan_dic.find(t);
        if(iter != scan_dic.end())        //find successfully
        {   
            scan_data = iter->second;
            laser_angle_min = scan_data.angle_min;
            laser_angle_max = scan_data.angle_max;
            laser_angle_increment = scan_data.angle_increment;
            laser_time_increment = scan_data.time_increment;
            ranges = scan_data.ranges;
            range_max = scan_data.range_max;
            range_min = scan_data.range_min;
            scan_time_stamp = scan_data.header.stamp;
            
            //clear the scan dictionary
            auto it_end = iter++;
            scan_dic.erase(scan_dic.begin(),it_end);
            ROS_INFO("Clearing scan_dic");
        }

        else
        {
            ROS_ERROR("Can not find scan data at amcl timestamp");
        }
    }
    */
    Eigen::Matrix4f set_init_guess(float x, float y, float yaw)
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

        laser_x_grid = (laser_x-grid_origin_x)/grid_map.info.resolution;
        laser_y_grid = (laser_y-grid_origin_y)/grid_map.info.resolution;
        cout << "laser_x_grid = " << laser_x_grid << ",laser_y_grid = " << laser_y_grid << ", laser_yaw="<< laser_yaw << endl;
        float distance_virtual;
        int index = 0;
        virtual_ranges.clear();

        for(int i=0; i<ranges.size(); i++)
        {   
            int x_grid = laser_x_grid;
            int y_grid = laser_y_grid;
            float angle,x_map,y_map,m;
            bool find_vir_flag=false;
            angle = laser_yaw -(laser_angle_min + i * laser_angle_increment);
            
            if(angle>2.0*M_PI)
            {
                angle = angle - 2.0*M_PI;
            }
            if(angle<-2.0*M_PI)
            {
                angle = angle + 2.0*M_PI;
            }

            
            m = tan(angle);
            //cout << "lidar_angle" << laser_yaw << endl;
            //cout << "angle:" << angle <<"m: "<<m<< endl;
            
            //for 1＆2 Quadrant
            if((angle>=0 && angle<=M_PI) || (angle<=-M_PI && angle>=-2.0*M_PI))
            {   
                
                if(m>=0 && m<=1)
                {   
                    //ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {
                        x_grid = x_grid+1;
                        y_grid = round(line_fx(x_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_map.info.resolution/2 +grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_map.info.resolution/2 +grid_origin_y;
                            find_vir_flag = true;
                            //ROS_INFO("find virtual point! x=%f y=%f",x_map,y_map);
                            break;
                        }
                    }
                }
                else if(m>1)
                {   
                    
                    //ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {
                        y_grid = y_grid+1;
                        x_grid = round(line_fy(y_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_map.info.resolution/2 +grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_map.info.resolution/2 +grid_origin_y;
                            find_vir_flag = true;
                            //ROS_INFO("find virtual point! x=%f y=%f",x_map,y_map);
                            break;
                        }

                    }
                }
                else if(m<-1)
                {   
                    //ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {
                        y_grid = y_grid+1;
                        x_grid = round(line_fy(y_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_map.info.resolution/2 +grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_map.info.resolution/2 +grid_origin_y;
                            find_vir_flag = true;
                            //ROS_INFO("find virtual point! x=%f y=%f",x_map,y_map);
                            break;
                        }

                    }
                }
                else if(m<0 && m>=-1)
                {   
                    //ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {
                        x_grid = x_grid-1;
                        y_grid = round(line_fx(x_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_map.info.resolution/2 +grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_map.info.resolution/2 +grid_origin_y;
                            find_vir_flag = true;
                            //ROS_INFO("find virtual point! x=%f y=%f",x_map,y_map);
                            break;
                        }
                    }
                }
            }

            else
            {
                if(m>=0 && m<=1)
                {   
                    //ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {
                        x_grid = x_grid-1;
                        y_grid = round(line_fx(x_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_map.info.resolution/2 +grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_map.info.resolution/2 +grid_origin_y;
                            find_vir_flag = true;
                            //ROS_INFO("find virtual point! x=%f y=%f",x_map,y_map);
                            break;
                        }
                    }
                }
                else if(m>1)
                {   
                    //ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {
                        y_grid = y_grid-1;
                        x_grid = round(line_fy(y_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_map.info.resolution/2 +grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_map.info.resolution/2 +grid_origin_y;
                            find_vir_flag = true;
                            //ROS_INFO("find virtual point! x=%f y=%f",x_map,y_map);
                            break;
                        }
                    }
                }
                else if(m<-1)
                {   
                    //ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {   
                        
                        y_grid = y_grid-1;
                        x_grid = round(line_fy(y_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_map.info.resolution/2 +grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_map.info.resolution/2 +grid_origin_y;
                            find_vir_flag = true;
                            //ROS_INFO("find virtual point! x=%f y=%f",x_map,y_map);
                            break;
                        }
                    }
                }
                else if(m<0 && m>=-1)
                {   
                    //ROS_INFO("start find virtual point ");
                    while(x_grid>=0 && x_grid < grid_width && y_grid>=0 && y_grid < grid_height) 
                    {
                        x_grid = x_grid+1;
                        y_grid = round(line_fx(x_grid, m));
                        index =x_grid+y_grid*grid_width;
                        if(grid_map.data[index]>=occ_threshold)
                        {   
                            //virtual point at map frame
                            x_map = x_grid * grid_map.info.resolution + grid_map.info.resolution/2 + grid_origin_x;
                            y_map = y_grid * grid_map.info.resolution + grid_map.info.resolution/2 + grid_origin_y;
                            find_vir_flag = true;
                            //ROS_INFO("find virtual point! x=%f y=%f",x_map,y_map);
                            break;
                        }
                    }
                }
            }

            //change into point cloud in map frame
            if(find_vir_flag)
            {   
                //store virtual point
                point.x = x_map;
                point.y = y_map;
                point.z = 0.0;  
                pc->push_back(point);

                //store as ranges(distance between virtual point and lidar)
                distance_virtual = sqrt( pow((x_map - laser_x),2) + pow((y_map - laser_y),2));
                virtual_ranges.push_back(distance_virtual);
            }

            else
            {   
                //store virtual point NAN
                point.x = NAN;
                point.y = NAN;
                point.z = 0.0;
                pc->push_back(point);
                //store as ranges(distance between virtual point and lidar) NAN
                distance_virtual = NAN;
                virtual_ranges.push_back(NAN);
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

    void laserScanToLDP(vector<float>scan_ranges, LDP& ldp)                                     
    {   
        double angle;
        unsigned int n = scan_ranges.size();
        ldp = ld_alloc_new(n);

        for (unsigned int i = 0; i < n; i++)
        {
            // calculate position in laser frame

            double r = scan_ranges[i];

            if (r > range_min && r < range_max)
            {
                // fill in laser scan data

                ldp->valid[i] = 1;
                ldp->readings[i] = r;
                //cout << "ldp->readings: " << ldp->readings[i] << endl;
            }
            else
            {
                ldp->valid[i] = 0;
                ldp->readings[i] = -1;  // for invalid range
            }
            angle = laser_angle_min + i * laser_angle_increment;
            ldp->theta[i] = angle;

            ldp->cluster[i]  = -1;
        }

        ldp->min_theta = laser_angle_min;
        ldp->max_theta = laser_angle_max;

        ldp->odometry[0] = 0.0;
        ldp->odometry[1] = 0.0;
        ldp->odometry[2] = 0.0;

        ldp->true_pose[0] = 0.0;
        ldp->true_pose[1] = 0.0;
        ldp->true_pose[2] = 0.0;
    }
  
    tf2::Transform createTf_from_XYyaw(double x, double y, double theta)
    {   
        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3(x, y, 0.0));
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta);
        transform.setRotation(q);
        return transform;
    }

    void tf_correct_odom()
    {   
        tf2::Transform new_laser_to_old_laser;
        
        
        tf2::Transform odom_to_map;
        //get the new laser to map transform
        new_laser_to_old_laser = createTf_from_XYyaw(laser_x_plicp, laser_y_plicp, laser_yaw_plicp);
        new_laser_to_map = laser_to_map_tf2*new_laser_to_old_laser;

        //find new base_link to map transform
        base_to_laser_tf2 = laser_to_base_tf2.inverse();
        new_base_to_map = new_laser_to_map*base_to_laser_tf2;
        PLICP_pose_x = new_base_to_map.getOrigin().x();
        PLICP_pose_y = new_base_to_map.getOrigin().y();
        tf2::Quaternion q;
        q = new_base_to_map.getRotation();
        PLICP_pose_yaw = tf2::getYaw(q);
        ROS_INFO("PLICP Pose: x=%f, y=%f, theta=%f",PLICP_pose_x,PLICP_pose_y,PLICP_pose_yaw);
        
        /*
        //get new odom to map and broadcast it
        tf2::Transform tf2_odom_to_map;
        tf2_odom_to_map = new_base_to_map*odom_to_base_tf2;

        geometry_msgs::TransformStamped odom_to_map_tf_stamped;
        odom_to_map_tf_stamped.header.frame_id = "map";
        odom_to_map_tf_stamped.header.stamp = ros::Time::now();
        odom_to_map_tf_stamped.child_frame_id = "odom_frame";
        tf2::convert(tf2_odom_to_map, odom_to_map_tf_stamped.transform);
        br.sendTransform(odom_to_map_tf_stamped);
        ROS_INFO("odom_to_map has been corrected\n");
        */
    }

    void do_PLICP()
    {   
        LDP true_scan_ldp;
        LDP virtual_scan_ldp;
        vector<float> ranges_inside_error;
        vector<float> virtual_ranges_inside_error;

        //need to change the scan into the csm PLICP data type LDP
        laserScanToLDP(ranges,true_scan_ldp);  
        laserScanToLDP(virtual_ranges,virtual_scan_ldp); 
        // CSM is used in the following way:
        // The scans are always in the laser frame
        // The reference scan (prevLDPcan_,true_scan_ldp) has a pose of [0, 0, 0]
        // The new scan (currLDPScan,virtual_scan_ldp) has a pose equal to the movement
        // of the laser in the laser frame since the last scan
        // The computed correction is then propagated using the tf machinery

        true_scan_ldp->odometry[0] = 0.0;
        true_scan_ldp->odometry[1] = 0.0;
        true_scan_ldp->odometry[2] = 0.0;

        true_scan_ldp->estimate[0] = 0.0;
        true_scan_ldp->estimate[1] = 0.0;
        true_scan_ldp->estimate[2] = 0.0;

        true_scan_ldp->true_pose[0] = 0.0;
        true_scan_ldp->true_pose[1] = 0.0;
        true_scan_ldp->true_pose[2] = 0.0;

        //next scan pose prediction respect to the last laser frame  = 0 ,which means do not predict
        input_.first_guess[0] = 0;
        input_.first_guess[1] = 0;
        input_.first_guess[2] = 0;
        input_.min_reading = range_min;
        input_.max_reading = range_max;
        input_.laser_ref = virtual_scan_ldp;
        input_.laser_sens = true_scan_ldp;

        ROS_INFO("scan has change into LDP, starting PLICP\n");
        //do PLICP
        sm_icp(&input_, &output_);

        //new laser pose on the old laser frame
        if (output_.valid)
        {   
            ROS_INFO("PLICP valid\n");
            laser_x_plicp = output_.x[0];
            laser_y_plicp = output_.x[1];
            laser_yaw_plicp = output_.x[2];
            //ROS_WARN("laser_x_plicp: %f, laser_y_plicp: %f, laser_yaw_plicp: %f", laser_x_plicp, laser_y_plicp, laser_yaw_plicp);
            //use the new laser pose to correct tf tree relationships
            tf_correct_odom();

        }
        else
        {
             ROS_WARN("PLICP is not Converged");
        }

        ld_free(true_scan_ldp);
        ld_free(virtual_scan_ldp);

    }

    void amcl_reinit()
    {   
        //load the pose of PLICP as new initail pose of AMCL
        geometry_msgs::PoseWithCovarianceStamped init_pose;
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, PLICP_pose_yaw);
        myQuaternion.normalize();

        init_pose.header.stamp = ros::Time::now();
        init_pose.header.frame_id = "map";

        init_pose.pose.pose.position.x = PLICP_pose_x;
        init_pose.pose.pose.position.y = PLICP_pose_y;
        init_pose.pose.pose.position.z = 0;

        init_pose.pose.pose.orientation.x = myQuaternion.getX();
        init_pose.pose.pose.orientation.y = myQuaternion.getY();
        init_pose.pose.pose.orientation.z = myQuaternion.getZ();
        init_pose.pose.pose.orientation.w = myQuaternion.getW();

        //initial covariance
        for(int i=0; i<36; i++)
        {
            init_pose.pose.covariance[i] = 0;
        }
        //set xx yy yawyaw covariance
        init_pose.pose.covariance[0] = cov_xx;
        init_pose.pose.covariance[6*1+1] = cov_yy;
        init_pose.pose.covariance[6*5+5] = cov_yawyaw;
        //publish new pose to reinitialize the AMCL
        amcl_init_pub.publish(init_pose);
        ROS_INFO("AMCL re-initialization complete");
    }

    void amcl_path_publisher(geometry_msgs::PoseWithCovarianceStamped::ConstPtr poseMsg)
    {
        geometry_msgs::PoseStamped pose;
        tf2::Quaternion myQuaternion;
        
        pose.pose = poseMsg->pose.pose;
        pose.header.stamp = poseMsg->header.stamp;
        pose.header.frame_id = poseMsg->header.frame_id;
        
        amcl_path.header.frame_id = poseMsg->header.frame_id;
        amcl_path.header.stamp = poseMsg->header.stamp;
        amcl_path.poses.push_back(pose);
        amcl_path_pub.publish(amcl_path);
    }

    void PLICP_pose_path_publisher()
    {
        geometry_msgs::PoseStamped pose;
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, PLICP_pose_yaw);
        myQuaternion.normalize();

        pose.header.stamp = scan_time_stamp;
        pose.header.frame_id = "map";

        pose.pose.position.x = PLICP_pose_x;
        pose.pose.position.y = PLICP_pose_y;
        pose.pose.position.z = 0;

        pose.pose.orientation.x = myQuaternion.getX();
        pose.pose.orientation.y = myQuaternion.getY();
        pose.pose.orientation.z = myQuaternion.getZ();
        pose.pose.orientation.w = myQuaternion.getW();
        //pub pose after PLICP
        PLICP_pose_pub.publish(pose);
        ROS_WARN("PLICP pub time: %f",ros::Time::now().toSec());

        //store the pub time between 2 PLICP poses
        if(!scan_matching_init)
        {   
            last_scan_matching_time = ros::WallTime::now();
            scan_matching_init = true;
        }
        else
        {   
            scan_matching_pub_now = ros::WallTime::now();
            scan_matching_pub_duration = scan_matching_pub_now- last_scan_matching_time;
            //store the time data
            file_scan_matching_pose_time << last_scan_matching_time.toSec() << "," << scan_matching_pub_now.toSec() << "," << scan_matching_pub_duration.toSec() << "\n";
            last_scan_matching_time = scan_matching_pub_now;
        }
        
        //pub path of PLICP poses
        PLICP_path.header.frame_id = "map";
        PLICP_path.header.stamp = scan_time_stamp;
        PLICP_path.poses.push_back(pose);
        PLICP_path_pub.publish(PLICP_path);
    }
    void ICP_pose_path_publisher(Eigen::Matrix4f transform)
    {   
        Eigen::Isometry3d T;
        Eigen::Matrix4d transform_double;
        geometry_msgs::PoseStamped pose;
        transform_double = transform.cast<double>();
        pose.header.stamp = scan_time_stamp;
        pose.header.frame_id = "map";

        //pub icp pose
        T = transform_double;
        pose.pose = tf2::toMsg(T);
        ICP_pose_pub.publish(pose);

        //store the pub time between 2 PLICP poses
        if(!scan_matching_init)
        {   
            last_scan_matching_time = ros::WallTime::now();
            scan_matching_init = true;
        }
        else
        {   
            scan_matching_pub_now = ros::WallTime::now();
            scan_matching_pub_duration = scan_matching_pub_now- last_scan_matching_time;
            //store the time data
            file_scan_matching_pose_time << last_scan_matching_time.toSec() << "," << scan_matching_pub_now.toSec() << "," << scan_matching_pub_duration.toSec() << "\n";
            last_scan_matching_time = scan_matching_pub_now;
        }
        //pub icp path
        ICP_path.header.frame_id = "map";
        ICP_path.header.stamp = scan_time_stamp;
        ICP_path.poses.push_back(pose);
        ICP_path_pub.publish(ICP_path);
    }

    void aligned_pc_publish(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
    {
        sensor_msgs::PointCloud2 pc_msg;
        pcl::toROSMsg(*pc,pc_msg);
        pc_msg.header.stamp = scan_time_stamp;
        pc_msg.header.frame_id = "map";
        aligned_pc_pub.publish(pc_msg);
        ROS_INFO("aligned point cloud is published successfully");
    }


    void amclsyncCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg)
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
        
        //get amcl time and count duration between 2 amcl pose received
        if(!amcl_init)
        {   
            last_amcl_time = ros::WallTime::now();
            amcl_receive_now = last_amcl_time;
            amcl_init = true;
        }
        else
        {
            amcl_receive_now = ros::WallTime::now();
            amcl_pub_duration = amcl_receive_now - last_amcl_time;
            //store the time data
            file_amcl_time << last_amcl_time.toSec() << "," << amcl_receive_now.toSec() << "," << amcl_pub_duration.toSec() << "\n";
            last_amcl_time = amcl_receive_now;
        }
        

        scan_matching_start_time = amcl_receive_now;
        ROS_WARN("scan_matching start time : %f ", scan_matching_start_time.toSec());
        //get the amcl global pose
        amcl_pose_x = poseMsg->pose.pose.position.x;
        amcl_pose_y = poseMsg->pose.pose.position.y;
        amcl_pose_yaw = tf::getYaw(poseMsg->pose.pose.orientation);
        amcl_time_stamp = poseMsg->header.stamp;
        ROS_INFO("Received amcl Pose: x=%f, y=%f, theta=%f\n",amcl_pose_x,amcl_pose_y,amcl_pose_yaw);
        ROS_INFO("amcl timestamp: %f\n",amcl_time_stamp.toSec());
        ROS_INFO("Received amcl time: %f\n",ros::Time::now().toSec());
        
        //get grid map use mapserver service
        ros::NodeHandle nh;
        ros::ServiceClient map_client = nh.serviceClient<nav_msgs::GetMap>("static_map");

        nav_msgs::GetMap mapsrv;
        if (!map_received && map_client.call(mapsrv))
        {
            //store the grid map
            grid_map = mapsrv.response.map;
            grid_origin_x = grid_map.info.origin.position.x;
            grid_origin_y = grid_map.info.origin.position.y;
            grid_height = grid_map.info.height;
            grid_width = grid_map.info.width;
            ROS_INFO("Received Map: width=%d, height=%d, resolution=%.3f\n", grid_map.info.width, grid_map.info.height, grid_map.info.resolution);
            ROS_INFO("Received Map: origin_x=%.3f, origin_y=%.3f\n", grid_origin_x, grid_origin_y);
            map_received = true;

        }
        else if(!map_received && !map_client.call(mapsrv))
        {
            ROS_ERROR("Failed to call static_map service");
        }

        //load the scan corresponding to the amcl_pose timestamp
        //load_corres_scan(amcl_time_stamp);
        ROS_INFO("Scan timestamp: %f\n",scan_time_stamp.toSec());

        if(scan_time_stamp!=amcl_time_stamp)
        {
            ROS_WARN("The scan timestamp is different from the amcl timestamp");
        }
        //create the scan point cloud
        scan_pc = create_scan_pc();
        ROS_INFO("Received scan_pc");
        sensor_msgs::PointCloud2 scan_pc_msg;
        pcl::toROSMsg(*scan_pc, scan_pc_msg);
        scan_pc_msg.header.stamp = scan_time_stamp;
        scan_pc_msg.header.frame_id = "laser";
        scan_pc_pub.publish(scan_pc_msg);
        ROS_INFO("scan_pc published");
        //get the laser to base_link tf
        get_laser_pose(poseMsg->pose.pose);

        switch(method)
        {
            case ICP:
            {
                //load pointcloud map
                if(!pc_map_received)
                {
                    load_pc_map();
                    pc_map_received = true;
                }
                //publish point cloud map
                publish_pc_map();
                ROS_INFO("pc_map published");
                //transform scan points from laser to base_link
                pcl::transformPointCloud(*scan_pc,*scan_pc_at_base,laser_to_base_eigen.matrix());
                for (size_t i=0; i<scan_pc_at_base->size(); i++) 
                {
                    scan_pc_at_base->points[i].z = 0;
                }
                //transform amcl pose to eigen matrix as initial guess
                init_guess = set_init_guess(amcl_pose_x, amcl_pose_y, amcl_pose_yaw);

                // Create an ICP object
                pcl::PointCloud<pcl::PointXYZ>::Ptr output_pc(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                icp.setMaxCorrespondenceDistance (0.15);
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
                    ROS_INFO("ICP converged, fitness score: %lf\n", icp.getFitnessScore());
                    icp_result_transform = icp.getFinalTransformation();
                    ICP_pose_path_publisher(icp_result_transform);
                    aligned_pc_publish(output_pc);
                }
                else
                {
                    ROS_WARN("ICP not converged");
                }

                break;
            }
            case GICP:
            {
                //load pointcloud map
                if(!pc_map_received)
                {
                    load_pc_map();
                    pc_map_received = true;
                }
                //publish point cloud map
                publish_pc_map();
                ROS_INFO("pc_map published");
                //transform scan points from laser to base_link
                pcl::transformPointCloud(*scan_pc,*scan_pc_at_base,laser_to_base_eigen.matrix());
                for (size_t i=0; i<scan_pc_at_base->size(); i++) 
                {
                    scan_pc_at_base->points[i].z = 0;
                }
                //transform amcl pose to eigen matrix as initial guess
                init_guess = set_init_guess(amcl_pose_x, amcl_pose_y, amcl_pose_yaw);

                // Create an GICP object
                pcl::PointCloud<pcl::PointXYZ>::Ptr output_pc(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;;
                gicp.setMaxCorrespondenceDistance (0.15);
                gicp.setMaximumIterations (50);
                gicp.setTransformationEpsilon (1e-5);
                gicp.setEuclideanFitnessEpsilon(1e-5);
                //set source pc as radar point cloud, and target pc as map point cloud
                gicp.setInputSource(scan_pc_at_base);
                gicp.setInputTarget(map_pc);
                //run the GICP, then get the transformation matrix after icp as new base_link
                gicp.align(*output_pc, init_guess);

                if(gicp.hasConverged())
                {   
                    ROS_INFO("GICP converged, fitness score: %lf\n", gicp.getFitnessScore());
                    icp_result_transform = gicp.getFinalTransformation();
                    ICP_pose_path_publisher(icp_result_transform); 
                    aligned_pc_publish(output_pc);
                }
                else
                {
                    ROS_WARN("GICP not converged");
                }

                break;
            }

            case NDT:
            {   
                //load pointcloud map
                if(!pc_map_received)
                {
                    load_pc_map();
                    pc_map_received = true;
                }
                //publish point cloud map
                publish_pc_map();
                ROS_INFO("pc_map published");
                //transform scan points from laser to base_link
                pcl::transformPointCloud(*scan_pc,*scan_pc_at_base,laser_to_base_eigen.matrix());
                for (size_t i=0; i<scan_pc_at_base->size(); i++) 
                {
                    scan_pc_at_base->points[i].z = 0;
                }
                //transform amcl pose to eigen matrix as initial guess
                init_guess = set_init_guess(amcl_pose_x, amcl_pose_y, amcl_pose_yaw);

                // Create an NDT object
                pcl::PointCloud<pcl::PointXYZ>::Ptr output_pc(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
                ndt.setMaximumIterations (100);
                ndt.setTransformationEpsilon (0.001);
                ndt.setStepSize (0.1);
                ndt.setResolution (0.8);

                //set source pc as radar point cloud, and target pc as map point cloud
                ndt.setInputSource(scan_pc_at_base);
                ndt.setInputTarget(map_pc);
                //run the NDT, then get the transformation matrix after icp as new base_link
                ndt.align(*output_pc, init_guess);

                if(ndt.hasConverged())
                {   
                    ROS_INFO("NDT converged, fitness score: %lf\n", ndt.getFitnessScore());
                    icp_result_transform = ndt.getFinalTransformation();
                    ICP_pose_path_publisher(icp_result_transform); 
                    aligned_pc_publish(output_pc);
                }
                else
                {
                    ROS_WARN("NDT not converged");
                }


                break;
            }

            case PLICP:
            {
                //create the vitual point cloud
                tf2_listener_odom(amcl_time_stamp);
                virtual_pc = create_vitual_scan_pc();
                ROS_INFO("Received virtual_pc");
                sensor_msgs::PointCloud2 virtual_pc_msg;
                pcl::toROSMsg(*virtual_pc, virtual_pc_msg);
                virtual_pc_msg.header.stamp = scan_time_stamp;
                virtual_pc_msg.header.frame_id = "map";
                virtual_pc_pub.publish(virtual_pc_msg);
                ROS_INFO("virtual_pc published");
                
                //create the virtual scan
                virtual_scan.header.stamp = scan_time_stamp;
                virtual_scan.header.frame_id = "laser";
                virtual_scan.angle_min = laser_angle_min;
                virtual_scan.angle_max = laser_angle_max;
                virtual_scan.angle_increment = laser_angle_increment;
                virtual_scan.time_increment = laser_time_increment;
                virtual_scan.range_min = range_min;
                virtual_scan.range_max = range_max;
                virtual_scan.ranges = virtual_ranges;
                virtual_scan_pub.publish(virtual_scan);
                ROS_INFO("virtual_scan published");
                
                //do PLICP to correct the pose of AMCL
                do_PLICP();
                //get aligned scan after PLICP pose
                pcl::PointCloud<pcl::PointXYZ>::Ptr PLICP_aligned_pc(new pcl::PointCloud<pcl::PointXYZ>);
                new_laser_to_map_msgs = tf2::toMsg(new_laser_to_map);
                new_laser_to_map_eigen = tf2::transformToEigen(new_laser_to_map_msgs);
                pcl::transformPointCloud(*scan_pc,*PLICP_aligned_pc,new_laser_to_map_eigen.matrix());
                aligned_pc_publish(PLICP_aligned_pc);
                //draw the path of AMCL+PLICP and origin AMCL
                PLICP_pose_path_publisher();
                
                break;
            }
                
        }
        
        amcl_path_publisher(poseMsg);
        scan_matching_end_time = ros::WallTime::now();
        scan_matching_time_used =  scan_matching_end_time - scan_matching_start_time;
        file_scan_matching_time << scan_matching_start_time.toSec() << "," << scan_matching_end_time.toSec() << "," << scan_matching_time_used.toSec() << "\n";
        ROS_INFO("scan_matching_time used : %f (s)\n", scan_matching_time_used.toSec());
        
        /*
        //reinitialize the AMCL
        re_init_count++;
        if(re_init_count >20)
        {
            amcl_reinit();
            re_init_count = 0;
        }
        
        */

    }
};


int main(int argc, char** argv) 
{
    ros::init (argc, argv, "scan");
    ros::NodeHandle nh("~");
    ICP_with_AMCL ICP_with_AMCL(nh);

    ros::spin();
    return 0;
}
