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
#include <tf2/utils.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>

#include <csm/csm_all.h> 
using namespace CSM ;
using namespace std;

class scan_to_pc
{
private:
    double amcl_pose_x;
    double amcl_pose_y;
    double amcl_pose_yaw; 
    double PLICP_pose_x;
    double PLICP_pose_y;
    double PLICP_pose_yaw; 
    nav_msgs::Path amcl_path;
    nav_msgs::Path PLICP_path;


    bool amcl_init = false;
    bool PLICP_init = false;
    ros::Time PLICP_start_time;
    ros::Time amcl_receive_now;
    ros::Time last_amcl_time;
    ros::Time PLICP_end_time;
    ros::Time last_PLICP_time;
    ros::Time PLICP_pub_now;
    ros::Duration PLICP_time_used; 
    ros::Duration amcl_pub_duration;
    ros::Duration PLICP_pub_duration; 
   

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc;
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


    tf::TransformListener listener;
    //tf::TransformBroadcaster br;
    tf2_ros::TransformBroadcaster br;
    tf::StampedTransform laser_to_map_transform;
    tf::StampedTransform odom_to_base_link;
    tf::StampedTransform base_link_to_laser;

    std::string amcl_time_save_path;
    std::string ICP_time_save_path;
    std::string PLICP_pose_time_save_path;
    std::ofstream file_amcl_time;
    std::ofstream file_icp_time;
    std::ofstream file_PLICP_pose_time;

public:
    scan_to_pc(ros::NodeHandle nh)
    {   
        _nh = nh;
        InitParams();
        laser_scan_sub = _nh.subscribe("/scan", 100, &scan_to_pc::laserScanCallback,this);
        //subscibe amcl pose
        poseSub = _nh.subscribe("/amcl_pose", 1, &scan_to_pc::amclposeCallback,this);

        scan_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/scan_pc", 10);
        virtual_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/virtual_pc", 10);
        virtual_scan_pub = _nh.advertise<sensor_msgs::LaserScan >("/virtual_scan", 10);
        amcl_path_pub = _nh.advertise<nav_msgs::Path>("/amcl_path", 1);
        PLICP_path_pub = _nh.advertise<nav_msgs::Path>("/PLICP_path", 1);
        PLICP_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("/PLICP_pose", 1);

        _nh.param<string>("amcl_time_save_path", amcl_time_save_path,"/Default/path");
        _nh.param<string>("ICP_time_save_path", ICP_time_save_path,"/Default/path");
        _nh.param<string>("PLICP_pose_time_save_path", PLICP_pose_time_save_path,"/Default/path");
        file_amcl_time.open(amcl_time_save_path);
        file_amcl_time << "last time, this time, pub duration\n";
        file_icp_time.open(ICP_time_save_path);
        file_icp_time << "start time, end time, time duration\n";
        file_PLICP_pose_time.open(PLICP_pose_time_save_path);
        file_PLICP_pose_time << "last time, this time, pub duration\n";
    }
    
    ~scan_to_pc()
    {
        ROS_WARN("Exit Localization");
        file_amcl_time.close();
        file_icp_time.close();
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
    void tf_listener_laser()
    {
        try{
            //"map is target,laser is source"
            listener.lookupTransform("map", "laser", ros::Time(0),laser_to_map_transform);
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            // 处理异常
            return;
        }
        
        laser_x = laser_to_map_transform.getOrigin().x(),
        laser_y = laser_to_map_transform.getOrigin().y(),
        laser_yaw = tf::getYaw(laser_to_map_transform.getRotation());
        ROS_INFO("get lidar pose:  x: %f, y: %f, yaw: %f",laser_x,laser_y,laser_yaw);
    }
    void tf_listener_odom()
    {
        try{
            //"base_link is target,odom is source"
            listener.lookupTransform("base_link", "odom_frame", ros::Time(0),odom_to_base_link);
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            // 处理异常
            return;
        }
    
        ROS_INFO("get odom_to_map");
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
            cout << "lidar_angle" << laser_yaw << endl;
            cout << "angle:" << angle <<"m: "<<m<< endl;
            //for 1＆2 Quadrant
            if((angle>=0 && angle<=M_PI) || (angle<=-M_PI && angle>=-2.0*M_PI))
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
                            ROS_INFO("find virtual point! x=%f y=%f",x_map,y_map);
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
                            ROS_INFO("find virtual point! x=%f y=%f",x_map,y_map);
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
                        y_grid = y_grid-1;
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
                            ROS_INFO("find virtual point! x=%f y=%f",x_map,y_map);
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
  
    tf::Transform createTf_from_XYyaw(double x, double y, double theta)
    {   
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, theta);
        transform.setRotation(q);
        return transform;
    }

    void tf_correct_odom()
    {   
        tf::Transform new_laser_to_old_laser;
        tf::Transform new_laser_to_map;
        tf::Transform new_base_to_map;
        tf::Transform odom_to_map;
        //get the new laser to map transform
        new_laser_to_old_laser = createTf_from_XYyaw(laser_x_plicp, laser_y_plicp, laser_yaw_plicp);
        new_laser_to_map = laser_to_map_transform*new_laser_to_old_laser;

        //get base_link to laser transform
        try{
            //map is target,base_link is source
            listener.lookupTransform("laser", "base_link", ros::Time(0),base_link_to_laser);
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            // error
            return;
        }
        //find new base_link to map transform
        new_base_to_map = new_laser_to_map*base_link_to_laser;
        PLICP_pose_x = new_base_to_map.getOrigin().x();
        PLICP_pose_y = new_base_to_map.getOrigin().y();
        tf::Quaternion q;
        q = new_base_to_map.getRotation();
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        PLICP_pose_yaw = yaw;
        ROS_INFO("PLICP Pose: x=%f, y=%f, theta=%f",PLICP_pose_x,PLICP_pose_y,PLICP_pose_yaw);

        //get new odom to map and broadcast it
        tf_listener_odom();
        odom_to_map = new_base_to_map*odom_to_base_link;
        tf2::Transform tf2_odom_to_map;
        geometry_msgs::Pose odom_pose_in_map;
        tf::poseTFToMsg(odom_to_map, odom_pose_in_map);
        tf2::convert(odom_pose_in_map, tf2_odom_to_map);

        geometry_msgs::TransformStamped odom_to_map_tf_stamped;
        odom_to_map_tf_stamped.header.frame_id = "map";
        odom_to_map_tf_stamped.header.stamp = ros::Time::now();
        odom_to_map_tf_stamped.child_frame_id = "odom_frame";
        tf2::convert(tf2_odom_to_map, odom_to_map_tf_stamped.transform);
        br.sendTransform(odom_to_map_tf_stamped);
        ROS_INFO("odom_to_map has been corrected\n");
    }

    void PLICP()
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
        input_.laser_ref = true_scan_ldp;
        input_.laser_sens = virtual_scan_ldp;

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

        pose.header.stamp = ros::Time::now();
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

        //store the pub time between 2 PLICP poses
        if(!PLICP_init)
        {   
            last_PLICP_time = ros::Time::now();
            PLICP_init = true;
        }
        else
        {   
            PLICP_pub_now = ros::Time::now();
            PLICP_pub_duration = PLICP_pub_now- last_PLICP_time;
            //store the time data
            file_PLICP_pose_time << last_PLICP_time.toSec() << "," << PLICP_pub_now.toSec() << "," << PLICP_pub_duration.toSec() << "\n";
            last_PLICP_time = PLICP_pub_now;
        }
        
        //pub path of PLICP poses
        PLICP_path.header.frame_id = "map";
        PLICP_path.header.stamp = ros::Time::now();
        PLICP_path.poses.push_back(pose);
        PLICP_path_pub.publish(PLICP_path);
    }


    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
    {
        laser_angle_min = laser_scan->angle_min;
        laser_angle_max = laser_scan->angle_max;
        laser_angle_increment = laser_scan->angle_increment;
        laser_time_increment = laser_scan->time_increment;
        ("Received Laser Scan with angle min: %f, angle max: %f", laser_scan->angle_min, laser_scan->angle_max);
        ranges = laser_scan->ranges;
        range_max = laser_scan->range_max;
        range_min = laser_scan->range_min;

        //cout << "Ranges size: " << laser_scan->ranges.size()<<endl;
        //ROS_INFO("Received Laser Scan ranges size=%d", ranges.size());
    }

    void amclposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg)
    {
        //get amcl time and count duration between 2 amcl pose received
        if(!amcl_init)
        {   
            last_amcl_time = ros::Time::now();
            amcl_init = true;
        }
        else
        {
            amcl_receive_now = ros::Time::now();
            amcl_pub_duration = amcl_receive_now - last_amcl_time;
            //store the time data
            file_amcl_time << last_amcl_time.toSec() << "," << amcl_receive_now.toSec() << "," << amcl_pub_duration.toSec() << "\n";
            last_amcl_time = amcl_receive_now;
        }
        

        PLICP_start_time = amcl_receive_now;
        //get the amcl global pose
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
            //store the grid map
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

        //create the scan point cloud
        scan_pc = create_scan_pc();
        ROS_INFO("Received scan_pc");
        sensor_msgs::PointCloud2 scan_pc_msg;
        pcl::toROSMsg(*scan_pc, scan_pc_msg);
        scan_pc_msg.header.stamp = ros::Time::now();
        scan_pc_msg.header.frame_id = "laser";
        scan_pc_pub.publish(scan_pc_msg);
        ROS_INFO("scan_pc published");

        //create the vitual point cloud
        tf_listener_laser();

        virtual_pc = create_vitual_scan_pc();
        ROS_INFO("Received virtual_pc");
        sensor_msgs::PointCloud2 virtual_pc_msg;
        pcl::toROSMsg(*virtual_pc, virtual_pc_msg);
        virtual_pc_msg.header.stamp = ros::Time::now();
        virtual_pc_msg.header.frame_id = "map";
        virtual_pc_pub.publish(virtual_pc_msg);
        ROS_INFO("virtual_pc published");
        
        //create the virtual scan
        virtual_scan.header.stamp = ros::Time::now();
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
        PLICP();
        PLICP_end_time = ros::Time::now();
        PLICP_time_used =  PLICP_end_time - PLICP_start_time;
        file_icp_time << PLICP_start_time.toSec() << "," << PLICP_end_time.toSec() << "," << PLICP_time_used.toSec() << "\n";
        ROS_INFO("PLICP time used : %f (s)\n", PLICP_time_used.toSec());
        
        //draw the path of AMCL+PLICP and origin AMCL
        PLICP_pose_path_publisher();
        amcl_path_publisher(poseMsg);
        
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
