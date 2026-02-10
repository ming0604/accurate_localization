#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include <sys/shm.h>
#include <sys/sem.h>

double       *shmPtrros = NULL;
int           shmidros;
int           semidros;
key_t         shmKeyros = (key_t)888;
key_t         semKeyros = (key_t)999;

struct sembuf sop; // for P() V() operation
long          page_size = sysconf(_SC_PAGESIZE);

int P(int s);
int V(int s);

using namespace std;

class Pose100Hz
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        //publish pose at 100HZ from tf tree
        ros::Publisher fixed_freq_pose_pub_;
        ros::Timer pose_timer_;
        //IMU data subscriber
        ros::Subscriber imu_sub_;
        double imu_data_[3];

        geometry_msgs::TransformStamped transformStamped;

        string global_frame_id_;
        string base_frame_id_;
        bool use_shm_;


        void initSemShm();
        //callback function
        void poseReceived(const ros::TimerEvent& event);
        void imuReceived(const sensor_msgs::ImuConstPtr& msg);
        
    public:
        Pose100Hz(): tfListener(tfBuffer), private_nh_("~")
        {   
            
            // get parameters from launch file
            private_nh_.param<string>("global_frame_id", global_frame_id_, "map");
            private_nh_.param<string>("base_frame_id", base_frame_id_, "base_link");
            private_nh_.param("use_shared_memory", use_shm_, true);

            // initialize shared memory and semaphore(if needed)
            if(use_shm_)
            {
                initSemShm();
            }
                
            imu_sub_ = nh_.subscribe("t265/imu", 2, &Pose100Hz::imuReceived, this);
            // publisher and timer for 100Hz take pose from tf tree
            fixed_freq_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("freq_pose", 2, true);
            pose_timer_ = nh_.createTimer(ros::Duration(0.01), &Pose100Hz::poseReceived, this);
        }
        ~Pose100Hz()
        {
            //TODO: release shared memory and semaphore(if needed)
        }

}

void Pose100Hz::initSemShm()
{
    // ======================== shm init ========================
    if((shmidros = shmget(shmKeyros, page_size, IPC_CREAT | 0666)) == -1){
        printf("shmget error\n");
        exit(1);
    }
    if((shmPtrros = (double*)shmat(shmidros, NULL, 0)) == (double*)-1){
        printf("shmat error\n");
        exit(1);
    }

    // ======================== sem init ========================
    if((semidros = semget(semKeyros, 1, IPC_CREAT | 0666)) == -1){
        printf("semget error\n");
        exit(1);
    }
    //TODO: initialize shared memory and semaphore
    V(semidros);
}

void Pose100Hz::imuReceived(const sensor_msgs::ImuConstPtr& msg)
{
  imu_data_[0] = msg->angular_velocity.x;
  imu_data_[1] = msg->angular_velocity.y;
  imu_data_[2] = msg->angular_velocity.z;
}

void Pose100Hz::poseReceived(const ros::TimerEvent& event)
{
  geometry_msgs::PoseWithCovarianceStamped ffp;

  // ROS_INFO("FFP");
  ffp.header.frame_id = global_frame_id_;
  ffp.header.stamp = ros::Time::now(); 
    
  try{
    transformStamped = tfBuffer.lookupTransform(global_frame_id_, base_frame_id_, ros::Time(0));
    ffp.pose.pose.position.x = transformStamped.transform.translation.x;
    ffp.pose.pose.position.y = transformStamped.transform.translation.y;
    ffp.pose.pose.orientation = transformStamped.transform.rotation;

    P(semidros);
    // amclPtr =shmPtr_ros;
    shmPtrros[0] = ffp.header.stamp.toNSec();
    shmPtrros[1] = ffp.pose.pose.position.x;
    shmPtrros[2] = ffp.pose.pose.position.y;
    shmPtrros[3] = tf2::getYaw(ffp.pose.pose.orientation);
    // shmPtrros[4] = (ffp.pose.pose.position.x - lp.pose.pose.position.x)/0.01;
    // shmPtrros[5] = 0;
    // shmPtrros[6] = 0;
    shmPtrros[7] = imu_data_[1];
    V(semidros);

    // ffp.pose.covariance[0] = 0.025;
    // ffp.pose.covariance[6] = 0.025;
    // ffp.pose.covariance[35] = 0.001;

    fixed_freq_pose_pub_.publish(ffp);
    // lp.pose.pose.position.x = ffp.pose.pose.position.x;
    // lp.pose.pose.position.y = ffp.pose.pose.position.y;
  }
  catch(tf2::TransformException &ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_100Hz");
    Pose100Hz pose100Hz;

    ros::spin();
    return 0;
}

//========================================================================
//                          semaphore operations 
//========================================================================
int P(int s){ // aquire (try to decrease)
	sop.sem_num = 0;
	sop.sem_op = -1;
	sop.sem_flg = 0;

	if(semop(s, &sop, 1) < 0){
		printf("semop error: (P(sem))\n");
		return -1;
	}
	return 0;
}

int V(int s){ // release (increase)
	sop.sem_num = 0;
	sop.sem_op = 1;
	sop.sem_flg = 0;

	if(semop(s, &sop, 1) < 0){
		printf("semop error: (V(sem))\n");
		return -1;
	}
	return 0;
}