#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <nav_core/recovery_behavior.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "sensor_msgs/LaserScan.h"
#include "stdio.h"
#include "boost/asio.hpp"   //包含boost库函数
#include "boost/bind.hpp"
#include "math.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//pclz

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/registration.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>

//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "../include/calib_odom/Odom_Calib.hpp"

#include <csm/csm_all.h>

using namespace std;
using namespace boost::asio;//定义一个命名空间，用于后面的读写操作
//using namespace karto;

//用来进行里程计矫正的类
OdomCalib Odom_calib;


//std::vector<geometry_msgs::PointStamped> mcu_path;

Eigen::Vector3d  cal_delta_distence(Eigen::Vector3d odom_pose);
/*
 * 获取激光数据类
*/
class Scan2
{
public:
    Scan2();

    //进行PI-ICP需要的变量
    LDP m_prevLDP;
    sm_params m_PIICPParams;
    sm_result m_OutputResult;

    //odom & scan　进行位姿积分
    Eigen::Vector3d scan_pos_cal;
    Eigen::Vector3d odom_pos_cal;


    std::vector<Eigen::Vector3d> odom_increments;          //用来储存两帧之间的里程计的增量

    std::string odom_frame_;
    std::string base_frame_;

    ros::NodeHandle node_;
    tf::TransformListener tf_;

    ros::Subscriber calib_flag_sub_;

    ros::Publisher odom_path_pub_,scan_path_pub_,calib_path_pub_;

    nav_msgs::Path path_odom,path_scan;
    ros::Time current_time;

    //进行时间同步
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;

    void CalibFlagCallBack(const std_msgs::Empty &msg);

    //回调函数
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr&scan2);

    //tf树查询里程计位姿
    bool getOdomPose(Eigen::Vector3d& pose, const ros::Time& t);

    //发布odom & laser path
    void pub_msg( Eigen::Vector3d& pose,nav_msgs::Path &path,ros::Publisher &mcu_path_pub_);

    //为了发布correct path
    void publishPathEigen(std::vector<Eigen::Vector3d>& path_eigen,ros::Publisher& path_pub_);

    //进行pl-icp的相关函数.
    void SetPIICPParams();
    void LaserScanToLDP(sensor_msgs::LaserScan *pScan,
                                 LDP& ldp);
    Eigen::Vector3d  PIICPBetweenTwoFrames(LDP& currentLDPScan,
                                           Eigen::Vector3d tmprPose);


};


Eigen::Vector3d now_pos,last_pos;

void Scan2:: pub_msg( Eigen::Vector3d& pose,nav_msgs::Path &path,ros::Publisher &mcu_path_pub_)
{

    current_time = ros::Time::now();
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = pose(0);
    this_pose_stamped.pose.position.y = pose(1);

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="odom";
    path.poses.push_back(this_pose_stamped);
    mcu_path_pub_.publish(path);

}

void Scan2::publishPathEigen(std::vector<Eigen::Vector3d>& path_eigen,ros::Publisher& path_pub_)
{
    nav_msgs::Path visual_path;

    current_time = ros::Time::now();

    visual_path.header.stamp = ros::Time::now();
    visual_path.header.frame_id="odom";

    geometry_msgs::PoseStamped tmpPose;
    tmpPose.header.stamp = current_time;
    tmpPose.header.frame_id="odom";

    for(int i = 0 ;i < path_eigen.size();i++)
    {
        Eigen::Vector3d poseEigen = path_eigen[i];

        tmpPose.pose.position.x = poseEigen(0);
        tmpPose.pose.position.y = poseEigen(1);

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(poseEigen(2));
        tmpPose.pose.orientation.x = goal_quat.x;
        tmpPose.pose.orientation.y = goal_quat.y;
        tmpPose.pose.orientation.z = goal_quat.z;
        tmpPose.pose.orientation.w = goal_quat.w;

        visual_path.poses.push_back(tmpPose);
    }

    path_pub_.publish(visual_path);
}

/*
 * 得到时刻t时候 机器人在里程计坐标下的坐标
*/
bool Scan2::getOdomPose(Eigen::Vector3d& pose, const ros::Time& t)
{
    // Get the robot's pose
    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                               tf::Vector3(0,0,0)), t, base_frame_);
    tf::Stamped<tf::Transform> odom_pose;
    try
    {
        tf_.transformPose(odom_frame_, ident, odom_pose);
    }


    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    double yaw = tf::getYaw(odom_pose.getRotation());



    pose << odom_pose.getOrigin().x(),
            odom_pose.getOrigin().y(),
            yaw;
    //pub_msg(pose,path_odom,odom_path_pub_);
    return true;
}


//构造函数
Scan2::Scan2()
{
    ros::NodeHandle private_nh_("~");

    m_prevLDP = NULL;
    SetPIICPParams();

    scan_pos_cal.setZero();
    odom_pos_cal.setZero();
    odom_increments.clear();

    if(!private_nh_.getParam("odom_frame", odom_frame_))
        odom_frame_ = "odom";
    if(!private_nh_.getParam("base_frame", base_frame_))
        base_frame_ = "base_link";

    //订阅对应的topic 接受到这个topic 系统就开始进行最小二乘的解算
    calib_flag_sub_ = node_.subscribe("calib_flag",5,&Scan2::CalibFlagCallBack,this);

    //发布路径
    odom_path_pub_ = node_.advertise<nav_msgs::Path>("odom_path_pub_",1,true);
    scan_path_pub_ = node_.advertise<nav_msgs::Path>("scan_path_pub_",1,true);
    calib_path_pub_ = node_.advertise<nav_msgs::Path>("calib_path_pub_",1,true);
    current_time = ros::Time::now();

    path_odom.header.stamp=current_time;
    path_scan.header.stamp=current_time;
    path_odom.header.frame_id="odom";
    path_scan.header.frame_id="odom";

    //进行里程计和激光雷达数据的同步
    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "/sick_scan", 10);
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 10);
    scan_filter_->registerCallback(boost::bind(&Scan2::scanCallBack, this, _1));

    std::cout <<"Calibration Online,Wait for Data!!!!!!!"<<std::endl;
}

//订阅topic表示开始进行标定.
void Scan2::CalibFlagCallBack(const std_msgs::Empty &msg)
{
    Eigen::Matrix3d correct_matrix = Odom_calib.Solve();

    Eigen::Matrix3d tmp_transform_matrix;

    std::cout<<"correct_matrix:"<<std::endl<<correct_matrix<<std::endl;

    //计算矫正之后的路径
    Eigen::Vector3d calib_pos(0,0,0);                 //矫正之后的位姿
    std::vector<Eigen::Vector3d> calib_path_eigen;    //矫正之后的路径
    for(int i = 0; i < odom_increments.size();i++)
    {
        Eigen::Vector3d odom_inc = odom_increments[i];
        Eigen::Vector3d correct_inc = correct_matrix * odom_inc;

        tmp_transform_matrix << cos(calib_pos(2)),-sin(calib_pos(2)),0,
                            sin(calib_pos(2)), cos(calib_pos(2)),0,
                                            0,                 0,1;

        calib_pos += tmp_transform_matrix * correct_inc;

        calib_path_eigen.push_back(calib_pos);
    }

    //发布矫正之后的路径
    publishPathEigen(calib_path_eigen,calib_path_pub_);

    //矫正完毕，退出订阅
    scan_filter_sub_->unsubscribe();

    std::cout <<"calibration over!!!!"<<std::endl;
}


//激光数据回调函数
void Scan2::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &_laserScanMsg)
{
    static long int dataCnt = 0;
    sensor_msgs::LaserScan scan;
    Eigen::Vector3d odom_pose;              //激光对应的里程计位姿
    Eigen::Vector3d d_point_odom;           //里程计计算的dpose
    Eigen::Vector3d d_point_scan;           //激光的scanmatch计算的dpose
    Eigen::MatrixXd transform_matrix(3,3);  //临时的变量

    double c,s;
    scan = *_laserScanMsg;

    //得到对应的里程计数据
    if(!getOdomPose(odom_pose, _laserScanMsg->header.stamp))
        return ;

    //前后两帧里程计的位姿差
    d_point_odom = cal_delta_distence(odom_pose);

    //如果运动的距离太短，则不进行处理．
    if(d_point_odom(0) < 0.05 &&
       d_point_odom(1) < 0.05 &&
       d_point_odom(2) < tfRadians(5.0))
    {
        return ;
    }
    last_pos = now_pos;

    //记录下里程计的增量数据
    odom_increments.push_back(d_point_odom);


    //把当前的激光数据转换为 pl-icp能识别的数据 & 进行矫正
    //d_point_scan就是用激光计算得到的两帧数据之间的旋转 & 平移
    LDP currentLDP;
    if(m_prevLDP != NULL)
    {
        LaserScanToLDP(&scan,currentLDP);
        d_point_scan = PIICPBetweenTwoFrames(currentLDP,d_point_odom);
    }
    else
    {
        LaserScanToLDP(&scan,m_prevLDP);
    }

    // 构造旋转矩阵 生成三种位姿

    // 两针scan计算本身累计的位姿 for laser_path visualization
    c = cos(scan_pos_cal(2));
    s = sin(scan_pos_cal(2));
    transform_matrix<<c,-s,0,
                      s, c,0,
                      0, 0,1;
    scan_pos_cal+=(transform_matrix*d_point_scan);

    // 里程计累计的位姿          for odom_path visualization
    c = cos(odom_pos_cal(2));
    s = sin(odom_pos_cal(2));
    transform_matrix<<c,-s,0,
                      s, c,0,
                      0, 0,1;
    odom_pos_cal+=(transform_matrix*d_point_odom);

    //放到路径当中 //for visualization
    pub_msg(odom_pos_cal,path_odom,odom_path_pub_);
    pub_msg(scan_pos_cal,path_scan,scan_path_pub_);

    //构造超定方程组
    Odom_calib.Add_Data(d_point_odom,d_point_scan);
    dataCnt++;

    std::cout <<"Data Cnt:"<<dataCnt<<std::endl;
}

//TODO:
//求解得到两帧数据之间的位姿差
//即求解当前位姿　在　上一时刻　坐标系中的坐标
Eigen::Vector3d  cal_delta_distence(Eigen::Vector3d odom_pose)
{

    Eigen::Vector3d d_pos;  //return value
    now_pos = odom_pose;

    //TODO:

    Eigen::Matrix3d Tnow,Tprev;

    double theta = last_pos(2);
    double x = last_pos(0);
    double y = last_pos(1);

    //前一次的位姿
    Tprev << cos(theta),-sin(theta),x,
             sin(theta), cos(theta),y,
                      0,          0,       1;

    //当前的位姿
    x = now_pos(0);
    y = now_pos(1);
    theta = now_pos(2);
    Tnow << cos(theta),-sin(theta),x,
             sin(theta), cos(theta),y,
                      0,          0,       1;

    //相对位姿
    Eigen::Matrix3d T = Tprev.inverse() * Tnow;

    d_pos(0) = T(0,2);
    d_pos(1) = T(1,2);
    d_pos(2) = atan2(T(1,0),T(0,0));

    //end of TODO:

    return d_pos;
}

//设置PI-ICP的参数
void Scan2::SetPIICPParams()
{
    //设置激光的范围
    m_PIICPParams.min_reading = 0.1;
    m_PIICPParams.max_reading = 20;

    //设置位姿最大的变化范围
    m_PIICPParams.max_angular_correction_deg = 20.0;
    m_PIICPParams.max_linear_correction = 1;

    //设置迭代停止的条件
    m_PIICPParams.max_iterations = 50;
    m_PIICPParams.epsilon_xy = 0.000001;
    m_PIICPParams.epsilon_theta = 0.0000001;

    //设置correspondence相关参数
    m_PIICPParams.max_correspondence_dist = 1;
    m_PIICPParams.sigma = 0.01;
    m_PIICPParams.use_corr_tricks = 1;

    //设置restart过程，因为不需要restart所以可以不管
    m_PIICPParams.restart = 0;
    m_PIICPParams.restart_threshold_mean_error = 0.01;
    m_PIICPParams.restart_dt = 1.0;
    m_PIICPParams.restart_dtheta = 0.1;

    //设置聚类参数
    m_PIICPParams.clustering_threshold = 0.2;

    //用最近的10个点来估计方向
    m_PIICPParams.orientation_neighbourhood = 10;

    //设置使用PI-ICP
    m_PIICPParams.use_point_to_line_distance = 1;

    //不进行alpha_test
    m_PIICPParams.do_alpha_test = 0;
    m_PIICPParams.do_alpha_test_thresholdDeg = 5;

    //设置trimmed参数 用来进行outlier remove
    m_PIICPParams.outliers_maxPerc = 0.9;
    m_PIICPParams.outliers_adaptive_order = 0.7;
    m_PIICPParams.outliers_adaptive_mult = 2.0;

    //进行visibility_test 和 remove double
    m_PIICPParams.do_visibility_test = 1;
    m_PIICPParams.outliers_remove_doubles = 1;
    m_PIICPParams.do_compute_covariance = 0;
    m_PIICPParams.debug_verify_tricks = 0;
    m_PIICPParams.use_ml_weights = 0;
    m_PIICPParams.use_sigma_weights = 0;
}



//把激光雷达数据 转换为PI-ICP需要的数据
void Scan2::LaserScanToLDP(sensor_msgs::LaserScan *pScan,
                           LDP& ldp)
{
    int nPts = pScan->intensities.size();
    ldp = ld_alloc_new(nPts);

    for(int i = 0;i < nPts;i++)
    {
        double dist = pScan->ranges[i];
        if(dist > 0.1 && dist < 20)
        {
            ldp->valid[i] = 1;
            ldp->readings[i] = dist;
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1;
        }
        ldp->theta[i] = pScan->angle_min+pScan->angle_increment*i;
    }
    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[nPts-1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}


//求两帧之间的icp位姿匹配
Eigen::Vector3d  Scan2::PIICPBetweenTwoFrames(LDP& currentLDPScan,
                                              Eigen::Vector3d tmprPose)
{
    m_prevLDP->odometry[0] = 0.0;
    m_prevLDP->odometry[1] = 0.0;
    m_prevLDP->odometry[2] = 0.0;

    m_prevLDP->estimate[0] = 0.0;
    m_prevLDP->estimate[1] = 0.0;
    m_prevLDP->estimate[2] = 0.0;

    m_prevLDP->true_pose[0] = 0.0;
    m_prevLDP->true_pose[1] = 0.0;
    m_prevLDP->true_pose[2] = 0.0;

    //设置匹配的参数值
    m_PIICPParams.laser_ref = m_prevLDP;
    m_PIICPParams.laser_sens = currentLDPScan;

    m_PIICPParams.first_guess[0] = tmprPose(0);
    m_PIICPParams.first_guess[1] = tmprPose(1);
    m_PIICPParams.first_guess[2] = tmprPose(2);

    m_OutputResult.cov_x_m = 0;
    m_OutputResult.dx_dy1_m = 0;
    m_OutputResult.dx_dy2_m = 0;

    sm_icp(&m_PIICPParams,&m_OutputResult);

    //nowPose在lastPose中的坐标
    Eigen::Vector3d  rPose;
    if(m_OutputResult.valid)
    {
        //得到两帧激光之间的相对位姿
        rPose(0)=(m_OutputResult.x[0]);
        rPose(1)=(m_OutputResult.x[1]);
        rPose(2)=(m_OutputResult.x[2]);

//        std::cout <<"Iter:"<<m_OutputResult.iterations<<std::endl;
//        std::cout <<"Corr:"<<m_OutputResult.nvalid<<std::endl;
//        std::cout <<"Erro:"<<m_OutputResult.error<<std::endl;

//        std::cout <<"PI ICP GOOD"<<std::endl;
    }
    else
    {
        std::cout <<"PI ICP Failed!!!!!!!"<<std::endl;
        rPose = tmprPose;
    }

    //更新

    //ld_free(m_prevLDP);

    m_prevLDP = currentLDPScan;

    return rPose;
}



int main(int argc,char** argv)
{
    ros::init(argc, argv, "message_filter_node");
    ros::Time::init();
    ros::NodeHandle n;
    Scan2 scan;


    Odom_calib.Set_data_len(12000);
    Odom_calib.set_data_zero();
    ros::spin();


    return 0;
}
