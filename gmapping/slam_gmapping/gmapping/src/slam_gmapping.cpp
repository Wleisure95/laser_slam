/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */
/* Modified by: Charles DuHadway */


/**

@mainpage slam_gmapping

@htmlinclude manifest.html

@b slam_gmapping is a wrapper around the GMapping SLAM library. It reads laser
scans and odometry and computes a map. This map can be
written to a file using e.g.

  "rosrun map_server map_saver static_map:=dynamic_map"

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : data from a laser range scanner
- @b "/tf": odometry from the robot


Publishes to (name/type):
- @b "/tf"/tf/tfMessage: position relative to the map


@section services
 - @b "~dynamic_map" : returns the map


@section parameters ROS parameters

Reads the following parameters from the parameter server

Parameters used by our GMapping wrapper:

- @b "~throttle_scans": @b [int] throw away every nth laser scan
- @b "~base_frame": @b [string] the tf frame_id to use for the robot base pose
- @b "~map_frame": @b [string] the tf frame_id where the robot pose on the map is published
- @b "~odom_frame": @b [string] the tf frame_id from which odometry is read
- @b "~map_update_interval": @b [double] time in seconds between two recalculations of the map


Parameters used by GMapping itself:

Laser Parameters:
激光雷达的相关参数

激光雷达的最大距离和最大使用距离
- @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond this range get discarded completely. (default: maximum laser range minus 1 cm, as received in the the first LaserScan message)
- @b "~/maxUrange" @b [double] maximum range of the laser scanner that is used for map building (default: same as maxRange)

scan-match计算score时的方差(不是标准差)，单位的cell
score是指整个指数项。
likelihood是指高斯分布的指数项的系数，即e的指数系数
- @b "~/sigma" @b [double] standard deviation for the scan matching process (cell)

进行scan-match的时候寻找激光的匹配点的时候使用。
因为不是用的似然场模型，所以激光击中的点由于噪声的影响会在真实障碍物的附近。
因此需要在一个激光击中的点的邻域内进行查找，这个参数定义邻域的范围
这个数值表示单位表示cell的单位 也就是说这个值应该是整数
- @b "~/kernelSize" @b [double] search window for the scan matching process

scan-matching的过程中的初始的搜索步长和迭代次数
- @b "~/lstep" @b [double] initial search step for scan matching (linear)
- @b "~/astep" @b [double] initial search step for scan matching (angular)
- @b "~/iterations" @b [double] number of refinement steps in the scan matching. The final "precision" for the match is lstep*2^(-iterations) or astep*2^(-iterations), respectively.

计算在likelihoodandscore()函数中 计算likelihood使用用的方差(不是标准差)
likelihood是指高斯分布的指数项的系数，即e的指数
score是指整个指数项。
- @b "~/lsigma" @b [double] standard deviation for the scan matching process (single laser beam)


- @b "~/ogain" @b [double] gain for smoothing the likelihood

对于一帧激光雷达数据来说 只取每第(n+1)个激光束  这个是相对于scan-match来说的。
如果n等于0 则取每第1帧激光束
如果n等于1 则取每第2帧激光束 也就是说使用的激光束变成原来的1/2
如果n等于2 则取每第3帧激光束 也就是说使用的激光束变成原来的1/3
- @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a match (0 = take all rays)

scan-matching结果接受的最小得分
- @b "~/minimumScore" @b [double] minimum score for considering the outcome of the scanmatching good. Can avoid 'jumping' pose estimates in large open spaces when using laser scanners with limited range (e.g. 5m). (0 = default. Scores go up to 600+, try 50 for example when experiencing 'jumping' estimate issues)

Motion Model Parameters (all standard deviations of a gaussian noise model)
机器人的运动模型的相关的噪声参数
- @b "~/srr" @b [double] linear noise component (x and y)
- @b "~/stt" @b [double] angular noise component (theta)
- @b "~/srt" @b [double] linear -> angular noise component
- @b "~/str" @b [double] angular -> linear noise component

Others:
进行滤波器更新的最小距离
- @b "~/linearUpdate" @b [double] the robot only processes new measurements if the robot has moved at least this many meters
- @b "~/angularUpdate" @b [double] the robot only processes new measurements if the robot has turned at least this many rads

粒子滤波器的相关参数
- @b "~/resampleThreshold" @b [double] threshold at which the particles get resampled. Higher means more frequent resampling.
- @b "~/particles" @b [int] (fixed) number of particles. Each particle represents a possible trajectory that the robot has traveled

Likelihood sampling (used in scan matching)
- @b "~/llsamplerange" @b [double] linear range
- @b "~/lasamplerange" @b [double] linear step size
- @b "~/llsamplestep" @b [double] linear range
- @b "~/lasamplestep" @b [double] angular step size

Initial map dimensions and resolution:
初始时候地图的维度和分辨率
- @b "~/xmin" @b [double] minimum x position in the map [m]
- @b "~/ymin" @b [double] minimum y position in the map [m]
- @b "~/xmax" @b [double] maximum x position in the map [m]
- @b "~/ymax" @b [double] maximum y position in the map [m]
- @b "~/delta" @b [double] size of one pixel [m]

*/



#include "slam_gmapping.h"

#include <iostream>

#include <time.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"

#include "std_msgs/String.h"

#include "tf/LinearMath/Scalar.h"

#include "../include/gmapping/sensor/sensor_range/rangesensor.h"
#include "../include/gmapping/sensor/sensor_odometry/odometrysensor.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamGMapping::SlamGMapping():
    map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
    laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
    seed_ = time(NULL);
    init();
}

SlamGMapping::SlamGMapping(long unsigned int seed, long unsigned int max_duration_buffer):
    map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
    laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL),
    seed_(seed), tf_(ros::Duration(max_duration_buffer))
{
    init();
}


/*
slamgmapping的初始化，主要用来读取配置文件中写入的参数
*/

void SlamGMapping::init()
{
    // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

    // The library is pretty chatty
    //gsp_ = new GMapping::GridSlamProcessor(std::cerr);
    gsp_ = new GMapping::GridSlamProcessor();
    ROS_ASSERT(gsp_);

    tfB_ = new tf::TransformBroadcaster();
    ROS_ASSERT(tfB_);

    gsp_laser_ = NULL;
    gsp_odom_ = NULL;

    got_first_scan_ = false;
    got_map_ = false;



    // Parameters used by our GMapping wrapper GMapping的ROS壳使用的参数
    if(!private_nh_.getParam("throttle_scans", throttle_scans_))
        throttle_scans_ = 1;
    if(!private_nh_.getParam("base_frame", base_frame_))
        base_frame_ = "base_link";
    if(!private_nh_.getParam("map_frame", map_frame_))
        map_frame_ = "map";
    if(!private_nh_.getParam("odom_frame", odom_frame_))
        odom_frame_ = "odom";

    /****************************************************************************************
   * **************************************************************************************/

    /////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////

    private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

    double tmp;
    if(!private_nh_.getParam("map_update_interval", tmp))
        tmp = 5.0;
    map_update_interval_.fromSec(tmp);

    // Parameters used by GMapping itself     GMapping算法本身使用的参数
    maxUrange_ = 0.0;  maxRange_ = 0.0; // preliminary default, will be set in initMapper()
    if(!private_nh_.getParam("minimumScore", minimum_score_))
        minimum_score_ = 0;
    if(!private_nh_.getParam("sigma", sigma_))
        sigma_ = 0.05;
    if(!private_nh_.getParam("kernelSize", kernelSize_))
        kernelSize_ = 1;
    if(!private_nh_.getParam("lstep", lstep_))
        lstep_ = 0.05;
    if(!private_nh_.getParam("astep", astep_))
        astep_ = 0.05;
    if(!private_nh_.getParam("iterations", iterations_))
        iterations_ = 5;
    if(!private_nh_.getParam("lsigma", lsigma_))
        lsigma_ = 0.075;
    if(!private_nh_.getParam("ogain", ogain_))
        ogain_ = 3.0;
    if(!private_nh_.getParam("lskip", lskip_))
        lskip_ = 0;
    if(!private_nh_.getParam("srr", srr_))
        srr_ = 0.1;
    if(!private_nh_.getParam("srt", srt_))
        srt_ = 0.2;
    if(!private_nh_.getParam("str", str_))
        str_ = 0.1;
    if(!private_nh_.getParam("stt", stt_))
        stt_ = 0.2;
    if(!private_nh_.getParam("linearUpdate", linearUpdate_))
        linearUpdate_ = 1.0;
    if(!private_nh_.getParam("angularUpdate", angularUpdate_))
        angularUpdate_ = 0.5;
    if(!private_nh_.getParam("temporalUpdate", temporalUpdate_))
        temporalUpdate_ = -1.0;
    if(!private_nh_.getParam("resampleThreshold", resampleThreshold_))
        resampleThreshold_ = 0.5;
    if(!private_nh_.getParam("particles", particles_))
        particles_ = 30;
    if(!private_nh_.getParam("xmin", xmin_))
        xmin_ = -100.0;
    if(!private_nh_.getParam("ymin", ymin_))
        ymin_ = -100.0;
    if(!private_nh_.getParam("xmax", xmax_))
        xmax_ = 100.0;
    if(!private_nh_.getParam("ymax", ymax_))
        ymax_ = 100.0;
    if(!private_nh_.getParam("delta", delta_))
        delta_ = 0.05;
    if(!private_nh_.getParam("occ_thresh", occ_thresh_))
        occ_thresh_ = 0.25;
    if(!private_nh_.getParam("llsamplerange", llsamplerange_))
        llsamplerange_ = 0.01;
    if(!private_nh_.getParam("llsamplestep", llsamplestep_))
        llsamplestep_ = 0.01;
    if(!private_nh_.getParam("lasamplerange", lasamplerange_))
        lasamplerange_ = 0.005;
    if(!private_nh_.getParam("lasamplestep", lasamplestep_))
        lasamplestep_ = 0.005;

    if(!private_nh_.getParam("tf_delay", tf_delay_))
        tf_delay_ = transform_publish_period_;
}


/*订阅一些主题 发布一些主题*/
void SlamGMapping::startLiveSlam()
{
    entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
    sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);

    {
        //订阅激光数据 同时和odom_frame之间的转换同步
        scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
        scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
        scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1));

        std::cout <<"Subscribe LaserScan & odom!!!"<<std::endl;
    }

    /*发布转换关系的线程*/
    transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));
}

//用bag进行gmapping建图的时候，使用这个函数
void SlamGMapping::startReplay(const std::string & bag_fname, std::string scan_topic)
{
    double transform_publish_period;
    ros::NodeHandle private_nh_("~");
    entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
    sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);

    rosbag::Bag bag;
    bag.open(bag_fname, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/tf"));
    topics.push_back(scan_topic);
    rosbag::View viewall(bag, rosbag::TopicQuery(topics));

    // Store up to 5 messages and there error message (if they cannot be processed right away)
    std::queue<std::pair<sensor_msgs::LaserScan::ConstPtr, std::string> > s_queue;
    foreach(rosbag::MessageInstance const m, viewall)
    {
        tf::tfMessage::ConstPtr cur_tf = m.instantiate<tf::tfMessage>();
        if (cur_tf != NULL)
        {
            for (size_t i = 0; i < cur_tf->transforms.size(); ++i)
            {
                geometry_msgs::TransformStamped transformStamped;
                tf::StampedTransform stampedTf;
                transformStamped = cur_tf->transforms[i];
                tf::transformStampedMsgToTF(transformStamped, stampedTf);
                tf_.setTransform(stampedTf);
            }
        }

        sensor_msgs::LaserScan::ConstPtr s = m.instantiate<sensor_msgs::LaserScan>();
        if (s != NULL)
        {
            if (!(ros::Time(s->header.stamp)).is_zero())
            {
                s_queue.push(std::make_pair(s, ""));
            }
            // Just like in live processing, only process the latest 5 scans
            if (s_queue.size() > 5)
            {
                ROS_WARN_STREAM("Dropping old scan: " << s_queue.front().second);
                s_queue.pop();
            }
            // ignoring un-timestamped tf data
        }

        // Only process a scan if it has tf data
        while (!s_queue.empty())
        {
            try
            {
                tf::StampedTransform t;
                tf_.lookupTransform(s_queue.front().first->header.frame_id, odom_frame_, s_queue.front().first->header.stamp, t);
                this->laserCallback(s_queue.front().first);
                s_queue.pop();
            }
            // If tf does not have the data yet
            catch(tf2::TransformException& e)
            {
                // Store the error to display it if we cannot process the data after some time
                s_queue.front().second = std::string(e.what());
                break;
            }
        }
    }

    bag.close();
}

/*发布map->odom的转换关系*/
void SlamGMapping::publishLoop(double transform_publish_period)
{
    if(transform_publish_period == 0)
        return;

    ros::Rate r(1.0 / transform_publish_period);
    while(ros::ok())
    {
        publishTransform();
        r.sleep();
    }
}

/*析构函数*/
SlamGMapping::~SlamGMapping()
{
    if(transform_thread_)
    {
        transform_thread_->join();
        delete transform_thread_;
    }

    delete gsp_;
    if(gsp_laser_)
        delete gsp_laser_;
    if(gsp_odom_)
        delete gsp_odom_;
    if (scan_filter_)
        delete scan_filter_;
    if (scan_filter_sub_)
        delete scan_filter_sub_;
}

/*得到里程计的位姿 即激光雷达在里程计坐标系中的位姿*/
bool SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
    // Get the pose of the centered laser at the right time
    centered_laser_pose_.stamp_ = t;
    // Get the laser's pose that is centered
    tf::Stamped<tf::Transform> odom_pose;
    try
    {
        tf_.transformPose(odom_frame_, centered_laser_pose_, odom_pose);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    double yaw = tf::getYaw(odom_pose.getRotation());

    gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),
                                        odom_pose.getOrigin().y(),
                                        yaw);
    return true;
}

/**
 * @brief SlamGMapping::initMapper  gmapping算法的初始化
 * 这个函数在收到的第一帧激光雷达数据的时候会被调用一次，之后就再也不会被调用了。
 * 这个函数的功能主要就是对gmapping算法中需要的一些参数进行赋值，即对gmapping算法进行初始化
 * 这个参数一开始由gmapping wrapper读取，但是在这个函数里面真正的会赋值给gmapping算法
 * 1.判断激光雷达是否是水平放置的，如果不是 则报错。
 * 2.假设激光雷达数据的角度是对称的 & 递增的 为每个激光束分配角度。
 * 3.为gmapping算法设置各种需要的参数。
 *
 *
 * @param scan
 * @return
 */
bool SlamGMapping::initMapper(const sensor_msgs::LaserScan& scan)
{

    //得到激光雷达相对于车身坐标系(base_link)的位姿
    laser_frame_ = scan.header.frame_id;
    // Get the laser's pose, relative to base.
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = laser_frame_;
    ident.stamp_ = scan.header.stamp;
    try
    {
        tf_.transformPose(base_frame_, ident, laser_pose);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
                 e.what());
        return false;
    }

    // create a point 1m above the laser position and transform it into the laser-frame
    // 创造一个比激光雷达高1m的一个点，然后把这个点转换到激光雷达坐标系 这个点主要用来检测激光雷达和车身的位姿关系
    // 来判断激光雷达是否是倾斜的 判断条件：如果激光雷达是水平放置的，那么转换回去之后，z的坐标也依然是1
    tf::Vector3 v;
    v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
    tf::Stamped<tf::Vector3> up(v, scan.header.stamp,
                                base_frame_);
    try
    {
        tf_.transformPoint(laser_frame_, up, up);
        ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
    }
    catch(tf::TransformException& e)
    {
        ROS_WARN("Unable to determine orientation of laser: %s",
                 e.what());
        return false;
    }

    // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
    // 如果激光雷达不是水平放置的，则会进行报错。
    if (fabs(fabs(up.z()) - 1) > 0.001)
    {
        ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                 up.z());
        return false;
    }

    //激光雷达的数量
    gsp_laser_beam_count_ = scan.ranges.size();

    //激光雷达的中间角度
    double angle_center = (scan.angle_min + scan.angle_max)/2;

    //判断激光雷达是正着放置的还是反着放置的。
    if (up.z() > 0)
    {
        do_reverse_range_ = scan.angle_min > scan.angle_max;
        centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center),
                                                                   tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
        ROS_INFO("Laser is mounted upwards.");
    }
    else
    {
        do_reverse_range_ = scan.angle_min < scan.angle_max;
        centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(M_PI,0,-angle_center),
                                                                   tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
        ROS_INFO("Laser is mounted upside down.");
    }


    // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
    // 这里认为激光雷达数据的角度是对称的，且是从小到大递增的。
    // 因此根据激光雷达数据中的angle_min,angle_max,angle_increment等数据为每个激光束分配角度。
    laser_angles_.resize(scan.ranges.size());
    // Make sure angles are started so that they are centered
    double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;
    for(unsigned int i=0; i<scan.ranges.size(); ++i)
    {
        laser_angles_[i]=theta;
        theta += std::fabs(scan.angle_increment);
    }


    ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan.angle_min, scan.angle_max,
              scan.angle_increment);
    ROS_DEBUG("Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(),
              laser_angles_.back(), std::fabs(scan.angle_increment));


    GMapping::OrientedPoint gmap_pose(0, 0, 0);

    // setting maxRange and maxUrange here so we can set a reasonable default
    // 设置激光雷达的最大观测距离和最大使用距离
    ros::NodeHandle private_nh_("~");
    if(!private_nh_.getParam("maxRange", maxRange_))
        maxRange_ = scan.range_max - 0.01;
    if(!private_nh_.getParam("maxUrange", maxUrange_))
        maxUrange_ = maxRange_;

    // The laser must be called "FLASER".
    // We pass in the absolute value of the computed angle increment, on the
    // assumption that GMapping requires a positive angle increment.  If the
    // actual increment is negative, we'll swap the order of ranges before
    // feeding each scan to GMapping.
    // 根据上面得到的激光雷达的数据信息 来初始化一个激光传感器
    gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                           gsp_laser_beam_count_,
                                           fabs(scan.angle_increment),
                                           gmap_pose,
                                           0.0,
                                           maxRange_);
    ROS_ASSERT(gsp_laser_);

    //为gmapping算法设置sensormap
    GMapping::SensorMap smap;
    smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
    gsp_->setSensorMap(smap);

    //初始化里程计传感器
    gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
    ROS_ASSERT(gsp_odom_);


    /// @todo Expose setting an initial pose
    /// 得到里程计的初始位姿，如果没有 则把初始位姿设置为(0,0,0)
    GMapping::OrientedPoint initialPose;
    if(!getOdomPose(initialPose, scan.header.stamp))
    {
        ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
        initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
    }

    //为gmapping算法设置各种参数
    gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                                kernelSize_, lstep_, astep_, iterations_,
                                lsigma_, ogain_, lskip_);

    gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
    gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
    gsp_->setUpdatePeriod(temporalUpdate_);
    gsp_->setgenerateMap(false);
    gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                  delta_, initialPose);

    gsp_->setllsamplerange(llsamplerange_);
    gsp_->setllsamplestep(llsamplestep_);

    /// @todo Check these calls; in the gmapping gui, they use
    /// llsamplestep and llsamplerange intead of lasamplestep and
    /// lasamplerange.  It was probably a typo, but who knows.
    gsp_->setlasamplerange(lasamplerange_);
    gsp_->setlasamplestep(lasamplestep_);
    gsp_->setminimumScore(minimum_score_);

    // Call the sampling function once to set the seed.
    GMapping::sampleGaussian(1,seed_);

    ROS_INFO("Initialization complete");

    return true;
}

/*
 * 加入一个激光雷达的数据 主要函数 这里面会调用processScan()函数
 * 这个函数被laserCallback()函数调用
*/
bool SlamGMapping::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
    //得到与激光的时间戳相对应的机器人的里程计的位姿
    if(!getOdomPose(gmap_pose, scan.header.stamp))
        return false;

    //检测是否所有帧的数据都是相等的 如果不相等就进行计算 不知道为什么 感觉完全没必要啊。
    //特别是对于champion_nav_msgs的LaserScan来说 激光束的数量经常变
    if(scan.ranges.size() != gsp_laser_beam_count_)
        return false;

    // GMapping wants an array of doubles...
    double* ranges_double = new double[scan.ranges.size()];

    // If the angle increment is negative, we have to invert the order of the readings.
    // 如果激光是反着装的，这激光的顺序需要反过来，同时这里会排除掉所有激光距离小于range_min的值。
    // 排除的方式是把他们设置为最大值
    if (do_reverse_range_)
    {
        ROS_DEBUG("Inverting scan");
        int num_ranges = scan.ranges.size();
        for(int i=0; i < num_ranges; i++)
        {
            // Must filter out short readings, because the mapper won't
            if(scan.ranges[num_ranges - i - 1] < scan.range_min)
                ranges_double[i] = (double)scan.range_max;
            else
                ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
        }
    }
    else
    {
        for(unsigned int i=0; i < scan.ranges.size(); i++)
        {
            // Must filter out short readings, because the mapper won't
            if(scan.ranges[i] < scan.range_min)
                ranges_double[i] = (double)scan.range_max;
            else
                ranges_double[i] = (double)scan.ranges[i];
        }
    }

    //把ROS的激光雷达数据信息 转换为 GMapping算法看得懂的形式
    GMapping::RangeReading reading(scan.ranges.size(),
                                   ranges_double,
                                   gsp_laser_,
                                   scan.header.stamp.toSec());

    // ...but it deep copies them in RangeReading constructor, so we don't
    // need to keep our array around.
    // 上面的初始话是进行深拷贝 因此申请的内存可以直接释放。
    delete[] ranges_double;

    //设置和激光数据的时间戳匹配的机器人的位姿
    reading.setPose(gmap_pose);

    ROS_DEBUG("processing scan");

    //调用gmapping算法进行处理
    return gsp_->processScan(reading);
}

/*
 * 接受到激光雷达数据的回调函数 在这里面调用addScan()函数
 * 如果addScan()函数调用成功，也就是说激光数据被成功的插入到地图中后，
 * 如果到了地图更新的时间，则对地图进行更新，通过调用updateMap()函数来进行相应的操作。
 *
 * laserCallback()->addScan()->gmapping::processScan()
 *                ->updateMap()
 *
*/
void SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{

    laser_count_++;
    if ((laser_count_ % throttle_scans_) != 0)
        return;

    static ros::Time last_map_update(0,0);

    // We can't initialize the mapper until we've got the first scan
    if(!got_first_scan_)
    {
        if(!initMapper(*scan))
            return;
        got_first_scan_ = true;
    }

    GMapping::OrientedPoint odom_pose;
    if(addScan(*scan, odom_pose))
    {
        ROS_DEBUG("scan processed");

        GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
        ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
        ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
        ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

        tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
        tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

        map_to_odom_mutex_.lock();
        map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
        map_to_odom_mutex_.unlock();

        /*如果没有地图那肯定需要直接更新，如果有地图了则需要到时间了，才更新地图了*/
        if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
        {
            /*多久更新一次地图*/
            updateMap(*scan);
            last_map_update = scan->header.stamp;
            ROS_DEBUG("Updated the map");
        }
    }
    else
        ROS_DEBUG("cannot process scan");
}

/*计算位姿的信息熵*/
double SlamGMapping::computePoseEntropy()
{
    double weight_total=0.0;
    for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
        it != gsp_->getParticles().end();
        ++it)
    {
        weight_total += it->weight;
    }
    double entropy = 0.0;
    for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
        it != gsp_->getParticles().end();
        ++it)
    {
        if(it->weight/weight_total > 0.0)
            entropy += it->weight/weight_total * log(it->weight/weight_total);
    }
    return -entropy;
}

/*
更新地图:这里的更新地图，只是为了可视化。因为真正的地图都存储在粒子里面。
这里会拿一个权值最大的粒子的地图发布出来.

得到权值最大的粒子，然后遍历这个粒子的整个轨迹，根据轨迹上记录的信息来进行建图
然后把得到的地图发布出去
这个函数被laserCallback()调用，每次addScan()成功了，就会调用这个函数来生成地图，并发布出去

*/
void SlamGMapping::updateMap(const sensor_msgs::LaserScan& scan)
{
    ROS_DEBUG("Update map");
    boost::mutex::scoped_lock map_lock (map_mutex_);
    GMapping::ScanMatcher matcher;

    /*设置scanmatcher的各个参数*/
    matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),
            gsp_laser_->getPose());

    matcher.setlaserMaxRange(maxRange_);
    matcher.setusableRange(maxUrange_);
    matcher.setgenerateMap(true);

    /*得到权值最高的粒子*/
    GMapping::GridSlamProcessor::Particle best =
            gsp_->getParticles()[gsp_->getBestParticleIndex()];

    //发布位姿的熵
    std_msgs::Float64 entropy;
    entropy.data = computePoseEntropy();
    if(entropy.data > 0.0)
        entropy_publisher_.publish(entropy);


    //如果没有地图 则初始化一个地图
    if(!got_map_)
    {
        map_.map.info.resolution = delta_;
        map_.map.info.origin.position.x = 0.0;
        map_.map.info.origin.position.y = 0.0;
        map_.map.info.origin.position.z = 0.0;
        map_.map.info.origin.orientation.x = 0.0;
        map_.map.info.origin.orientation.y = 0.0;
        map_.map.info.origin.orientation.z = 0.0;
        map_.map.info.origin.orientation.w = 1.0;
    }

    /*地图的中点*/
    GMapping::Point center;
    center.x=(xmin_ + xmax_) / 2.0;
    center.y=(ymin_ + ymax_) / 2.0;

    /*初始化一个scanmatcherMap 创建一个地图*/
    GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_,
                                  delta_);

    /*更新地图*/
    //遍历粒子的整条轨迹 按照轨迹上各个节点存储的信息来重新计算一个地图
    ROS_DEBUG("Trajectory tree:");
    for(GMapping::GridSlamProcessor::TNode* n = best.node;n;n = n->parent)
    {
        ROS_DEBUG("  %.3f %.3f %.3f",
                  n->pose.x,
                  n->pose.y,
                  n->pose.theta);
        if(!n->reading)
        {
            ROS_DEBUG("Reading is NULL");
            continue;
        }
        //进行地图更新
        //matcher.invalidateActiveArea();
        //matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
        //matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
        matcher.registerScan(smap, n->pose, &(n->reading->m_dists[0]));
    }

    // the map may have expanded, so resize ros message as well
    // 扩充地图的大小
    if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY())
    {

        // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
        //       so we must obtain the bounding box in a different way
        GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
        GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
        xmin_ = wmin.x; ymin_ = wmin.y;
        xmax_ = wmax.x; ymax_ = wmax.y;

        ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
                  xmin_, ymin_, xmax_, ymax_);

        map_.map.info.width = smap.getMapSizeX();
        map_.map.info.height = smap.getMapSizeY();
        map_.map.info.origin.position.x = xmin_;
        map_.map.info.origin.position.y = ymin_;
        map_.map.data.resize(map_.map.info.width * map_.map.info.height);

        ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
    }

    //根据地图的信息计算出来各个点的情况:occ、free、noinformation
    //这样对地图进行标记主要是方便用RVIZ显示出来
    for(int x=0; x < smap.getMapSizeX(); x++)
    {
        for(int y=0; y < smap.getMapSizeY(); y++)
        {
            /// @todo Sort out the unknown vs. free vs. obstacle thresholding
            /// 得到.xy被占用的概率
            GMapping::IntPoint p(x, y);
            double occ=smap.cell(p);
            assert(occ <= 1.0);

            //unknown
            if(occ < 0)
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = GMAPPING_UNKNOWN;

            //占用
            else if(occ > occ_thresh_)
            {
                //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = GMAPPING_OCC;
            }

            //freespace
            else
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = GMAPPING_FREE;
        }
    }

    //到了这一步，肯定是有地图了。
    got_map_ = true;

    //make sure to set the header information on the map
    //把计算出来的地图发布出去
    map_.map.header.stamp = ros::Time::now();
    map_.map.header.frame_id = tf_.resolve( map_frame_ );

    sst_.publish(map_.map);
    sstm_.publish(map_.map.info);
}


/*
 * 地图服务的回调函数
 * 别的函数可以向gmapping请求地图。
 * gmapping结果到这个请求之后，就会把地图发回去。
*/
bool SlamGMapping::mapCallback(nav_msgs::GetMap::Request  &req,
                               nav_msgs::GetMap::Response &res)
{
    boost::mutex::scoped_lock map_lock (map_mutex_);
    if(got_map_ && map_.map.info.width && map_.map.info.height)
    {
        res = map_;
        return true;
    }
    else
        return false;
}

/*发布map到odom的转换关系*/
void SlamGMapping::publishTransform()
{
    map_to_odom_mutex_.lock();
    ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
    tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_, odom_frame_));
    map_to_odom_mutex_.unlock();
}
