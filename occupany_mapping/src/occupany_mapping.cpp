#include "occupany_mapping.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/Path.h"

#include <tf/transform_broadcaster.h>


/**
 * Increments all the grid cells from (x0, y0) to (x1, y1);
 * //不包含(x1,y1)
 * 2D画线算法　来进行计算两个点之间的grid cell
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 */
std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1)
{
  GridIndex tmpIndex;
  std::vector<GridIndex> gridIndexVector;

  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep)
  {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1)
  {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  int deltaX = x1 - x0;
  int deltaY = abs(y1 - y0);
  int error = 0;
  int ystep;
  int y = y0;

  if (y0 < y1)
  {
    ystep = 1;
  }
  else
  {
    ystep = -1;
  }

  int pointX;
  int pointY;
  for (int x = x0; x <= x1; x++)
  {
    if (steep)
    {
      pointX = y;
      pointY = x;
    }
    else
    {
      pointX = x;
      pointY = y;
    }

    error += deltaY;

    if (2 * error >= deltaX)
    {
      y += ystep;
      error -= deltaX;
    }

    //不包含最后一个点．
    if(pointX == x1 && pointY == y1) continue;

    //保存所有的点
    tmpIndex.SetIndex(pointX,pointY);

    gridIndexVector.push_back(tmpIndex);
  }

  return gridIndexVector;
}

void SetMapParams(void )
{
   mapParams.width = 900;
   mapParams.height = 900;
   mapParams.resolution = 0.04;

   mapParams.log_free = -1;
   mapParams.log_occ = 2;

   mapParams.origin_x = 0.0;
   mapParams.origin_y = 0.0;

   //地图的原点，在地图的正中间
   mapParams.offset_x = 700;
   mapParams.offset_y = 600;

   pMap = new unsigned char[mapParams.width*mapParams.height];

   //初始化为50
   for(int i = 0; i < mapParams.width * mapParams.height;i++)
        pMap[i] = 50;
}


//从世界坐标系转换到栅格坐标系
GridIndex ConvertWorld2GridIndex(double x,double y)
{
    GridIndex index;

    index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) + mapParams.offset_x;
    index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) + mapParams.offset_y;

    return index;
}

int GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.x + index.y * mapParams.width;
}


//判断index是否有效
bool isValidGridIndex(GridIndex index)
{
    if(index.x >= 0 && index.x < mapParams.width && index.y >= 0 && index.y < mapParams.height)
        return true;

    return false;
}

void DestoryMap()
{
    if(pMap != NULL)
        delete pMap;
}

//
void OccupanyMapping(std::vector<GeneralLaserScan>& scans,std::vector<Eigen::Vector3d>& robot_poses)
{
    std::cout <<"Scans Size:"<<scans.size()<<std::endl;
    std::cout <<"Poses Size:"<<robot_poses.size()<<std::endl;

    //枚举所有的激光雷达数据
    for(int i = 0; i < scans.size();i++)
    {
        GeneralLaserScan scan = scans[i];
        Eigen::Vector3d robotPose = robot_poses[i];

        //机器人的下标
        GridIndex robotIndex = ConvertWorld2GridIndex(robotPose(0),robotPose(1));

        for(int id = 0; id < scan.range_readings.size();id++)
        {
            double dist = scan.range_readings[id];
            double angle = scan.angle_readings[id];

            if(std::isinf(dist) || std::isnan(dist)) continue;

            double theta = robotPose(2);

            //雷达坐标系下的坐标
            double laser_x =  dist * cos(theta + angle);
            double laser_y = -dist * sin(theta + angle); //激光数据的Y轴是反向的。数据有关，特例


            //世界坐标系下的坐标--激光机器人进行转换

            double world_x = laser_x + robotPose(0);
            double world_y = laser_y + robotPose(1);

            //转换到地图坐标系
            GridIndex mapIndex = ConvertWorld2GridIndex(world_x,world_y);

            //去除超过地图的点．
            if(isValidGridIndex(mapIndex) == false)continue;

            if(isValidGridIndex(robotIndex) == false)
            {
                std::cout <<"Error,This should not happen"<<std::endl;
                continue;
            }

            //得到所有的被激光通过的index，并且更新栅格
            std::vector<GridIndex> freeIndex = TraceLine(robotIndex.x,robotIndex.y,mapIndex.x,mapIndex.y);

            for(int k = 0; k < freeIndex.size();k++)
            {
                GridIndex tmpIndex = freeIndex[k];
                int linearIndex = GridIndexToLinearIndex(tmpIndex);

                int data = pMap[linearIndex];
                data += mapParams.log_free;
                if(data < 0)
                    data = 0;

                pMap[linearIndex] = data;

            }

            //更新被击中的点．
            int tmpIndex = GridIndexToLinearIndex(mapIndex);
            int data = pMap[tmpIndex];
            data += mapParams.log_occ;
            if(data > 100)
                data = 100;
            pMap[tmpIndex] = data;
        }
    }
}


//发布地图．
void PublishMap(ros::Publisher& map_pub)
{
    nav_msgs::OccupancyGrid rosMap;

    rosMap.info.resolution = mapParams.resolution;
    rosMap.info.origin.position.x = 0.0;
    rosMap.info.origin.position.y = 0.0;
    rosMap.info.origin.position.z = 0.0;
    rosMap.info.origin.orientation.x = 0.0;
    rosMap.info.origin.orientation.y = 0.0;
    rosMap.info.origin.orientation.z = 0.0;
    rosMap.info.origin.orientation.w = 1.0;

    rosMap.info.origin.position.x = mapParams.origin_x;
    rosMap.info.origin.position.y = mapParams.origin_y;
    rosMap.info.width = mapParams.width;
    rosMap.info.height = mapParams.height;
    rosMap.data.resize(rosMap.info.width * rosMap.info.height);

    //0~100
    int cnt0,cnt1,cnt2;
    cnt0 = cnt1 = cnt2 = 100;
    for(int i = 0; i < mapParams.width * mapParams.height;i++)
    {
       if(pMap[i] == 50)
       {
           cnt0++;
           rosMap.data[i] = -1.0;
       }
       else if(pMap[i] < 50)
       {
           cnt1++;
           rosMap.data[i] = 0;

           rosMap.data[i] = pMap[i];
       }
       else if(pMap[i] > 50)
       {
           cnt2++;
           rosMap.data[i] = 100;

           rosMap.data[i] = pMap[i];
       }
    }

    std::cout <<"Unknown:"<<cnt0<<std::endl;
    std::cout <<"Free:"<<cnt1<<std::endl;
    std::cout <<"Occu:"<<cnt2<<std::endl;

    rosMap.header.stamp = ros::Time::now();
    rosMap.header.frame_id = "map";

    map_pub.publish(rosMap);
}

void PubChampionScan(std::vector<GeneralLaserScan>& scans,std::vector<Eigen::Vector3d>& robot_poses,
                     ros::Publisher& ros_pub)
{
    const int xxx = 750;

    sensor_msgs::PointCloud scanPoints;
    scanPoints.header.frame_id="map";
    scanPoints.header.stamp = ros::Time::now();

    geometry_msgs::Point32 tmpPt;

    //枚举所有的激光雷达数据
    for(int i = 0; i < scans.size();i++)
    {
        GeneralLaserScan scan = scans[i];
        Eigen::Vector3d robotPose = robot_poses[i];

        if(i >= xxx)break;

        //机器人的下标
        GridIndex robotIndex = ConvertWorld2GridIndex(robotPose(0),robotPose(1));

        for(int id = 0; id < scan.range_readings.size();id++)
        {
            double dist = scan.range_readings[id];
            double angle = scan.angle_readings[id];

            if(std::isinf(dist) || std::isnan(dist)) continue;

            //雷达坐标系下的坐标
            double laser_x = dist * cos(angle);
            double laser_y = dist * sin(angle);

            //世界坐标系下的坐标--激光机器人进行转换
            double theta = robotPose(2);
            double world_x = cos(theta) * laser_x - sin(theta) * laser_y + robotPose(0) + mapParams.offset_x * mapParams.resolution;
            double world_y = -(sin(theta) * laser_x + cos(theta) * laser_y) + robotPose(1) + mapParams.offset_y * mapParams.resolution;

            tmpPt.x = world_x;
            tmpPt.y = world_y;
            tmpPt.z = 0.0;
            scanPoints.points.push_back(tmpPt);
        }
    }

    ros_pub.publish(scanPoints);
}
void pubpath(std::vector<Eigen::Vector3d>& robot_poses,ros::Publisher& ros_pub)
{
  nav_msgs::Path v_path;
  v_path.header.frame_id="map";
  v_path.header.stamp=ros::Time::now();

  geometry_msgs::PoseStamped tmpPose;
  tmpPose.header.stamp=ros::Time::now();
  tmpPose.header.frame_id="map";

  for(int i=0;i<robot_poses.size();i++)
  {
    Eigen::Vector3d pose = robot_poses[i];

    tmpPose.pose.position.x = pose(0)+mapParams.offset_x * mapParams.resolution;
    tmpPose.pose.position.y = pose(1)+mapParams.offset_y * mapParams.resolution;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
    tmpPose.pose.orientation.x = goal_quat.x;
    tmpPose.pose.orientation.y = goal_quat.y;
    tmpPose.pose.orientation.z = goal_quat.z;
    tmpPose.pose.orientation.w = goal_quat.w;

    v_path.poses.push_back(tmpPose);
  }

  ros_pub.publish(v_path);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "OccupanyMapping");

    ros::NodeHandle nodeHandler;

    ros::Publisher mapPub = nodeHandler.advertise<nav_msgs::OccupancyGrid>("laser_map",1,true);
    ros::Publisher laserPub = nodeHandler.advertise<sensor_msgs::PointCloud>("scan_point",1,true);
    ros::Publisher odomPub = nodeHandler.advertise<nav_msgs::Path>("path",1,true);

    std::vector<Eigen::Vector3d> robotPoses;
    std::vector<GeneralLaserScan> generalLaserScans;

    std::string basePath = "/home/leisure/all_ws/laser_ws/src/laser_slam/occupany_mapping/data";

    std::string posePath= basePath + "/pose.txt";
    std::string anglePath = basePath + "/scanAngles.txt";
    std::string scanPath = basePath + "/ranges.txt";

    //读取数据
    ReadPoseInformation(posePath,robotPoses);

    ReadLaserScanInformation(anglePath,
                             scanPath,
                             generalLaserScans);

    //设置地图信息
    SetMapParams();

    OccupanyMapping(generalLaserScans,robotPoses);

    PubChampionScan(generalLaserScans,robotPoses,laserPub);

    PublishMap(mapPub);
    pubpath(robotPoses,odomPub);

    ros::spin();

    DestoryMap();

    std::cout <<"Release Memory!!"<<std::endl;

}




