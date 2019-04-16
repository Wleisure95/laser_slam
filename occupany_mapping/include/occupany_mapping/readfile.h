#ifndef READFILE_H
#define READFILE_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <iostream>


//max 3000
#define READ_DATA_NUMBER  3071

typedef struct general_laser_scan
{
    std::vector<double> range_readings;
    std::vector<double> angle_readings;
}GeneralLaserScan;


void ReadPoseInformation(const std::string path,std::vector<Eigen::Vector3d>& poses);
void ReadLaserScanInformation(const std::string anglePath,
                              const std::string laserPath,
                              std::vector< GeneralLaserScan >& laserscans);

#endif
