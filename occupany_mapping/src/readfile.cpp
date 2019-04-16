#include "readfile.h"

#include <fstream>
#include <boost/algorithm/string.hpp>

template <class Type>
Type stringToNum(const std::string& str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;

    return num;
}

typedef std::string::size_type string_size;
std::vector<std::string> splitString(const std::string &s, const std::string &seperator)
{
    std::vector<std::string> result;
    string_size i = 0;

    while(i != s.size())
    {
        //找到字符串中首个不等于分隔符的字母；
        int flag = 0;
        while(i != s.size() && flag == 0)
        {
            flag = 1;
            for(string_size k = 0; k < seperator.size(); k++)
            {
                if(s[i] == seperator[k])
                {
                    i++;
                    flag = 0;
                    break;
                }
            }
        }

        //找到又一个分隔符，将两个分隔符之间的字符串取出；
        flag = 0;
        string_size j = i;
        while(j != s.size() && flag == 0)
        {
            for(string_size k = 0; k < seperator.size(); k++)
            {
                if(s[j] == seperator[k])
                {
                    flag = 1;
                    break;
                }
            }
            if(flag == 0)
                j++;
        }

        if(i != j)
        {
            result.push_back(s.substr(i, j-i));
            i = j;
        }
    }
    return result;
}



//读取机器人的位姿信息．
void ReadPoseInformation(const std::string path,std::vector<Eigen::Vector3d>& poses)
{
    std::ifstream fin(path.c_str());
    if(fin.is_open() == false)
    {
        std::cout <<"Read File Failed!!!"<<std::endl;
        return ;
    }

    int cnt = 0;

    std::string line;
    while(std::getline(fin,line))
    {
        cnt++;
        std::vector<std::string> results;

        results = splitString(line,",");

        double x = stringToNum<double>(results[0]);
        double y = stringToNum<double>(results[1]);
        double theta = stringToNum<double>(results[2]);

        Eigen::Vector3d pose(x,y,theta);

        poses.push_back(pose);

        if(cnt >= READ_DATA_NUMBER)
            break;

    }
    fin.close();

    std::cout <<"Read Pose Good!!!"<<std::endl;
}


void ReadLaserScanInformation(const std::string anglePath,
                              const std::string laserPath,
                              std::vector< GeneralLaserScan >& laserscans)
{
    //////////////////////////////////////////读取角度信息///////////////////////////////////////////////
    std::ifstream fin(anglePath.c_str());
    if(fin.is_open() == false)
    {
        std::cout <<"Read Angle File Failed!!!"<<std::endl;
        return ;
    }

    GeneralLaserScan tmpGeneralLaserScan;

    //读取角度信息
    int cnt = 0;
    std::string line;
    std::getline(fin,line);
    std::vector<std::string> results;
    results = splitString(line,",");
    for(int i = 0; i < results.size();i++)
    {
        double anglei = stringToNum<double>(results[i]);
        tmpGeneralLaserScan.angle_readings.push_back(anglei);
    }
    fin.close();

    std::cout <<"Read Angle good:"<<tmpGeneralLaserScan.angle_readings.size()<<std::endl;

    //////////////////////////////////////////读取激光信息///////////////////////////////////////////////

    fin.open(laserPath.c_str());
    if(fin.is_open() == false)
    {
        std::cout <<"Read Scan File Failed!!!"<<std::endl;
        return ;
    }

    laserscans.clear();

    int xx;
    while(std::getline(fin,line))
    {
        //读取一行，每一行进行分割
        std::vector<std::string> results;
        results = splitString(line,",");

        cnt++;
        xx = results.size();

        tmpGeneralLaserScan.range_readings.clear();
        for(int i = 0; i < results.size();i++)
        {
            double rangei = stringToNum<double>(results[i]);
            tmpGeneralLaserScan.range_readings.push_back(rangei);
        }

        laserscans.push_back(tmpGeneralLaserScan);

        if(cnt >= READ_DATA_NUMBER)
            break;
    }

    std::cout <<"XX:"<<xx<<std::endl;

    fin.close();

    std::cout <<"Read Laser Scans Good!!!!"<<std::endl;
}







