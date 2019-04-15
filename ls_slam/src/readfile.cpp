#include "readfile.h"

#include <fstream>
#include <boost/algorithm/string.hpp>

#include "gaussian_newton.h"


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


void ReadVertexInformation(const std::string path,std::vector<Eigen::Vector3d>& nodes)
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

        results = splitString(line," ");

        int id = stringToNum<int>(results[1]);

        double x = stringToNum<double>(results[2]);
        double y = stringToNum<double>(results[3]);
        double theta = stringToNum<double>(results[4]);

        Eigen::Vector3d pose(x,y,theta);

        nodes.push_back(pose);
    }
    fin.close();
}


void ReadEdgesInformation(const std::string path,std::vector<Edge>& edges)
{
    std::ifstream fin(path.c_str());
    if(fin.is_open() == false)
    {
        std::cout <<"Read File Failed!!!"<<std::endl;
        return ;
    }

    edges.clear();

    int cnt = 0;
    std::string line;
    while(std::getline(fin,line))
    {
        std::vector<std::string> results;
        results = splitString(line," ");

        cnt++;
        int xi = stringToNum<int>(results[1]);
        int xj = stringToNum<int>(results[2]);

        double dx = stringToNum<double>(results[3]);
        double dy = stringToNum<double>(results[4]);
        double dtheta = stringToNum<double>(results[5]);

        double inf_xx  = stringToNum<double>(results[6]);
        double inf_xy  = stringToNum<double>(results[7]);
        double inf_yy  = stringToNum<double>(results[8]);
        double inf_tt  = stringToNum<double>(results[9]);
        double inf_xt  = stringToNum<double>(results[10]);
        double inf_yt  = stringToNum<double>(results[11]);

        Edge tmpEdge;
        tmpEdge.xi = xi;
        tmpEdge.xj = xj;

        tmpEdge.measurement = Eigen::Vector3d(dx,dy,dtheta);

        tmpEdge.infoMatrix << inf_xx,inf_xy,inf_xt,
                              inf_xy,inf_yy,inf_yt,
                              inf_xt,inf_yt,inf_tt;


        edges.push_back(tmpEdge);
    }

    std::cout <<"Edges:"<<cnt<<std::endl;

    fin.close();
}






