#ifndef OCCUPANY_MAPPING_H
#define OCCUPANY_MAPPING_H

#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <eigen3/Eigen/Core>

#include "readfile.h"

typedef struct gridindex_
{
    int x;
    int y;

    void SetIndex(int x_,int y_)
    {
        x  = x_;
        y  = y_;
    }
}GridIndex;



typedef struct map_params
{
    double log_occ,log_free;
    double resolution;
    double origin_x,origin_y;
    int height,width;
    int offset_x,offset_y;
}MapParams;


MapParams mapParams;

unsigned char* pMap;






#endif

