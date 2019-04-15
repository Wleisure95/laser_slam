#ifndef READFILE_H
#define READFILE_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <iostream>

#include "gaussian_newton.h"


void ReadVertexInformation(const std::string path,std::vector<Eigen::Vector3d>& nodes);
void ReadEdgesInformation(const std::string path,std::vector<Edge>& edges);








#endif
