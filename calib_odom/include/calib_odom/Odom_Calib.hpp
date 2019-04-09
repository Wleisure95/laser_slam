#ifndef ODOM_CALIB_H
#define ODOM_CALIB_H
#include <iostream>
#include <eigen3/Eigen/Jacobi>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Householder>
class OdomCalib
{

public:
    OdomCalib(){
        data_len = 0;
        now_len = 0;
    }

   // virtual ~OdomCalib();
    void Set_data_len(int len);
    bool Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan);
    Eigen::Matrix3d Solve();
    bool is_full();
    void set_data_zero();

private:
    Eigen::MatrixXd  A;
    Eigen::VectorXd  b;
    int data_len,now_len;

};

#endif
