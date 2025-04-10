#ifndef CTRL_WRAPPER_H
#define CTRL_WRAPPER_H
#include "manipulator.h" //
#include "trajectory.h" //
#include <stdlib.h>



namespace CoNAC_Params {
    extern double u_ball;      
    extern double w1;  
    extern double w2;        
    extern double beta[8];    
    extern double th_max[3];    
    extern double B[4];       
    extern double Lambda_arr[4]; 
    extern double A_zeta[4];
    extern double B_zeta[4];
    extern double u1_max;
    extern double u2_max;
}

namespace CoNAC_Data {
    extern double th_arr[58];
    extern double lbd_arr[8];
    extern double Vn_arr[3];
    extern double out_arr[2]; 
    extern Eigen::Vector3d Vn;
    extern Eigen::Matrix<double, 8, 1> lbd;
    extern bool CoNAC_initialized;
    extern double zeta_arr[2];
}

void ctrl_wrapper(const int CONTROL_NUM, const Eigen::Vector2d& q, const Eigen::Vector2d& qdot,
                  const Eigen::Vector2d& r, const Eigen::Vector2d& rdot,
                  Eigen::Vector2d& u, 
                  Eigen::Matrix<double, 8, 1>& lbd, Eigen::Vector3d& Vn);

void initializeCoNAC();


#endif // CTRL_WRAPPER_H
