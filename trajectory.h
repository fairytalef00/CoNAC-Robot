#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "C:\Users\fairy\Documents\Arduino\libraries\Eigen\Dense"
#include "manipulator.h"
#include <cmath>

namespace Trajectory {

  // 경로 관련 변수
  extern Eigen::Vector2d r;      // 경로 위치
  extern Eigen::Vector2d rdot;   // 경로 속도
  extern Eigen::Vector2d rddot;  // 경로 가속도
  extern Eigen::Vector2d last_xd; // 이전 목표 위치
  extern Eigen::Vector2d x0;      // 초기 위치 x
  extern Eigen::Vector2d xdot0;
  extern Eigen::Vector2d q0;      // 초기 위치 q
  extern Eigen::Vector2d qdot0; 
  extern double Ttraj;           // 경로 생성 시간 (초)
  extern double rep_count;
  extern double cycle_count;

  // 함수 선언
  void initializeTrajectory();                         // 초기화
  void generateReference0(double dt);   
  void generateReference1(double dt);    
  void generateReference2(double dt);
  void generateReference3(double dt);           
  void poly_filter(const Eigen::Vector2d& q0, const Eigen::Vector2d& qd, double T, double t);
  void poly_filter_home(const Eigen::Vector2d& q0, const Eigen::Vector2d& qd, double T, double t);
  void traj_flag();
}

#endif
