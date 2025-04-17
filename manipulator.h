#ifndef MANIPULATOR_H
#define MANIPULATOR_H

// 매크로 충돌 방지
#ifdef B1
#undef B1
#endif

#include "C:\Users\fairy\Documents\Arduino\libraries\Eigen\Dense"
// #include <Eigen/Dense>
#include <cmath>
#include <Arduino.h>
#include "parameters.h"
#include "ctrl_wrapper.h"

// Namespace 정의
namespace Manipulator {


  // 벡터 및 행렬 선언
  extern Eigen::Vector2d q;      // [q1, q2]
  extern Eigen::Vector2d qdot;   // [qdot1, qdot2]
  extern Eigen::Vector2d qd;     // [qd1, qd2]
  extern Eigen::Vector2d qdotd;   // [qdotd1, qdotd2]
  extern Eigen::Vector2d x;      // [x1, x2]
  extern Eigen::Vector2d xdot;   // [xdot1, xdot2]
  extern Eigen::Vector2d xd;     // [xd1, xd2]
  extern Eigen::Vector2d xdotd;   // [xdotd1, xdotd2]
  extern Eigen::Vector2d tau;   // [tau1, tau2]
  extern Eigen::Vector2d u;      // [u1, u2]
  extern Eigen::Vector2d u_sat;

  extern Eigen::Matrix2d M, C;   // 질량 행렬 및 코리올리스 행렬
  extern Eigen::Vector2d G;      // 중력 벡터
  extern Eigen::Vector2d F;      // friction
  extern Eigen::Matrix2d J;
  extern Eigen::Matrix2d Jdot;
  extern Eigen::Matrix2d Jinv;    

  extern Eigen::Vector3d Fraw;  // end effector [Fx,Fy,Fz]
  extern Eigen::Vector2d _Fraw;
  extern Eigen::Vector3d Fbias;  // end effector [Fx,Fy,Fz]
  extern Eigen::Vector3d Traw; // end effector [Tx,Ty,Tz]
  extern Eigen::Vector2d Fext;  // end effector [Fy,Fz]
  extern Eigen::Vector3d Tbias;
  extern Eigen::Matrix2d Md, Dd, Kd, L; 
  extern float cutoff;

  // 함수 선언
  Eigen::Matrix2d massMatrix(float q2);
  Eigen::Matrix2d coriolisMatrix(float q2, float qdot1, float qdot2);
  Eigen::Vector2d gravityVector(float q1, float q2);
  Eigen::Vector2d frictionVector(float qdot1, float qdot2, 
                                const Eigen::Vector2d& u, 
                                const Eigen::Vector2d& G, 
                                const Eigen::Matrix2d& J, 
                                const Eigen::Vector2d& Fext);
  Eigen::Matrix2d JacobianInv(const Eigen::Vector2d& q);
  Eigen::Matrix2d Jacobian(const Eigen::Vector2d& q); 
  Eigen::Matrix2d JacobianDot(const Eigen::Vector2d& q, const Eigen::Vector2d& qdot); 
  Eigen::Vector2d qddot(const Eigen::Vector2d& q, const Eigen::Vector2d& qdot, 
                                const Eigen::Matrix2d& M, const Eigen::Matrix2d& C, 
                                const Eigen::Vector2d& G, const Eigen::Vector2d& u, 
                                const Eigen::Matrix2d& J, const Eigen::Vector2d& Fext);
  Eigen::Matrix2d TransformRot(const Eigen::Vector2d& q);
  Eigen::Matrix2d TransformTool2End(const Eigen::Vector2d& q);    


  void computeDYN(Eigen::Matrix2d& M, Eigen::Matrix2d& C, Eigen::Vector2d& G,
                  const Eigen::Vector2d& q, const Eigen::Vector2d& qdot);
  void computeFK(Eigen::Vector2d& x, Eigen::Vector2d& xdot, 
                  const Eigen::Vector2d& q, const Eigen::Vector2d& qdot);
  void computeJ(Eigen::Matrix2d& J, Eigen::Matrix2d& Jdot, Eigen::Matrix2d& Jinv, 
                  const Eigen::Vector2d& q, const Eigen::Vector2d& qdot);
  void computeIK(Eigen::Vector2d& q, 
                 const Eigen::Vector2d& x);
  void computeIK(Eigen::Vector2d& q, Eigen::Vector2d& qdot, 
                 const Eigen::Vector2d& x, const Eigen::Vector2d& xdot);

  void TransformFT(Eigen::Vector2d& Fext, const Eigen::Vector2d& q, const Eigen::Vector3d& Fraw) ;
  void updateDynamics(const float dt);

  // Low-pass Filter (float)
  float LowPassFilter1(float& filtered_data, const float data, float cutoff, float dt);

  // High-pass Filter (float)
  float HighPassFilter1(float& filtered_data, const float data, float cutoff, float dt);

  // Low-pass Filter (Eigen::Vector2d)
  Eigen::Vector2d LowPassFilter2(Eigen::Vector2d& filtered_data, const Eigen::Vector2d& data, float cutoff, float dt);

  // High-pass Filter (Eigen::Vector2d)
  Eigen::Vector2d HighPassFilter2(Eigen::Vector2d& filtered_data, const Eigen::Vector2d& data, float cutoff, float dt);

  Eigen::Vector2d saturation(Eigen::Vector2d& u);
  
  // 초기화 함수
  void initializeManipulator();

} // namespace Manipulator

#endif
