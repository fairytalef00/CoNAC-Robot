#include "manipulator.h"

namespace Manipulator {

  Eigen::Vector2d q;     // [q1, q2]
  Eigen::Vector2d qd;    // [qd1, qd2]
  Eigen::Vector2d qdot;  // [qdot1, qdot2]
  Eigen::Vector2d qdotd; // [qdotd1, qdotd2]
  Eigen::Vector2d u;     // [u1, u2]
  Eigen::Vector2d u_sat;

  Eigen::Vector2d tau;   // [tau1, tau2]
  Eigen::Matrix2d M;
  Eigen::Matrix2d C;
  Eigen::Vector2d G;
  Eigen::Matrix2d J;
  Eigen::Vector2d F;
  Eigen::Matrix2d Jdot;
  Eigen::Matrix2d Jinv;    
  Eigen::Vector2d x;  // end effector [y,z]
  Eigen::Vector2d xdot;  // end effector [ydot,zdot]
  Eigen::Vector2d xd;  // end effector [y,z]
  Eigen::Vector2d xdotd;  // end effector [ydot,zdot]
  Eigen::Vector3d Fraw;  // end effector [Fx,Fy,Fz]
  Eigen::Vector2d _Fraw;
  Eigen::Vector3d Fbias;  // end effector [Fx,Fy,Fz]
  Eigen::Vector3d Traw; // end effector [Tx,Ty,Tz]
  Eigen::Vector3d Tbias;  // end effector [Fx,Fy,Fz]
  Eigen::Vector2d Fext;  // end effector [Fy,Fz]

  Eigen::Matrix2d Md, Kd, Dd, L; // control gain

  Eigen::Vector2d r;      // 경로 위치
  Eigen::Vector2d rdot;   // 경로 속도c:\Users\fairy\Desktop\matlab_prac\manipulator\model\codegen\lib\forward_kinematics_3\inverse_kinematics_3.c
  Eigen::Vector2d rddot;  // 경로 가속도

  float cutoff = 5.0; 

  // 질량 행렬 (M)
  Eigen::Matrix2d massMatrix(float q2) {
      Eigen::Matrix2d M;
      float c2 = cos(q2);
      M(0, 0) = I1 + I2 + lc1*lc1*m1 + l1*l1*m2 + lc2*lc2*m2  + 2*c2*l1*lc2*m2;
      M(0, 1) = I2 + lc2*lc2*m2 + c2*l1*lc2*m2;
      M(1, 0) = M(0, 1);
      M(1, 1) = I2 + lc2 * lc2 * m2;
      return M;
  }

  // 코리올리스 행렬 (C)
  Eigen::Matrix2d coriolisMatrix(float q2, float qdot1, float qdot2) {
      Eigen::Matrix2d C;
      C(0, 0) = - l1 * lc2 * m2 * sin(q2) * qdot2;
      C(0, 1) = -l1 * lc2 * m2 * sin(q2) * (qdot1 + qdot2);
      C(1, 0) = l1 * lc2 * m2 * sin(q2) * qdot1;
      C(1, 1) = 0;
      return C;
  }

  // 중력 벡터 (G)
  Eigen::Vector2d gravityVector(float q1, float q2) {
      Eigen::Vector2d G;
      float c12 = cos(q1+q2);
      float c1 = cos(q1);
      G(0) = g * (c12 * lc2 * m2 + c1 * (lc1 * m1 + l1 * m2));
      G(1) = c12 * g * lc2 * m2;
      return G;
  }

  // Friction Model (F)
  Eigen::Vector2d frictionVector(float qdot1, float qdot2, 
                                const Eigen::Vector2d& u, 
                                const Eigen::Vector2d& G, 
                                const Eigen::Matrix2d& J, 
                                const Eigen::Vector2d& Fext) {
      float Fc1 = 0.35;  // Coulomb friction for joint 1
      float Fc2 = 0.2;  // Coulomb friction for joint 2
      float Fv1 = 0.2; // Viscous friction coefficient for joint 1
      float Fv2 = 0.1; // Viscous friction coefficient for joint 2
      float Fs1 = 0.7;   // Static friction for joint 1
      float Fs2 = 0.4;   // Static friction for joint 2

      float stiction_threshold = 1e-3; // 정지 마찰을 고려할 속도 임계값

      // 순수 외력 계산: u - G - J^T * Fext
      Eigen::Vector2d effective_torque = u - G;

      auto friction = [&](float qdot, float Fc, float Fv, float Fs, float effective_torque) {
          float friction_force = 0.0f;

          if (fabs(qdot) < stiction_threshold) {
              // 속도가 거의 0일 때
              if (fabs(effective_torque) < Fs) {
                  // 순수 외력이 정지 마찰력보다 작으면 움직이지 않음
                  friction_force = effective_torque;
              } else {
                  // 정지 마찰력 극복 시, 운동 마찰로 전환
                  if (effective_torque >= 0) {
                      friction_force = Fc + Fv * qdot;
                  } else {
                      friction_force = -Fc + Fv * qdot;
                  }
              }
          } else {
              // 일반적인 Coulomb + Viscous 마찰 모델 적용
              if (qdot >= 0) {
                  friction_force = Fc + Fv * qdot;
              } else {
                  friction_force = -Fc + Fv * qdot;
              }
          }

          return friction_force;
      };

      return Eigen::Vector2d(
          friction(qdot1, Fc1, Fv1, Fs1, effective_torque(0)),
          friction(qdot2, Fc2, Fv2, Fs2, effective_torque(1))
      );
  }


  Eigen::Matrix2d Jacobian(const Eigen::Vector2d& q) {
      Eigen::Matrix2d J;
      float s1 = sin(q(0));
      float s12 = sin(q(0)+q(1));
      float c1 = cos(q(0));
      float c12 = cos(q(0)+q(1));

      J(0, 0) = - l1 * s1 - l2 * s12;
      J(0, 1) = - l2 * s12;
      J(1, 0) = c1 * l1 + c12 *l2;
      J(1, 1) = c12 * l2;
      return J;
  }

  Eigen::Matrix2d JacobianInv(const Eigen::Vector2d& q) {
      Eigen::Matrix2d Jinv;
      float s1 = sin(q(0));
      float s2 = sin(q(1));
      float s12 = sin(q(0)+q(1));
      float c1 = cos(q(0));
      float c12 = cos(q(0)+q(1));

      // 정규 Jacobian 역 계산
      Jinv(0, 0) = (l2 * c12) / (l1 * l2 * s2);
      Jinv(0, 1) = (l2 * s12) / (l1 * l2 * s2);
      Jinv(1, 0) = -(c1 * l1 + c12 * l2) / (l1 * l2 * s2);
      Jinv(1, 1) = (l1 * s1 + l2 * s12) / (l1 * l2 * s2);
      return Jinv;
  }

  Eigen::Matrix2d JacobianDot(const Eigen::Vector2d& q, const Eigen::Vector2d& qdot) {
      Eigen::Matrix2d Jdot;
      float s1 = sin(q(0));
      float s12 = sin(q(0)+q(1));
      float c1 = cos(q(0));
      float c12 = cos(q(0)+q(1));

      Jdot(0, 0) = -c1 * l1 * qdot(0) - c12 * l2 * (qdot(0) + qdot(1));
      Jdot(0, 1) = -c12 * l2 * (qdot(0) + qdot(1));
      Jdot(1, 0) = -l1 * qdot(0) * s1 - l2 * (qdot(0) + qdot(1)) * s12;
      Jdot(1, 1) = -l2 * (qdot(0) + qdot(1)) * s12;

      return Jdot;
  }

  Eigen::Vector2d qddot(const Eigen::Vector2d& q, const Eigen::Vector2d& qdot, 
                                const Eigen::Matrix2d& M, const Eigen::Matrix2d& C, 
                                const Eigen::Vector2d& G, const Eigen::Vector2d& u, 
                                const Eigen::Matrix2d& J, const Eigen::Vector2d& Fext) {
      return M.inverse() * (-C * qdot - G + u + J.transpose() * Fext);
  }


  void computeDYN(Eigen::Matrix2d& M, Eigen::Matrix2d& C, Eigen::Vector2d& G,
                  const Eigen::Vector2d& q, const Eigen::Vector2d& qdot) {
      M = massMatrix(q(1));
      C = coriolisMatrix(q(1), qdot(0), qdot(1));
      G = gravityVector(q(0), q(1));
  }

  void computeFK(Eigen::Vector2d& x, Eigen::Vector2d& xdot, const Eigen::Vector2d& q, const Eigen::Vector2d& qdot) {
      x(0) = l1 * cos(q(0)) + l2 * cos(q(0)+q(1));
      x(1) = l1 * sin(q(0)) + l2 * sin(q(0)+q(1));

      J = Jacobian(q);
      xdot = J * qdot; 
  }

  void computeJ(Eigen::Matrix2d& J, Eigen::Matrix2d& Jdot, Eigen::Matrix2d& Jinv, 
                  const Eigen::Vector2d& q, const Eigen::Vector2d& qdot) {
      J = Jacobian(q);
      Jinv = JacobianInv(q);
      Jdot = JacobianDot(q, qdot);
  }

  void computeIK(Eigen::Vector2d& q, const Eigen::Vector2d& x) {
      float dist2 = x(0) * x(0) + x(1) * x(1);  // 목표점까지의 거리의 제곱
      float l_sum2 = (l1 + l2) * (l1 + l2);  // 최대 도달 가능 거리의 제곱
      float l_diff2 = (l1 - l2) * (l1 - l2);  // 최소 도달 가능 거리의 제곱

      // IK 해가 존재하지 않으면 계산을 수행하지 않음
      if (dist2 > l_sum2 || dist2 < l_diff2) {
          return;  // 그냥 함수 종료
      }

      // IK 해가 존재하면 계산 수행
      float c2 = (-l2 * l2 - l1 * l1 + dist2) / (2 * l1 * l2);
      float s2 = -sqrt(1 - c2 * c2);  // sin 값을 계산하여 방향 정보 보정 (-)
      float q2 = atan2(s2, c2);
      float q1 = atan2(x(1),x(0)) - atan2(l2 * s2, l1 + l2 * c2);

      q(0) = q1;
      q(1) = q2;
  }


  void computeIK(Eigen::Vector2d& q, Eigen::Vector2d& qdot, const Eigen::Vector2d& x, const Eigen::Vector2d& xdot) {
      computeIK(q, x);
      
      // qdot 추가 계산
      Eigen::Matrix2d Jinv = JacobianInv(q);
      qdot = Jinv * xdot;
  }

  Eigen::Matrix2d TransformRot(const Eigen::Vector2d& q) {
      Eigen::Matrix2d RotM;
      float s12 = sin(q(0)+q(1));
      float c12 = cos(q(0)+q(1));

      RotM(0, 0) = c12;
      RotM(0, 1) = -s12;
      RotM(1, 0) = s12;
      RotM(1, 1) = c12;

      return RotM;
  }

  Eigen::Matrix2d TransformTool2End(const Eigen::Vector2d& q) {
      Eigen::Matrix2d T2EM;

      T2EM(0, 0) = cos(-M_PI/2);
      T2EM(0, 1) = -sin(-M_PI/2);
      T2EM(1, 0) = sin(-M_PI/2);
      T2EM(1, 1) = cos(-M_PI/2);

      return T2EM;
  }

  void TransformFT(Eigen::Vector2d& Fext, const Eigen::Vector2d& q, const Eigen::Vector3d& Fraw) {
      Eigen::Vector2d _Fraw;
      Eigen::Matrix2d RotM;
      Eigen::Matrix2d T2EM;

      RotM = TransformRot(q);
      T2EM = TransformTool2End(q);

      _Fraw(0) = Fraw(0);
      _Fraw(1) = Fraw(2);

      Fext = RotM * T2EM *_Fraw;

  }

  // Low-pass Filter (float)
  float LowPassFilter1(float& filtered_data, const float data, float cutoff, float dt) {
      float alpha = cutoff / (cutoff + (1.0 / dt));
      filtered_data = alpha * data + (1.0 - alpha) * filtered_data;
      return filtered_data;
  }

  // High-pass Filter (float)
  float HighPassFilter1(float& filtered_data, const float data, float cutoff, float dt) {
      float alpha = cutoff / (cutoff + (1.0 / dt));
      float prev_filtered = filtered_data;
      filtered_data = alpha * data + (1.0 - alpha) * filtered_data;
      return data - filtered_data;
  }

  // Low-pass Filter (Eigen::Vector2d)
  Eigen::Vector2d LowPassFilter2(Eigen::Vector2d& filtered_data, const Eigen::Vector2d& data, float cutoff, float dt) {
      float alpha = cutoff / (cutoff + (1.0 / dt));
      filtered_data = alpha * data + (1.0 - alpha) * filtered_data;
      return filtered_data;
  }

  // High-pass Filter (Eigen::Vector2d)
  Eigen::Vector2d HighPassFilter2(Eigen::Vector2d& filtered_data, const Eigen::Vector2d& data, float cutoff, float dt) {
      float alpha = cutoff / (cutoff + (1.0 / dt));
      Eigen::Vector2d prev_filtered = filtered_data;
      filtered_data = alpha * data + (1.0 - alpha) * filtered_data;
      return data - filtered_data;
  }

  // Saturation
  Eigen::Vector2d saturation(Eigen::Vector2d& u) {
      using namespace CoNAC_Params;
      Eigen::Vector2d u_sat;
      double norm_u = u.norm();

      // Apply norm constraint (u_ball)
      if (norm_u > u_ball) {
          u_sat = u / norm_u * u_ball;
      } else {
        u_sat = u;
      }
      // Apply constraint on u(0) (u1_max)
      if (u_sat(0) > u1_max) {
          u_sat(1) = u_sat(1) * u1_max / fabs(u_sat(0));
          u_sat(0) = u1_max;
      } else if (u_sat(0) < -u1_max) {
          u_sat(1) = u_sat(1) * u1_max / fabs(u_sat(0));
          u_sat(0) = -u1_max;
      }
 
    // Apply constraint on u(1) (u2_max)
    if (u_sat(1) > u2_max) {
        u_sat(0) = u_sat(0) * u2_max / fabs(u_sat(1));
        u_sat(1) = u2_max;
    } else if (u_sat(1) < -u2_max) {
        u_sat(0) = u_sat(0) * u2_max / fabs(u_sat(1));
        u_sat(1) = -u2_max;
    }
        
      return u_sat;
  }

//   // Saturation
//   Eigen::Vector2d saturation(Eigen::Vector2d& u) {
//     Eigen::Vector2d u_sat = u;

//     if (u.norm() > u_ball){
//         u_sat = u / u.norm(u) * u_ball;

//         // case 1
//         if (u_sat(0) > u1_max){
//             u_sat(1) = u_sat(1) * u1_max / fabs(u_sat(0));
//             u_sat(0) = u1_max;
//         }
//         else if (u_sat(0) < - u1_max){
//             u_sat(1) = u_sat(1) * u1_max / fabs(u_sat(0));
//             u_sat(0) = - u1_max;
//         }

//         // case 2
//         else if (fabs(u_sat(0)) < u1_max){
//             u_sat = u / u.norm() * u_ball;
//         }
//     } 
//     else if (u.norm() < u_ball){
//         // case 3
//         if (u_sat(0) > u1_max){
//             u_sat(1) = u_sat(1) * u1_max / fabs(u_sat(0));
//             u_sat(0) = u1_max;
//         }
//         else if (u_sat(0) < - u1_max){
//             u_sat(1) = u_sat(1) * u1_max / fabs(u_sat(0));
//             u_sat(0) = -u1_max;
//         }
//         // case 4
//         else if (fabs(u_sat(0)) <u1_max){
//             u_sat = u;
//         }
//     }
     
//     return u_sat;
//   }

  void updateDynamics(const float dt) {
      // 질량, 코리올리스, 중력 계산
      computeDYN(M, C, G, q, qdot);
      F = frictionVector(qdot(0), qdot(1), u_sat, G, J, Fext);

      Eigen::Vector2d qddot = M.inverse() * (- C * qdot - G - F + u_sat);
      qdot += qddot * dt;
      q += qdot * dt;

    }

  // 초기화 함수 구현
  void initializeManipulator() {
      q.setZero();
      qd.setZero();
      qdot.setZero();
      qdotd.setZero();
      u.setZero();
      u_sat.setZero();
      M.setZero();
      C.setZero();
      G.setZero();
      J.setZero();
      F.setZero();
      Jdot.setZero();
      Jinv.setZero();
      x.setZero();
      xdot.setZero();
      Fext.setZero();
      Fraw.setZero();
      _Fraw.setZero();
      Fbias.setZero();
      Traw.setZero();
      computeFK(xd, xdotd, q, qdot);

      Md.setZero();
      Dd.setZero();
      Kd.setZero();
      L.setZero();
  }

} // namespace Manipulator
