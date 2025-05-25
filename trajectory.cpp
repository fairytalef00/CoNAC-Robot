#include "trajectory.h"
#include "motor_control.h"

namespace Trajectory {
  // 변수 정의
  Eigen::Vector2d r;      // 경로 위치
  Eigen::Vector2d rdot;   // 경로 속도
  Eigen::Vector2d rddot;  // 경로 가속도
  Eigen::Vector2d last_xd; // 이전 목표 위치
  Eigen::Vector2d x0;       // 시작 위치 x
  Eigen::Vector2d xdot0;
  Eigen::Vector2d q0;       // 시작 위치 q
  Eigen::Vector2d qdot0;
  Eigen::Vector2d qd1;
  Eigen::Vector2d qd2;
  Eigen::Vector2d qd3;
  Eigen::Vector2d qd4;
  Eigen::Vector2d qd5;
  Eigen::Vector2d qd6;
  Eigen::Vector2d qd7;

  // double Ttraj = 16.0;               // 경로 생성 시간 (초)
  double Ttraj = 21.0;               // 경로 생성 시간 (초)
  double Thome = 5.0;               // 홈 위치 경로 생성 시간 (초)
  double elapsed_time = 0.0;        // 경과 시간

  // 초기화 함수
  void initializeTrajectory() {
      using namespace Manipulator;
      r = q;
      rdot.setZero();
      rddot.setZero();

      q0 = q;
      qdot0.setZero();
      elapsed_time = 0.0;
  }

  // Reference Generator 함수
  void generateReference0(double dt) {
      using namespace Manipulator;

      // 시간 경과 업데이트
      elapsed_time += dt;
      if (elapsed_time <= Thome)
      {
        double t = elapsed_time;

        qd << -M_PI/2, 0;
        // poly_filter_home(q0, qd, Thome, t);

        // linear trajectory
        r = q0 + (qd - q0) * (t/Thome);
        rdot = (qdotd - qdot0) / Thome;
        rddot.setZero();

      } else {
        CONTROL_FLAG = STANDBY;
      }
  }

  void generateReference1(double dt) {
    // 1. qd0 -> qd1 
    // 2. idle(0.5s)

    // //episode 1
    // 3. qd1 -> qd2 
    // 4. qd2 -> qd3
    // 5. qd3 -> qd2
    // 6. qd2 -> qd1

    // //episode 2
    // 3. qd1 -> qd2
    // 4. qd2 -> qd3
    // 5. qd3 -> qd2
    // 6. qd2 -> qd1

    // 7. idle(0.5s)
    // 8. qd1 -> qd0 
      
    using namespace Manipulator;

    elapsed_time += dt;
    double t = elapsed_time;

    // q0 << -M_PI/2, 0;
    qd1 << -M_PI/3, M_PI/3;
    qd2 << M_PI/4, -M_PI/2;      
    qd3 << -M_PI/4, M_PI/4;
    double Tidle = 2.0;         // idle time
    double Tinit = 4.0;   // init time
    double Ttotal = Tinit + Tidle + Ttraj + Tidle + Ttraj + Tidle + Tinit; // Total time 

    if (t < Ttotal) {
          if (t < Tinit) {
              // 1. qd0 -> qd1
              poly_filter(q0, qd1, Tinit, t);
          } else if (t < Tinit + Tidle) {
              // 2. Idle at qd1
              r = qd1;
              rdot.setZero();
              rddot.setZero();

          } else if (t < Tinit + Tidle + Ttraj) {
              // Episode 1: qd1 -> qd2 -> qd3 -> qd2 -> qd1
              double t_episode = t - (Tinit + Tidle);
              if (t_episode < Ttraj/4) {
                  poly_filter(qd1, qd2, Ttraj/4, t_episode); // qd1 -> qd2
              } else if (t_episode < 2 * Ttraj/4) {
                  poly_filter(qd2, qd3, Ttraj/4, t_episode - Ttraj/4); // qd2 -> qd3
              } else if (t_episode < 3 * Ttraj/4) {
                  poly_filter(qd3, qd2, Ttraj/4, t_episode - 2 * Ttraj/4); // qd3 -> qd2
              if (t_episode < Ttraj/4) {
                  poly_filter(qd1, qd2, Ttraj/4, t_episode); // qd1 -> qd2
              } else if (t_episode < 2 * Ttraj/4) {
                  poly_filter(qd2, qd3, Ttraj/4, t_episode - Ttraj/4); // qd2 -> qd3
              } else if (t_episode < 3 * Ttraj/4) {
                  poly_filter(qd3, qd2, Ttraj/4, t_episode - 2 * Ttraj/4); // qd3 -> qd2
              } else {
                  poly_filter(qd2, qd1, Ttraj/4, t_episode - 3 * Ttraj/4); // qd2 -> qd1
                  poly_filter(qd2, qd1, Ttraj/4, t_episode - 3 * Ttraj/4); // qd2 -> qd1
              }

          } else if (t < Tinit + Tidle + Ttraj + Ttraj) {
              // Episode 2: qd1 -> qd2 -> qd3 -> qd2 -> qd1
              double t_episode = t - (Tinit + Tidle + Ttraj);
              if (t_episode < Ttraj/4) {
                  poly_filter(qd1, qd2, Ttraj/4, t_episode); // qd1 -> qd2
              } else if (t_episode < 2 * Ttraj/4) {
                  poly_filter(qd2, qd3, Ttraj/4, t_episode - Ttraj/4); // qd2 -> qd3
              } else if (t_episode < 3 * Ttraj/4) {
                  poly_filter(qd3, qd2, Ttraj/4, t_episode - 2 * Ttraj/4); // qd3 -> qd2
              if (t_episode < Ttraj/4) {
                  poly_filter(qd1, qd2, Ttraj/4, t_episode); // qd1 -> qd2
              } else if (t_episode < 2 * Ttraj/4) {
                  poly_filter(qd2, qd3, Ttraj/4, t_episode - Ttraj/4); // qd2 -> qd3
              } else if (t_episode < 3 * Ttraj/4) {
                  poly_filter(qd3, qd2, Ttraj/4, t_episode - 2 * Ttraj/4); // qd3 -> qd2
              } else {
                  poly_filter(qd2, qd1, Ttraj/4, t_episode - 3 * Ttraj/4); // qd2 -> qd1
                  poly_filter(qd2, qd1, Ttraj/4, t_episode - 3 * Ttraj/4); // qd2 -> qd1
              }

          } else if (t < Tinit + Tidle + Ttraj + Ttraj + Tidle) {
          } else if (t < Tinit + Tidle + Ttraj + Ttraj + Tidle) {
              // 7. Idle at qd1
              r = qd1;
              rdot.setZero();
              rddot.setZero();

          } else {
              // 8. qd1 -> qd0
              double t_return = t - (Tinit + Tidle + Ttraj + Ttraj + Tidle);
              double t_return = t - (Tinit + Tidle + Ttraj + Ttraj + Tidle);
              poly_filter(qd1, q0, Tinit, t_return);
          }
      } else {
        CONTROL_FLAG = STANDBY;
      }
  }

  void generateReference2(double dt) {
    // idle
    // 1. qd0 -> qd1 
    // idle
    // 2. qd1 -> qd2 
    // idle
    // 3. qd2 -> qd3
    // idle
    // 4. qd3 -> qd4
    // idle
    // 5. qd4 -> qd0
    // idle
      
    using namespace Manipulator;

    elapsed_time += dt;
    double t = elapsed_time;

    // q0 << -M_PI/2, 0;
    qd1 << -M_PI/4, M_PI/2;
    qd2 << M_PI/4, -M_PI/2;      
    qd3 << -M_PI/4, M_PI/2;
    qd4 << M_PI/3, -M_PI/3;

    double Tidle = 2.0;         // idle time

    double Ttraj1 = Ttraj/5;
    double Ttraj2 = Ttraj/5;
    double Ttraj3 = Ttraj/5;
    double Ttraj4 = Ttraj/5;
    double Ttraj5 = Ttraj/5;

    double Ttotal = Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle + Ttraj5 + Tidle; // Total time

    if (t < Ttotal) {
        if (t < Tidle) {
            r = q0;
            rdot.setZero();
            rddot.setZero();

        } else if (t < Tidle + Ttraj1) {
            double t_traj1 = t - Tidle;
            poly_filter(q0, qd1, Ttraj1, t_traj1);

        } else if (t< Tidle + Ttraj1 + Tidle) {
            r = qd1;
            rdot.setZero();
            rddot.setZero();

        } else if (t< Tidle + Ttraj1 + Tidle + Ttraj2) {
            double t_traj2 = t - (Tidle + Ttraj1 + Tidle);
            poly_filter(qd1, qd2, Ttraj2, t_traj2); // qd1 -> qd2
            
        } else if (t< Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle) {
            r = qd2;
            rdot.setZero();
            rddot.setZero();

        } else if (t< Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3) {
            double t_traj3 = t - (Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle);
            poly_filter(qd2, qd3, Ttraj3, t_traj3); // qd2 -> qd3

        } else if (t< Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle) {
            r = qd3;
            rdot.setZero();
            rddot.setZero();

        } else if (t< Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4) {
            double t_traj4 = t - (Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle);
            poly_filter(qd3, qd4, Ttraj4, t_traj4); // qd3 -> qd4

        } else if (t< Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle) {
            r = qd4;
            rdot.setZero();
            rddot.setZero();
        
        } else if (t < Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle + Ttraj5) {
            double t_traj5 = t - (Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle);
            poly_filter(qd4, q0, Ttraj5, t_traj5); // qd4 -> qd0
        } else if (t < Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle + Ttraj5 + Tidle) {
            r = q0;
            rdot.setZero();
            rddot.setZero();
        }
    } else {
        CONTROL_FLAG = STANDBY;
    }
  }
  void generateReference3(double dt) {
    using namespace Manipulator;
    
    elapsed_time += dt;
    double t = elapsed_time;
    q0 << -M_PI/2, 0;
    qd1 << 0, M_PI/2;
    qd2 << M_PI/2, -M_PI/2;      
    qd3 << -M_PI/2, M_PI/2;
    qd4 = qd2;
    qd5 = qd3;
    qd6 = qd2;
    qd7 = q0;

    double Tidle = 2.0;         // idle time

    double Ttraj1 = Ttraj/7;
    double Ttraj2 = Ttraj/7;
    double Ttraj3 = Ttraj/7;
    double Ttraj4 = Ttraj/7;
    double Ttraj5 = Ttraj/7;
    double Ttraj6 = Ttraj/7;
    double Ttraj7 = Ttraj/7;

    double Ttotal = Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle + Ttraj5 + Tidle + Ttraj6 + Tidle + Ttraj7 + Tidle; // Total time

    if (t < Ttotal) {
        if (t < Tidle) {
        r = q0;
        rdot.setZero();
        rddot.setZero();

        } else if (t < Tidle + Ttraj1) {
        double t_traj1 = t - Tidle;
        poly_filter(q0, qd1, Ttraj1, t_traj1);

        } else if (t< Tidle + Ttraj1 + Tidle) {
        r = qd1;
        rdot.setZero();
        rddot.setZero();

        } else if (t< Tidle + Ttraj1 + Tidle + Ttraj2) {
        double t_traj2 = t - (Tidle + Ttraj1 + Tidle);
        poly_filter(qd1, qd2, Ttraj2, t_traj2); // qd1 -> qd2
        
        } else if (t< Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle) {
            r = qd2;
            rdot.setZero();
            rddot.setZero();

        } else if (t< Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3) {
            double t_traj3 = t - (Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle);
            poly_filter(qd2, qd3, Ttraj3, t_traj3); // qd2 -> qd3

        } else if (t< Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle) {
            r = qd3;
            rdot.setZero();
            rddot.setZero();

        } else if (t< Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4) {
            double t_traj4 = t - (Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle);
            poly_filter(qd3, qd4, Ttraj4, t_traj4); // qd3 -> qd4

        } else if (t< Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle) {
            r = qd4;
            rdot.setZero();
            rddot.setZero();
        
        } else if (t < Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle + Ttraj5) {
            double t_traj5 = t - (Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle);
            poly_filter(qd4, qd5, Ttraj5, t_traj5); // qd4 -> qd0

        } else if (t < Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle + Ttraj5 + Tidle) {
            r = qd5;
            rdot.setZero();
            rddot.setZero();

        } else if (t < Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle + Ttraj5 + Tidle + Ttraj6) {
            double t_traj6 = t - (Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle + Ttraj5 + Tidle);
            poly_filter(qd5, qd6, Ttraj6, t_traj6); // qd5 -> qd6

        } else if (t < Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle + Ttraj5 + Tidle + Ttraj6 + Tidle) {
            r = qd6;
            rdot.setZero();
            rddot.setZero();

        } else if (t < Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle + Ttraj5 + Tidle + Ttraj6 + Tidle + Ttraj7) {
            double t_traj7 = t - (Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle + Ttraj5 + Tidle + Ttraj6+Tidle);
            poly_filter(qd6, qd7, Ttraj7, t_traj7); // qd6 -> qd7

        } else if (t < Tidle + Ttraj1 + Tidle + Ttraj2 + Tidle + Ttraj3 + Tidle + Ttraj4 + Tidle + Ttraj5 + Tidle + Ttraj6 + Tidle + Ttraj7 + Tidle) {
            r = qd7;
            rdot.setZero();
            rddot.setZero();
        }
    } else {
        CONTROL_FLAG = STANDBY;
    }
  }

//   void poly_filter(const Eigen::Vector2d& q0, const Eigen::Vector2d& qd, double T, double t){

//     Eigen::Vector2d a0 = q0;
//     Eigen::Vector2d a1 = Eigen::Vector2d::Zero();
//     Eigen::Vector2d a2 = Eigen::Vector2d::Zero();

//     Eigen::Vector2d a3 = 10 * (qd - a0) / (T * T * T);
//     Eigen::Vector2d a4 = -15 * (qd - q0) / (T * T * T * T);
//     Eigen::Vector2d a5 = 6 * (qd - a0) / (T * T * T * T * T);

//     // 5차 다항식 계산
//     r = a0 + a1 * t + a2 * (t * t) + a3 * t * t * t + a4 * t * t * t * t + a5 * t * t * t * t * t;
//     rdot = a1 + 2.0 * a2 * t + 3.0 * a3 * t * t + 4.0 * a4 * t * t * t + 5.0 * a5 * t * t * t * t;

//   }

  void poly_filter(const Eigen::Vector2d& q0, const Eigen::Vector2d& qd, double T, double t) {
    // // 행렬 A 정의 (6x6)
    // Eigen::MatrixXd A(6, 6);
    // A << 1, 0, 0, 0, 0, 0,                // r(0) = q0
    //      0, 1, 0, 0, 0, 0,                // rdot(0) = qdot0
    //      0, 0, 2, 0, 0, 0,                // rddot(0) = 0
    //      1, T, T*T, T*T*T, T*T*T*T, T*T*T*T*T,  // r(T) = qd
    //      0, 1, 2*T, 3*T*T, 4*T*T*T, 5*T*T*T*T,  // rdot(T) = 0
    //      0, 0, 2, 6*T, 12*T*T, 20*T*T*T;       // rddot(T) = 0

    // // 초기 및 경계 조건 벡터 b 정의 (6x2)
    // Eigen::MatrixXd b(6, 2);
    // b << q0.transpose(),                 // r(0) = q0
    //      qdot0.transpose(),              // rdot(0) = qdot0
    //      Eigen::Vector2d::Zero().transpose(), // rddot(0) = 0
    //      qd.transpose(),                 // r(T) = qd
    //      Eigen::Vector2d::Zero().transpose(), // rdot(T) = 0
    //      Eigen::Vector2d::Zero().transpose(); // rddot(T) = 0

    // // 계수 계산 (A * coeffs = b)
    // Eigen::MatrixXd coeffs = A.colPivHouseholderQr().solve(b);

    // // 계수 추출
    // Eigen::Vector2d a0 = coeffs.row(0);
    // Eigen::Vector2d a1 = coeffs.row(1);
    // Eigen::Vector2d a2 = coeffs.row(2);
    // Eigen::Vector2d a3 = coeffs.row(3);
    // Eigen::Vector2d a4 = coeffs.row(4);
    // Eigen::Vector2d a5 = coeffs.row(5);
    // 초기 조건: q0, qdot0, rddot(0)=0
    // 최종 조건: qd, rdot(T)=0, rddot(T)=0

    Eigen::Vector2d a0 = q0;
    Eigen::Vector2d a1 = Eigen::Vector2d::Zero();
    Eigen::Vector2d a2 = Eigen::Vector2d::Zero();

    Eigen::Vector2d delta = qd - q0;

    Eigen::Vector2d a3 = (10.0 * delta) / (T * T * T);
    Eigen::Vector2d a4 = (-15.0 * delta) / (T * T * T * T);
    Eigen::Vector2d a5 = (6.0 * delta) / (T * T * T * T * T);


    // 5차 다항식 계산
    r = a0 + a1 * t + a2 * t * t + a3 * t * t * t + a4 * t * t * t * t + a5 * t * t * t * t * t;
    rdot = a1 + 2.0 * a2 * t + 3.0 * a3 * t * t + 4.0 * a4 * t * t * t + 5.0 * a5 * t * t * t * t;
    rddot = 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t * t + 20.0 * a5 * t * t * t;
  }
}
