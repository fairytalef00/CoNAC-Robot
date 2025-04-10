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

  // double Ttraj = 16.0;               // 경로 생성 시간 (초)
  double Ttraj = 16.0;               // 경로 생성 시간 (초)
  double Thome = 5.0;               // 홈 위치 경로 생성 시간 (초)
  double elapsed_time = 0.0;        // 경과 시간
  bool is_trajectory_active = false; // 경로 활성화 상태
  double cycle_count = 0.0;
  double MAX_CYCLE_NUM = 2.0; // 최대 사이클 수
  // double MAX_CYCLE_NUM = 5.0; // 최대 사이클 수
  double rep_count = 0.0;  // 에피소드 카운터; MS
  double MAX_REP_NUM =2;

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
      if (!is_trajectory_active) return;

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
          // 경로 완료
          elapsed_time = 0.0;
          is_trajectory_active = false;
          CONTROL_FLAG = STANDBY;
      }
  }

  void generateReference1(double dt) {
      using namespace Manipulator;
      if (!is_trajectory_active) return;

      if (cycle_count >= MAX_CYCLE_NUM) {
          is_trajectory_active = false; 
          elapsed_time = 0.0;

          CONTROL_FLAG = STANDBY;

          return;
      }
      elapsed_time += dt;
      double t = elapsed_time;
      q0 << -M_PI/2, 0;
      qd1 << M_PI/4, -M_PI/2;      
      qd2 << 3*M_PI/4, -3*M_PI/4;
      qd3 << -M_PI/4, M_PI/2;

      qd1 = q0+(qd1-q0) * (1+cycle_count)/(MAX_CYCLE_NUM);
      qd2 = q0+(qd2-q0) * (1+cycle_count)/(MAX_CYCLE_NUM);
      qd3 = q0+(qd3-q0) * (1+cycle_count)/(MAX_CYCLE_NUM);


      if (t < Ttraj) {
        if (t < Ttraj/4) {
          poly_filter(q0,qd1,Ttraj/4,t); // q0 -> qd1
        } else if (t < Ttraj*2/4) {
          poly_filter(qd1,qd2,Ttraj/4,t-Ttraj/4); // qd1 -> q2
        } else if (t < Ttraj*3/4) {
          poly_filter(qd2,qd3,Ttraj/4,t-Ttraj*2/4); // qd2 -> qd3
        } else {
          poly_filter(qd3,q0,Ttraj/4,t-Ttraj*3/4); // qd3 -> q0
        }
      } else {
        elapsed_time = 0.0;
        cycle_count = cycle_count + 1; 
      }
  }

  // void generateReference2(double dt) {
  //   if (rep_count >= MAX_REP_NUM)
  //   {
  //     is_trajectory_active = false; 
  //     elapsed_time = 0.0;
  //     rep_count = 0.0;
  //     cycle_count = 0.0;
  //     CONTROL_FLAG = REST2;
  //     return;
  //   }
    
  //   if (cycle_count >= MAX_CYCLE_NUM) {
  //     elapsed_time = 0.0;
  //     cycle_count = 0.0;
  //     rep_count += 1; 
  //   }

  //   generateReference1(dt);
  // }

  void generateReference3(double dt) {
    using namespace Manipulator;
    if (!is_trajectory_active) return;

    if (cycle_count >= MAX_CYCLE_NUM) {
        is_trajectory_active = false; 
        elapsed_time = 0.0;

        if (CONTROL_FLAG==4){
          CONTROL_FLAG = REST1;
        } else if (CONTROL_FLAG==5){
          CONTROL_FLAG = REST2;
        }

        return;
    }
    elapsed_time += dt;
    double t = elapsed_time;
    q0 << -M_PI/2, 0;
    qd1 << M_PI/4, -M_PI/2;      
    qd2 << 3*M_PI/4, -3*M_PI/4;
    qd3 << -M_PI/4, M_PI/2;

    qd1 = q0+(qd1-q0) * (1+cycle_count)/(MAX_CYCLE_NUM);
    qd2 = q0+(qd2-q0) * (1+cycle_count)/(MAX_CYCLE_NUM);
    qd3 = q0+(qd3-q0) * (1+cycle_count)/(MAX_CYCLE_NUM);


    if (t < Ttraj) {
      if (t < Ttraj/4) {
        poly_filter(q0,qd1,Ttraj/4,t); // q0 -> qd1
      } else if (t < Ttraj*2/4) {
        poly_filter(qd1,qd2,Ttraj/4,t-Ttraj/4); // qd1 -> q2
      } else if (t < Ttraj*3/4) {
        poly_filter(qd2,qd3,Ttraj/4,t-Ttraj*2/4); // qd2 -> qd3
      } else {
        poly_filter(qd3,q0,Ttraj/4,t-Ttraj*3/4); // qd3 -> q0
      }
    } else {
      elapsed_time = 0.0;
      cycle_count = cycle_count + 1; 
    }
  }

  void poly_filter(const Eigen::Vector2d& q0, const Eigen::Vector2d& qd, double T, double t){

    Eigen::Vector2d a0 = q0;
    Eigen::Vector2d a1 = Eigen::Vector2d::Zero();
    Eigen::Vector2d a2 = Eigen::Vector2d::Zero();

    Eigen::Vector2d a3 = 10 * (qd - a0) / (T * T * T);
    Eigen::Vector2d a4 = -15 * (qd - q0) / (T * T * T * T);
    Eigen::Vector2d a5 = 6 * (qd - a0) / (T * T * T * T * T);

    // 5차 다항식 계산
    r = a0 + a1 * t + a2 * (t * t) + a3 * t * t * t + a4 * t * t * t * t + a5 * t * t * t * t * t;
    rdot = a1 + 2.0 * a2 * t + 3.0 * a3 * t * t + 4.0 * a4 * t * t * t + 5.0 * a5 * t * t * t * t;

  }

  void poly_filter_home(const Eigen::Vector2d& q0, const Eigen::Vector2d& qd, double T, double t) {
    // 행렬 A 정의 (6x6)
    Eigen::MatrixXd A(6, 6);
    A << 1, 0, 0, 0, 0, 0,                // r(0) = q0
         0, 1, 0, 0, 0, 0,                // rdot(0) = qdot0
         0, 0, 2, 0, 0, 0,                // rddot(0) = 0
         1, T, T*T, T*T*T, T*T*T*T, T*T*T*T*T,  // r(T) = qd
         0, 1, 2*T, 3*T*T, 4*T*T*T, 5*T*T*T*T,  // rdot(T) = 0
         0, 0, 2, 6*T, 12*T*T, 20*T*T*T;       // rddot(T) = 0

    // 초기 및 경계 조건 벡터 b 정의 (6x2)
    Eigen::MatrixXd b(6, 2);
    b << q0.transpose(),                 // r(0) = q0
         qdot0.transpose(),              // rdot(0) = qdot0
         Eigen::Vector2d::Zero().transpose(), // rddot(0) = 0
         qd.transpose(),                 // r(T) = qd
         Eigen::Vector2d::Zero().transpose(), // rdot(T) = 0
         Eigen::Vector2d::Zero().transpose(); // rddot(T) = 0

    // 계수 계산 (A * coeffs = b)
    Eigen::MatrixXd coeffs = A.colPivHouseholderQr().solve(b);

    // 계수 추출
    Eigen::Vector2d a0 = coeffs.row(0);
    Eigen::Vector2d a1 = coeffs.row(1);
    Eigen::Vector2d a2 = coeffs.row(2);
    Eigen::Vector2d a3 = coeffs.row(3);
    Eigen::Vector2d a4 = coeffs.row(4);
    Eigen::Vector2d a5 = coeffs.row(5);

    // 5차 다항식 계산
    r = a0 + a1 * t + a2 * t * t + a3 * t * t * t + a4 * t * t * t * t + a5 * t * t * t * t * t;
    rdot = a1 + 2.0 * a2 * t + 3.0 * a3 * t * t + 4.0 * a4 * t * t * t + 5.0 * a5 * t * t * t * t;
    rddot = 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t * t + 20.0 * a5 * t * t * t;
  }

  void traj_flag() {
    using namespace Manipulator;

    initializeTrajectory();
    cycle_count = 0;
    rep_count = 0.0;
    elapsed_time = 0.0;
  
    is_trajectory_active = true;
  }
}
