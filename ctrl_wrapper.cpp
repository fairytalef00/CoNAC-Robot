#include "ctrl_wrapper.h"
#include "parameters.h"
#include <CoNAC.h>

namespace CoNAC_Params {
    double u_ball = 12.5f;      
    double alp1 = 1;        
    double alp2 = 0.8;        
    double beta[8] = {1e-3, 1e-3, 1e-3, 10, 10, 10, 10, 10};    
    double th_max[3] = {11, 12, 13};    
    double B[4] = {1, 0, 0, 1};  
    double Lambda_arr[4] = {5, 0, 0, 5};
    double A_zeta[4] = {-10, 0, 0, -10}; // Aux. System Matrix 
    double B_zeta[4] = {100, 0, 0, 100}; // Aux. Input Matrix
    double u1_max = 12.2f;
    double u2_max = 11.2f;
}

namespace CoNAC_Data {
    double th_arr[58];           // 신경망 가중치 배열
    double lbd_arr[8];        // 라그랑주 승수
    Eigen::Matrix<double, 8, 1> lbd;
    double Vn_arr[3];         // 신경망 가중치 노름 값
    Eigen::Vector3d Vn;
    double out_arr[2];
    bool CoNAC_initialized = false;

    double zeta_arr[2]; // Aux. State

}

// ----------------- 변환 함수 -----------------
inline Eigen::Vector2d Double2Vector2d(const double arr[2]) {
    return Eigen::Vector2d(arr[0], arr[1]);
}

inline Eigen::Vector3d Double2Vector3d(const double arr[3]) {
    return Eigen::Vector3d(arr[0], arr[1], arr[2]);
}

inline Eigen::Vector4d Double2Vector4d(const double arr[4]) {
    return Eigen::Vector4d(arr[0], arr[1], arr[2], arr[3]);
}

inline Eigen::Matrix<double, 8, 1> Double2Vector8d(const double arr[8]) {
    Eigen::Matrix<double, 8, 1> vec;
    for (int i = 0; i < 8; ++i) vec(i) = arr[i];
    return vec;
}

inline Eigen::Matrix<double, 178, 1> Double2Vector178d(const double arr[178]) {
    Eigen::Matrix<double, 178, 1> vec;
    for (int i = 0; i < 178; ++i) vec(i) = arr[i];
    return vec;
}

inline void Vector2Double2d(const Eigen::Vector2d& vec, double arr[2]) {
    arr[0] = vec[0];
    arr[1] = vec[1];
}

inline void Vector2Double3d(const Eigen::Vector3d& vec, double arr[3]) {
    arr[0] = vec[0];
    arr[1] = vec[1];
    arr[2] = vec[2];
}

inline void Vector2Double4d(const Eigen::Vector4d& vec, double arr[4]) {
    arr[0] = vec[0];
    arr[1] = vec[1];
    arr[2] = vec[2];
    arr[3] = vec[3];
}

inline void Vector2Double8d(const Eigen::Matrix<double, 8, 1>& vec, double arr[8]) {
    for (int i = 0; i < 8; ++i) arr[i] = vec(i);
}

inline void Vector2Double178d(const Eigen::Matrix<double, 178, 1>& vec, double arr[178]) {
    for (int i = 0; i < 178; ++i) arr[i] = vec(i);
}

inline Eigen::Matrix2d Double2Matrix2d(const double arr[4]) {
    Eigen::Matrix2d M;
    M << arr[0], arr[1],
         arr[2], arr[3];
    return M;
}

inline void Matrix2Double4d(const Eigen::Matrix2d& mat, double arr[4]) {
    arr[0] = mat(0, 0);
    arr[1] = mat(0, 1);
    arr[2] = mat(1, 0);
    arr[3] = mat(1, 1);
}

           
void ctrl_wrapper(const int CONTROL_NUM, const Eigen::Vector2d& q, const Eigen::Vector2d& qdot,
                  const Eigen::Vector2d& r, const Eigen::Vector2d& rdot,
                  Eigen::Vector2d& u, 
                  Eigen::Matrix<double, 8, 1>& lbd, Eigen::Vector3d& Vn) {

    using namespace CoNAC_Params;
    using namespace CoNAC_Data;

    // 필요한 배열 선언 및 초기화
    double q_arr[2], qdot_arr[2], r_arr[2], rdot_arr[2];

    // Eigen → double 변환
    Vector2Double2d(q, q_arr);
    Vector2Double2d(qdot, qdot_arr);
    Vector2Double2d(r, r_arr);
    Vector2Double2d(rdot, rdot_arr);
    Vector2Double8d(lbd, lbd_arr);

   // Aux 함수 호출
    CoNAC(q_arr, r_arr, qdot_arr, 
        rdot_arr, th_arr, lbd_arr, zeta_arr,
        A_zeta, B_zeta, alp1, alp2, 
        ctrl_dt, Lambda_arr, th_max, 
        u_ball, beta, u1_max, u2_max, 
        CONTROL_NUM, out_arr, Vn_arr);
    
    // void CoNAC(const double x1[2], const double xd1[2], const double x2[2],
    //     const double xd2[2], double th[58], double lbd[8], double zeta[2],
    //     const double A_zeta[4], const double B_zeta[4], double alp1,double alp2,
    //     double ctrl_dt, const double Lambda[4], const double th_max[3],
    //     double u_ball, const double beta[8], double uMax1, double uMax2,
    //     double CONTROL_NUM, double out[2], double Vn[3])

    // double → Eigen 변환
    u   = Double2Vector2d(out_arr);
    Vn  = Double2Vector3d(Vn_arr);
    lbd = Double2Vector8d(lbd_arr);
}

void initializeCoNAC() {
    using namespace CoNAC_Data;

    // 배열 초기화
    for (int i = 0; i < 58; ++i) {
        th_arr[i] = 0.0;
    }
    for (int i = 0; i < 4; ++i) {
        lbd_arr[i] = 0.0;
    }
    for (int i = 0; i < 3; ++i) {
        Vn_arr[i] = 0.0;
    }
    for (int i = 0; i < 2; ++i) {
        out_arr[i] = 0.0;
    }
    for (int i = 0; i < 2; ++i) {
        zeta_arr[i] = 0.0;
    }

    lbd.setZero();
    Vn.setZero();

    std::srand(18);
    for (int i = 0; i < 58; ++i) {
        double random = static_cast<double>(std::rand()) / RAND_MAX;
        double val = (random - 0.5) * 1e-1;
        th_arr[i] = std::round(val * 1e7) / 1e7; 
    }

}
