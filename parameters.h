#ifndef PARAMETERS_H
#define PARAMETERS_H

// ✅ C++에서는 constexpr 사용, C에서는 static const 사용
#ifdef __cplusplus
#define PARAMETER_CONST constexpr
#else
#define PARAMETER_CONST static const
#endif

// ========== 시뮬레이션/제어 주기 관련 파라미터 ==========
extern unsigned int CONTROL_NUM; // 1: CoNAC, 2: Aux.

extern float t;                    // 현재 시간
extern unsigned long lastPrintTimeState;
extern unsigned long lastControlTimeState;
extern unsigned long lastPrintTimeFreq;
extern unsigned long lastPrintTimeLimit;


// ✅ 고정된 상수 값
PARAMETER_CONST float ctrl_dt = 0.004f;             // 제어 루프 주기 [s]
PARAMETER_CONST float update_dt  = 0.002f;       // state update 주기 [s]
PARAMETER_CONST float traj_dt = 0.004f;        // 궤적 루프 주기 [s]
PARAMETER_CONST float printperiod = 0.004f;     // 출력 주기 [s]

// ✅ 마이크로초 단위 주기 계산
PARAMETER_CONST unsigned long updatePeriodMicros  = static_cast<unsigned long>(update_dt  * 1e6);
PARAMETER_CONST unsigned long ctrlPeriodMicros = static_cast<unsigned long>(ctrl_dt * 1e6);
PARAMETER_CONST unsigned long trajPeriodMicros = static_cast<unsigned long>(traj_dt * 1e6);
PARAMETER_CONST unsigned long printPeriodMillis = static_cast<unsigned long>(printperiod * 1e3);
PARAMETER_CONST unsigned long printPeriodMicros = static_cast<unsigned long>(printperiod * 1e6);

// ✅ 모델 파라미터 정의 (C & C++ 공통 사용 가능)

// ** 링크 길이 **
PARAMETER_CONST double l1 = 0.2;                 // Link 1 길이
// PARAMETER_CONST double l2 = 0.15 + 0.01 + 0.02;  // Link 2 길이 (Aluminum + end plate + ft sensor)
PARAMETER_CONST double l2 = 0.2;  // Link 2 길이 motor2
// Alternative values:
// PARAMETER_CONST double l2 = 0.15 + 0.01;       // Link 2 길이 (Aluminum + end plate)

// ** 질량 (Mass) **
PARAMETER_CONST double m1 = 2.465;  // Link 1 질량 (Aluminum + bearing + bolt + motor2)
// PARAMETER_CONST double m2 = 1.093;  // Link 2 질량 (Aluminum + end plate + ft sensor)
PARAMETER_CONST double m2 = 2.465;  // Link 2 질량 + motor
// PARAMETER_CONST double m2 = 1.330;  // Link 2 질량 (O-O Aluminum)
// PARAMETER_CONST double m2 = 0.63024;  // Link 2 질량 (-O Aluminum)
// PARAMETER_CONST double m2 = 0.8703;   // Link 2 질량 (Aluminum + end plate)

// ** 관성 (Moment of Inertia) **
PARAMETER_CONST double I1 = 0.06911;  // Link 1 관성 + 9×9× 0.0001002
// PARAMETER_CONST double I2 = 0.01532;  // Link 2 관성 (Aluminum + end plate + ft sensor)
PARAMETER_CONST double I2 = 0.06911;  // Link 2 관성 + motor
// PARAMETER_CONST double I2 = 0.002245;  // Link 2 관성 (O-O Aluminum)
// PARAMETER_CONST double I2 = 0.0026897;  // Link 2 관성 (-O Aluminum)
// PARAMETER_CONST double I2 = 0.0085731;  // Link 2 관성 (Aluminum + end plate)

// ** 중력 가속도 **
PARAMETER_CONST double g = 9.81;  // 중력 가속도

// ** 질량 중심 거리 (Center of Mass) **
PARAMETER_CONST double lc1 = 0.13888;  // Link 1 질량중심까지 거리

// PARAMETER_CONST double lc2 = 0.08691;  // Link 2 질량중심까지 거리 (Aluminum + end plate + ft sensor)
PARAMETER_CONST double lc2 = 0.13888;  // Link 2 + motor
// PARAMETER_CONST double lc2 = 0.08658;  // Link 2 질량중심까지 거리 (O-O Aluminum)
// PARAMETER_CONST double lc2 = 0.03028;  // Link 2 질량중심까지 거리 (-O Aluminum)
// PARAMETER_CONST double lc2 = 0.06501;  // Link 2 질량중심까지 거리 (Aluminum+end plate)

#endif // PARAMETERS_H
