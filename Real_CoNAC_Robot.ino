#include "CAN.h"
#include <HardwareTimer.h>
#include "motor_control.h"
#include "manipulator.h"
#include "trajectory.h"
#include "ctrl_wrapper.h"
#include "parameters.h"

unsigned long lastPrintTimeState = 0;
unsigned long lastControlTimeState = 0;
unsigned long lastPrintTimeFreq = 0;
unsigned long lastPrintTimeLimit = 0;

// ctrl_wrapper 실행 시간
unsigned long ctrlStartTime = 0;
unsigned long ctrlEndTime = 0;

// ctrl_time 저장
unsigned long CtrlTime = 0;

static ControlMode lastControlFlag = STANDBY; // Track the previous CONTROL_FLAG
static unsigned int REST1_count = 0; // 반복 횟수 추적
static unsigned int REST2_count = 0; // 반복 횟수 추적
float MAX_repet = 16.0;

// 1초 타이머
unsigned long lastPrintTimeCtrlExec = 0;
unsigned long lastREST1timeState = 0;
unsigned long lastREST2timeState = 0;

unsigned long loopCalTime = 0;
unsigned long loopEndTime = 0;
unsigned long lastcaltime = 0;

unsigned long lastupdateTime = 0;
unsigned long CalupdateTime = 0;
unsigned long EndupdateTime = 0;

// ✅ STM32F103 기준 타이머 크기 정리
// Timer	크기 (bit)	사용 추천
// TIM1	16-bit (고급)	PWM 등 특수 제어
// TIM2	32-bit	긴 주기/정밀 측정
// TIM3	16-bit	일반 제어
// TIM4	16-bit	일반 제어
// TIM5	32-bit	긴 주기 용
// TIM6	16-bit	간단한 짧은 루프
// TIM7	16-bit	짧은 간격용

HardwareTimer ControlTimer(3);     // 32-bit
HardwareTimer UpdateTimer(2);      // 32-bit

ControlMode prevMode = STANDBY; 

unsigned int CONTROL_NUM = 1;


float t = 0.0f;
Eigen::Matrix2d Kd1;
Eigen::Matrix2d Dd1;

Eigen::Vector2d d_hat(0.0, 0.0);
Eigen::Vector2d Zdot(0.0, 0.0);
Eigen::Vector2d Z(0.0, 0.0);

void setup() {
  delay(3000);
  Serial.begin(115200);

  using namespace CoNAC_Params;
  using namespace CoNAC_Data;
  initializeCoNAC();
  delay(500);

  Serial.print("Setup....");
  delay(500);

  if (!CanBus.begin(CAN_BAUD_1000K)) {
    Serial.println("Failed to initialize CAN!");
    while (1);
  }
  initializeDevice();
  Serial.print("CAN ok....");
  delay(500);

  initializeTimer(); // 타이머 초기화
  delay(500);

  using namespace Manipulator;
  initializeManipulator();

  Kd = Eigen::Vector2d(50, 70).asDiagonal();
  Dd = Eigen::Vector2d(12, 15).asDiagonal();
  Kd1 = Eigen::Vector2d(150, 150).asDiagonal();
  Dd1 = Eigen::Vector2d(50, 50).asDiagonal();
  L << 1, 0,
       0, 1;

  delay(500);

  using namespace Trajectory;
  initializeTrajectory();

  delay(1000);
  Serial.println("Let's go");
}

// Control Loop
void controlLoop() 
{
  using namespace Trajectory;
  using namespace Manipulator;
  using namespace CoNAC_Params;
  using namespace CoNAC_Data;
  float elapsedTime = micros(); 
  ctrlStartTime = micros();
  switch (CONTROL_FLAG){

    case STANDBY : 
        initializeTrajectory();
        u.setZero();
        u_sat.setZero();

        send_var_command5(3, q(0), q(1), r(0), r(1), 0);  
        send_var_command4(4, 0, 0, u(0), u(1));
        send_var_command5_2(5, 0, 0, 0, CONTROL_FLAG, CtrlTime*1e-3);
        
      break;

    case HOME :
      generateReference0(ctrl_dt); 

      computeDYN(M, C, G, q, qdot);
      u = M * (Dd * (rdot - qdot) + Kd * (r - q)) + C * qdot + G;

      u_sat(0) = constrain(u(0), -20, 20);
      u_sat(1) = constrain(u(1), -20, 20);

      send_var_command5(3, q(0), q(1), r(0), r(1), 0);  
      send_var_command4(4, 0, 0, u(0), u(1));
      send_var_command5_2(5, 0, 0, 0, CONTROL_FLAG, CtrlTime*1e-3);
      break;

    case EXECUTE0 :       // PD + DOB
      generateReference1(ctrl_dt); 
      // generateReference3(ctrl_dt); 
      computeDYN(M, C, G, q, qdot);
      u = M * (rddot + Dd * (rdot - qdot) + Kd * (r - q)) + C * qdot + G;
      
      // d_hat = L * (M * qdot + Z);
      // Zdot = - (u + C.transpose()*qdot - G) - d_hat;
      // Z += Zdot * ctrl_dt;

      // u_sat = saturation(u);
      u_sat(0) = constrain(u(0), -15, 15);
      u_sat(1) = constrain(u(1), -10, 10);

      send_var_command5(3, q(0), q(1), r(0), r(1), 0);  
      send_var_command4(4, 0, 0, u(0), u(1));
      send_var_command5_2(5, 0, 0, 0, CONTROL_FLAG, CtrlTime*1e-3);
      
      break;

    case REST1:
    case REST2:
      u.setZero();
      u_sat.setZero();
      send_torque_command1(1, u_sat(0));
      send_torque_command2(2, u_sat(1));
      send_var_command5(3, q(0), q(1), r(0), r(1), 0);  
      send_var_command4(4, 0, 0, u(0), u(1));
      send_var_command5_2(5, 0, 0, 0, CONTROL_FLAG, CtrlTime*1e-3);
      break;

      // lbd(1), // th1
      // lbd(2), // th2
      // lbd(3), // u_ball
      // lbd(4), // u_1 M
      // lbd(5), // u_2 M
      // lbd(6), // u_1 m
      // lbd(7), // u_2 m

    case EXECUTE1 : // c1 CoNAC // 3
        CONTROL_NUM = 1;

        beta[3] = 1e1;
        beta[5] = 1e2;
        beta[7] = 1e2;
        
        // refrence 
        generateReference1(ctrl_dt); 
        // generateReference3(ctrl_dt);
        
        // control input
        ctrl_wrapper(CONTROL_NUM, q, qdot, r, rdot, u, lbd, Vn);
        
        // saturation
        u_sat = saturation(u);

        // send torque command & state command
        send_var_command5(3, q(0), q(1), r(0), r(1), lbd(3));  
        send_var_command4(4, lbd(5), lbd(7), u(0), u(1));
        send_var_command5_2(5, Vn(0), Vn(1), Vn(2), CONTROL_FLAG, CtrlTime*1e-3);
      break;

    case EXECUTE2 : // c4 Aux // 4
      CONTROL_NUM = 2;

      //reference 
      generateReference1(ctrl_dt); 
      
      //control input
      ctrl_wrapper(CONTROL_NUM, q, qdot, r, rdot, u, lbd, Vn);
      
      //saturation 
      u_sat = saturation(u);
      
      // send torque command & state command
      send_var_command5(3, q(0), q(1), r(0), r(1), 0);  
      send_var_command4(4, zeta_arr[0], zeta_arr[1], u(0), u(1));
      send_var_command5_2(5, Vn(0), Vn(1), Vn(2), CONTROL_FLAG, CtrlTime*1e-3);
      
      break;      

    case EXECUTE3 : // c2 CONAC with small beta // 5
        CONTROL_NUM = 1;
        
        beta[3] = 0.1;
        beta[5] = 1;
        beta[7] = 1;

        generateReference1(ctrl_dt); 

        ctrl_wrapper(CONTROL_NUM, q, qdot, r, rdot, u, lbd, Vn);

        u_sat = saturation(u);

        send_var_command5(3, q(0), q(1), r(0), r(1), lbd(3));  
        send_var_command4(4, lbd(5), lbd(7), u(0), u(1));
        send_var_command5_2(5, Vn(0), Vn(1), Vn(2), CONTROL_FLAG, CtrlTime*1e-3);
      break;


    case EXECUTE4 : // c3 CONAC with nothing // 6
        CONTROL_NUM = 1;

        beta[3] = 0;
        beta[5] = 0;
        beta[7] = 0;    

        generateReference1(ctrl_dt); 
        // generateReference3(ctrl_dt); 
        ctrl_wrapper(CONTROL_NUM, q, qdot, r, rdot, u, lbd, Vn);
        u_sat = saturation(u);

        send_var_command5(3, q(0), q(1), r(0), r(1), lbd(0));  
        send_var_command4(4, lbd(1), lbd(2), u(0), u(1));
        send_var_command5_2(5, Vn(0), Vn(1), Vn(2), CONTROL_FLAG, CtrlTime*1e-3);
        
      break;

    default:
      return;
  }

  ctrlEndTime = micros();
  CtrlTime = (ctrlEndTime - ctrlStartTime);

  // Safety check for joint limits
  if (u_sat.array().isNaN().any()) {
    // Serial.println("Warning: u contains NaN. Resetting to zero.");
    u_sat.setZero();
  }
  send_torque_command1(1, u_sat(0));
  send_torque_command2(2, u_sat(1));


  /*
  C1: (13) proposed
    q (2)
    r (2)
    u  (2)
    th (3)
    lbd (3,5,7)
    t  (1)
    cmp (1)
  C2: (12) aux 
    q (2)
    r (2)
    u  (2)
    th (3)
    zeta (2)
    t  (1)
    cmp (1)
  C3: (13) with small beta
    q (2)
    r (2)
    u  (2)
    th (3)
    lbd (3,5,7)
    t  (1)
    cmp (1)
  C4: (13) no control input constraint
    q (2)
    r (2)
    u  (2)
    th (3)
    lbd (1,2,3)
    t  (1)
    cmp (1)
  */

  // loopEndTime = micros();
  // loopCalTime = loopEndTime - ctrlStartTime;


  // if (micros() - lastcaltime >= 1e6) {
  //   Serial.println(CtrlTime*1e-3,4);
  //   Serial.println(loopCalTime*1e-3,4);
  //   lastcaltime = micros();
  // }
}


// Main
void updateLoop() {
  using namespace Manipulator;
  using namespace Trajectory;
  using namespace CoNAC_Data;
  using namespace CoNAC_Params;

  unsigned long elapsedTime = micros();  // 진행 시간 (마이크로초 단위)
  // Reset trajectory state if CONTROL_FLAG changes
  if (CONTROL_FLAG != lastControlFlag) {
    traj_flag(); // Reset trajectory state
    lastControlFlag = CONTROL_FLAG;
  }

  switch (CONTROL_FLAG){

    case STANDBY : 
      if (prevMode != STANDBY) {
        initializeSim();
        // // for (int i = 3; i <= 7; ++i) {   
        // //   beta[i] = 0;
        // // }
        // for (int i = 0; i < 2; ++i) {         // zeta[0]~zeta[1] 업데이트
        //   zeta_arr[i] = 0;
        // }
        // A_zeta[0] = -10;
        // A_zeta[2] = -10;
        // lastREST1timeState = 0;
        // lastREST2timeState = 0;
      }
      break;

    case HOME : 
      if (prevMode != HOME) {
        initializeSim();
      }
      break;

    case EXECUTE0 : // 2
    case EXECUTE1 : // 3
    case EXECUTE2 : // 4
    case EXECUTE3 : // 5
    case EXECUTE4 : // 6
      break;

    case REST1 : 
      if (prevMode != REST1) {
        initializeSim();
        lastREST1timeState = millis(); // Reset the timer
        for (int i = 3; i <= 7; ++i) {         // beta[3]~beta[7] 업데이트
          beta[i] += 5;
        }
      }

      // 2초 대기 후 실행
      if (millis() - lastREST1timeState >= 2000) {
        REST1_count++;
        if (REST1_count >= MAX_repet) {
          REST1_count = 0; // 반복 횟수 초기화
          CONTROL_FLAG = EXECUTE4; // 반복 종료 후 EXECUTE2로 전환
        } else {
          // Execute3 실행
          CONTROL_FLAG = EXECUTE3;
        }
        lastREST1timeState = millis();
      }

      break;

    case REST2 : 
      if (prevMode != REST2) {
        initializeSim();
        for (int i = 3; i <= 7; ++i) {         // beta[3]~beta[7] 업데이트
          beta[i] += 0;
        }
        lastREST2timeState = millis(); // Reset the timer
        // A_zeta 업데이트
        A_zeta[0] -= 5;
        A_zeta[2] -= 5;

      }

      // 2초 대기 후 실행
      if (millis() - lastREST2timeState >= 2000) {
        REST2_count++;
        if (REST2_count >= MAX_repet) {
          REST2_count = 0; // 반복 횟수 초기화
          CONTROL_FLAG = STANDBY; // 반복 종료 후 EXECUTE2로 전환
        } else {
          // Execute1 실행
          CONTROL_FLAG = EXECUTE4;
        }
        lastREST2timeState = millis();
      }
      break;

    default:
      return;
  }

  prevMode = CONTROL_FLAG; // 현재 모드 저장

  updateState();

  // updateDynamics(update_dt);

  // CalupdateTime = EndupdateTime - elapsedTime;
  // if (micros() - lastupdateTime >= 1e6) {
  //   printState(elapsedTime * 1e-6);
  //   lastupdateTime = micros();
  // } 

}


void loop() {

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    handleInputCommand(input);
  }
  // // ✅ 1초마다 printFreq() 실행
  // if (millis() - lastPrintTimeFreq >= 1000) {
  //   printFreq();
  //   printGain();
  //   lastPrintTimeFreq = millis();
  // }
}

void printState(float elapsedTime) {
  using namespace Manipulator;
  using namespace Trajectory;
  using namespace CoNAC_Data;
  using namespace CoNAC_Params;

  // 출력할 데이터를 배열로 구성
  float data[] = {
    elapsedTime, CONTROL_FLAG,
    q(0), q(1), qdot(0), qdot(1),
    r(0), r(1), rdot(0), rdot(1),
    u(0), u(1),
    u_sat(0), 
    u_sat(1), 
    lbd(0), // th0 **** NUM LAMBDA: 8
    lbd(1), // th1 
    lbd(2), // th2
    lbd(3), // u_ball
    lbd(4), // u_1 M
    lbd(5), // u_2 M
    lbd(6), // u_1 m
    lbd(7), // u_2 m
    Vn(0),
    Vn(1),
    Vn(2), // 잊지마잉
    zeta_arr[0],
    zeta_arr[1],
    CtrlTime * 1e-6,  // Add average ctrl_wrapper execution time
    A_zeta[0],
    beta[3]
  };

  // 데이터 반복 출력
  for (size_t i = 0; i < sizeof(data) / sizeof(data[0]); i++) {
    if (i == 0) {
      Serial.print(data[i], 3); 
    } else if (i == 1){
      Serial.print(data[i],1);
    } else {
      Serial.print(data[i], 4);
    }
    if (i < sizeof(data) / sizeof(data[0]) - 1) Serial.print("\t");
  }
  Serial.println();
}

void printGain() {
  using namespace CoNAC_Params;
  using namespace CoNAC_Data;
  using namespace Trajectory;

    Serial.print("u_ball ");
    Serial.print(u_ball);
    Serial.print(" ");

    Serial.print("u_max ");
    Serial.print(u1_max);
    Serial.print(" ");
    Serial.print(u2_max);
    Serial.print(" ");

    Serial.print("alp ");
    Serial.print(alp1);
    Serial.print(" ");
    Serial.print(alp2);
    Serial.print(" ");

    Serial.print("beta ");
    for (int i = 0; i < 8; i++) {
      Serial.print(beta[i]);
      Serial.print(" ");
    }

    Serial.print("th_max ");
    for (int i = 0; i < 3; i++) {
      Serial.print(th_max[i]);
      Serial.print(" ");
    }

    Serial.print("Lambda_arr ");
    for (int i = 0; i < 4; i++) {
      Serial.print(Lambda_arr[i]);
      Serial.print(" ");
    }

    Serial.print("A_zeta ");
    Serial.print(A_zeta[0]);
    Serial.print(" ");

    Serial.print("th_arr ");
    for (int i = 0; i < 3; i++) {
      Serial.print(th_arr[i],4);
      Serial.print(" ");
    }

    Serial.println();
}

void checkJointLimits() {
  using namespace Manipulator;

  bool joint_limit_exceeded =
    q(0) > 1.7 || q(0) < -2.0 ||
    q(1) > 2.6 || q(1) < -2.6 ||
    qdot(0) < -7 || qdot(0) > 7 ||
    qdot(1) < -7 || qdot(1) > 7;

  if (joint_limit_exceeded) {
    if (CONTROL_FLAG == STANDBY) {
      // Serial.println("Joint limit exceeded!");
    } else {
      // Serial.println("Joint limit exceeded!");
      CONTROL_FLAG = HOME;  
    }
  }
}

void initializeSim(){
  using namespace Trajectory;
  using namespace Manipulator;
  using namespace CoNAC_Params;
  using namespace CoNAC_Data;

  // initialize random number generator
  std::srand(18);
  for (int i = 0; i < 58; ++i) {
    double random = static_cast<double>(std::rand()) / RAND_MAX;
    double val = (random - 0.5) * 1e0;
    th_arr[i] = std::round(val * 1e7) / 1e7; 
  }
  Vn.setZero();
  lbd.setZero();
  // q << - M_PI/2, 0;
  // qdot.setZero();
  initializeTrajectory(); // Trajectory 초기화

  u.setZero();
  u_sat.setZero();
}

void initializeTimer(){
  // HardwareTimer ControlTimer(TIM2);     // 32-bit
  // HardwareTimer UpdateTimer(TIM5);      // 32-bit
  // HardwareTimer TrajectoryTimer(TIM3);  // 16-bit
  // HardwareTimer PrintTimer(TIM4);       // 16-bit

  // Update Timer 설정 (500Hz)
  UpdateTimer.pause();
  UpdateTimer.setPeriod(updatePeriodMicros);  // Control loop 주기 설정
  UpdateTimer.attachInterrupt(updateLoop);
  UpdateTimer.refresh();
  UpdateTimer.resume();

  // Control Timer 설정 (250Hz)
  ControlTimer.pause();
  ControlTimer.setPeriod(ctrlPeriodMicros);
  ControlTimer.attachInterrupt(controlLoop);
  ControlTimer.refresh();
  ControlTimer.resume();

}
