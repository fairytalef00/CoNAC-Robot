// SIBAL nomma

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
unsigned long CtrlTime = 0;

static ControlMode lastControlFlag = STANDBY; // Track the previous CONTROL_FLAG

static unsigned int REST1_count = 0; // 반복 횟수 추적
static unsigned int REST2_count = 0; // 반복 횟수 추적
float MAX_repet = 16.0;

// 1초 타이머
unsigned long lastPrintTimeCtrlExec = 0;

unsigned long lastREST1timeState = 0;
unsigned long lastREST2timeState = 0;

// 타이머 및 실행 시간 추적 변수2
HardwareTimer SimTimer(2);     // TIM2: Control Loop
HardwareTimer TrajectoryTimer(3);  // TIM3: Trajectory Loop
HardwareTimer ControlTimer(4);

ControlMode prevMode = STANDBY; 

unsigned int CONTROL_NUM = 1;

void initializeSim(){
  using namespace Trajectory;
  using namespace Manipulator;
  using namespace CoNAC_Params;
  using namespace CoNAC_Data;
  std::srand(18);
  for (int i = 0; i < 146; ++i) {
    double random = static_cast<double>(std::rand()) / RAND_MAX;
    double val = (random - 0.5) * 1e-1;
    th_arr[i] = std::round(val * 1e7) / 1e7; 
  }
  Vn.setZero();
  lbd.setZero();
  q << - M_PI/2, 0;
  qdot.setZero();
  initializeTrajectory(); // Trajectory 초기화

  u.setZero();
}

void setup() {
  delay(1000);
  Serial.begin(115200);

  using namespace CoNAC_Params;
  using namespace CoNAC_Data;
  initializeCoNAC();

  // Serial.println("CoNAC initialized");
  
  // Control Timer 설정 (500Hz)
  SimTimer.pause();
  SimTimer.setPeriod(simPeriodMicros);  // Control loop 주기 설정
  SimTimer.attachInterrupt(SimLoop);
  SimTimer.refresh();
  SimTimer.resume();

  // Trajectory Timer 설정 (100Hz)
  TrajectoryTimer.pause();
  TrajectoryTimer.setPeriod(trajPeriodMicros);  // Trajectory loop 주기 설정
  TrajectoryTimer.attachInterrupt(trajectoryLoop);
  TrajectoryTimer.refresh();
  TrajectoryTimer.resume();

  ControlTimer.pause();
  ControlTimer.setPeriod(ctrlPeriodMicros);
  ControlTimer.attachInterrupt(controlLoop);
  ControlTimer.refresh();
  ControlTimer.resume();


  // Serial.println("Simulation & Trajectory timer Start");

  delay(500);

  using namespace Manipulator;
  initializeManipulator();

  Kd = Eigen::Vector2d(100, 100).asDiagonal();
  Dd = Eigen::Vector2d(10, 10).asDiagonal();

  // initial point
  q << -M_PI/2, 0;

  // Serial.println("Manipulator Library initialized");

  delay(500);

  using namespace Trajectory;
  initializeTrajectory();
  // Serial.println("Trajectory Library initialized");

  delay(1000);
  // Serial.println("Loop... start");
}

// Trajectory Loop
void trajectoryLoop() {
  using namespace Trajectory;

  switch (CONTROL_FLAG) {
    case STANDBY:
      break;

    case HOME:
      generateReference0(traj_dt); 
      break;

    case REST1:
      break;

    case REST2:
      break;

    case EXECUTE1:  // CoNAC cycle1
    case EXECUTE2:  // Aux cycle1

      generateReference1(traj_dt); 
      break;

    case EXECUTE3:  //CoNAC episode1
    case EXECUTE4:  // Aux episode1  
    generateReference3(traj_dt); 
      break;

    default:
      return;
  }
}

// Trajectory Loop
void controlLoop() {
  using namespace Trajectory;
  using namespace Manipulator;
  using namespace CoNAC_Params;
  using namespace CoNAC_Data;

  ctrlStartTime = micros();
  switch (CONTROL_FLAG){

    case STANDBY : 
      u.setZero();
      u_sat.setZero();
      break;

      case REST1:
      u.setZero();
      u_sat.setZero();
      break;

      case REST2:
      u.setZero();
      u_sat.setZero();
      break;

    case HOME :
      computeDYN(M, C, G, q, qdot);
      u = M * (rddot + Dd * (rdot - qdot) + Kd * (r - q)) + C * qdot + G;
      u_sat.setZero();
      break;

    case EXECUTE1 :
    case EXECUTE3 :
      CONTROL_NUM = 1;
      ctrl_wrapper(CONTROL_NUM, q, qdot, r, rdot, u, lbd, Vn);
      u_sat = saturation(u);
      break;

    case EXECUTE2 :
    case EXECUTE4 :
      CONTROL_NUM = 2;
      ctrl_wrapper(CONTROL_NUM, q, qdot, r, rdot, u, lbd, Vn);
      u_sat = saturation(u);
      break;

    default:
        return;
  }

  ctrlEndTime = micros();
  CtrlTime = (ctrlEndTime - ctrlStartTime);
}


// Main
void SimLoop() {
  using namespace Manipulator;
  using namespace Trajectory;
  using namespace CoNAC_Data;
  using namespace CoNAC_Params;

  // Reset trajectory state if CONTROL_FLAG changes
  if (CONTROL_FLAG != lastControlFlag) {
    traj_flag(); // Reset trajectory state
    lastControlFlag = CONTROL_FLAG;
  }

  switch (CONTROL_FLAG){

    case STANDBY : 
      if (prevMode != STANDBY) {
        initializeSim();
        for (int i = 3; i <= 7; ++i) {   
          beta[i] = 10;
        }
        A_zeta[0] = -10;
        A_zeta[2] = -10;
      }
      enterStandbyMode();
      break;
    case HOME : 
      if (prevMode != HOME) {
          std::srand(18);
          for (int i = 0; i < 146; ++i) {
            double random = static_cast<double>(std::rand()) / RAND_MAX;
            double val = (random - 0.5) * 1e-1;
            th_arr[i] = std::round(val * 1e7) / 1e7; 
        }
      }
      enterExecuteMode();    
      break;

    case REST1 : 
      if (prevMode != REST1) {
        initializeSim();
        lastREST1timeState = millis(); // Reset the timer
        for (int i = 3; i <= 7; ++i) {         // beta[3]~beta[7] 업데이트
          beta[i] += 5;
        }
      }

      enterStandbyMode();

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

      enterStandbyMode();

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

    case EXECUTE1 :
      if (prevMode != EXECUTE1) {
        // Serial.println("Start_Record");
      }  
      enterExecuteMode();
      break;

    case EXECUTE2 :
      if (prevMode != EXECUTE2) {
        // Serial.println("Start_Record");
      }  
      enterExecuteMode();
      break;

    case EXECUTE3 :
      if (prevMode != EXECUTE3) {
        // Serial.println("Start_Record");
      }  
      enterExecuteMode();
      break;

    case EXECUTE4 :
      if (prevMode != EXECUTE4) {
        // Serial.println("Start_Record");
      }  
      enterExecuteMode();
      break;

    default:
      return;
  }

  prevMode = CONTROL_FLAG; // 현재 모드 저장
}

void enterStandbyMode() {
  float elapsedTime = millis() / 1000.0f;  // 진행 시간 (초 단위)
  using namespace Manipulator;
  using namespace Trajectory;
  using namespace CoNAC_Params;
  using namespace CoNAC_Data;

  q << - M_PI/2, 0;
  qdot.setZero();
  q0 =q;
  qdot0.setZero();
  r << - M_PI/2, 0;
  rdot.setZero();
  u.setZero();
  u_sat.setZero();


  updateDynamics(sim_dt);
}


void enterExecuteMode() {
  float elapsedTime = millis() / 1000.0f;  // 진행 시간 (초 단위)
  using namespace Manipulator;
  using namespace CoNAC_Data;

  updateDynamics(sim_dt);
}

void loop() {

  float elapsedTime = micros() / 1e6;  // 진행 시간 (초 단위)

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    handleInputCommand(input);
  }
  // ✅ 2ms마다 printState() 실행
  if (millis() - lastPrintTimeState >= printIntervalMillis) {
    printState(elapsedTime);
    // printGain();
    lastPrintTimeState = millis();
  }
}


void printState(float elapsedTime) {
  using namespace Manipulator;
  using namespace Trajectory;
  using namespace CoNAC_Data;
  using namespace CoNAC_Params;

  // 출력할 데이터를 배열로 구성
  float data[] = {
    elapsedTime,
    CONTROL_FLAG,
    A_zeta[0],
    beta[3],
    q(0),
    q(1),
    qdot(0),
    qdot(1),
    r(0),
    r(1),
    rdot(0),
    rdot(1),
    u(0),
    u(1),
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
    CtrlTime * 1e-6 // Add average ctrl_wrapper execution time
  };

  // 데이터 반복 출력
  for (size_t i = 0; i < sizeof(data) / sizeof(data[0]); i++) {
    if (i == 0) {
      Serial.print(data[i], 3); 
    } else {
      Serial.print(data[i], 6);
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

    Serial.print("cycle_count ");
    Serial.print(cycle_count);
    Serial.print(" ");

    Serial.print("beta ");
    for (int i = 0; i < 4; i++) {
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
    q(0) > 2.0 || q(0) < -2.0 ||
    q(1) > 2.1 || q(1) < -2.5 ||
    qdot(0) < -2.5 || qdot(0) > 2.5 ||
    qdot(1) < -2.5 || qdot(1) > 2.5;

  if (joint_limit_exceeded) {
    if (CONTROL_FLAG == STANDBY) {
      Serial.println("Joint limit exceeded!");
    } else {
      Serial.println("Joint limit exceeded! Entering Standby Mode...");
      CONTROL_FLAG = STANDBY;  
    }
  }
}




