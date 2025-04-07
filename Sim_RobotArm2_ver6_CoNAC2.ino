// SIBAL

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
unsigned long ctrlExecSumMicros = 0;
unsigned int ctrlExecCount = 0;

// enterExecuteMode 실행 시간
unsigned long execModeStartTime = 0;
unsigned long execModeEndTime = 0;
unsigned long execModeSumMicros = 0;
unsigned int execModeCount = 0;

// 1초 타이머
unsigned long lastPrintTimeCtrlExec = 0;

// 타이머 및 실행 시간 추적 변수
HardwareTimer SimTimer(2);     // TIM2: Control Loop
HardwareTimer TrajectoryTimer(3);  // TIM3: Trajectory Loop
HardwareTimer ControlTimer(4);

ControlMode prevMode = STANDBY; 

unsigned int CONTROL_NUM = 1;

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

  static ControlMode lastControlFlag = STANDBY; // Track the previous CONTROL_FLAG

  // Reset trajectory state if CONTROL_FLAG changes
  if (CONTROL_FLAG != lastControlFlag) {
    traj_flag(); // Reset trajectory state
    lastControlFlag = CONTROL_FLAG;
  }

  switch (CONTROL_FLAG) {
    case STANDBY:
    case HOME:
      generateReference0(traj_dt); 
      break;

    case EXECUTE1:
    case EXECUTE2:
      generateReference1(traj_dt); 
      break;

    case EXECUTE3:
    case EXECUTE4:    
      generateReference2(traj_dt); 
      break;

      // TODO EXE5, 6 만들기기

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
  ctrlExecSumMicros += (ctrlEndTime - ctrlStartTime);
  ctrlExecCount++;
}


// Main
void SimLoop() 
{
  using namespace Manipulator;
  using namespace Trajectory;
  using namespace CoNAC_Data;

  switch (CONTROL_FLAG){

    case STANDBY : 
      if (prevMode != STANDBY) {
          std::srand(18);
          for (int i = 0; i < 58; ++i) {
            double random = static_cast<double>(std::rand()) / RAND_MAX;
            double val = (random - 0.5) * 1e-5;
            th_arr[i] = std::round(val * 1e7) / 1e7; 
        }
      }
      enterStandbyMode();
      break;

    case HOME : 
      if (prevMode != HOME) {
          std::srand(18);
          for (int i = 0; i < 58; ++i) {
            double random = static_cast<double>(std::rand()) / RAND_MAX;
            double val = (random - 0.5) * 1e-5;
            th_arr[i] = std::round(val * 1e7) / 1e7; 
        }
      }
      enterExecuteMode();    
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
  using namespace CoNAC_Params;
  using namespace CoNAC_Data;


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

  // Calculate average ctrl_wrapper execution time in seconds
  float avgCtrlTimeSec = (ctrlExecCount > 0) ? (ctrlExecSumMicros / float(ctrlExecCount)) / 1e6f : 0.0f;

  // 출력할 데이터를 배열로 구성
  float data[] = {
    elapsedTime,
    CONTROL_FLAG,
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
    avgCtrlTimeSec // Add average ctrl_wrapper execution time
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

    Serial.print("u_ball ");
    Serial.print(u_ball);
    Serial.print(" ");

    Serial.print("alp ");
    Serial.print(alp);
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
void printCalTime() {

    if (ctrlExecCount > 0 || execModeCount > 0) {
        if (ctrlExecCount > 0) {
            float avgCtrlTimeSec = (ctrlExecSumMicros / float(ctrlExecCount)) / 1e6f;
            Serial.print(avgCtrlTimeSec, 6);
        } else {
            Serial.print("0.000000");
        }

        // Serial.print(" ExecuteTime ");
        // if (execModeCount > 0) {
        //     float avgExecModeSec = (execModeSumMicros / float(execModeCount)) / 1e6f;
        //     Serial.print(avgExecModeSec, 6);
        // } else {
        //     Serial.print("0.000000");
        // }

        Serial.println();
    }

    // 초기화
    ctrlExecSumMicros = 0;
    ctrlExecCount = 0;
    execModeSumMicros = 0;
    execModeCount = 0;
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




