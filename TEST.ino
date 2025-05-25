#include "CAN.h"
#include <HardwareTimer.h>
#include "motor_control.h"
#include "manipulator.h"
#include "trajectory.h"
#include "ctrl_wrapper.h"
#include "parameters.h"
HardwareTimer ControlTimer(2);     // 32-bit
HardwareTimer UpdateTimer(3);      // 32-bit

Eigen::Matrix2d Kd1;
Eigen::Matrix2d Dd1;
Eigen::Vector2d filter_qdot(0.0, 0.0);
// float qdot_cutoff = 20.0f;
float qdot_cutoff = 60.0f;

Eigen::Vector2d d_hat(0.0, 0.0);
Eigen::Vector2d Zdot(0.0, 0.0);
Eigen::Vector2d Z(0.0, 0.0);

void setup() {
  delay(3000);
  Serial.begin(115200);

  using namespace CoNAC_Params;
  using namespace CoNAC_Data;

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

  Kd = Eigen::Vector2d(100, 100).asDiagonal();
  Dd = Eigen::Vector2d(20, 20).asDiagonal();
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
  switch (CONTROL_FLAG){

    case STANDBY : 
      initializeTrajectory();
      u.setZero();
      u_sat.setZero();
      send_var_command4(3, r(0), r(1), rdot(0), rdot(1));  
      // send_var_command4(4, qdot(0), qdot(1), rdot(0), rdot(1));
      send_var_command4(5, u(0), u(1), CONTROL_FLAG, 0);
        
      break;

    case HOME :
      generateReference0(ctrl_dt); 

      computeDYN(M, C, G, q, qdot);
      u = M * (Dd * (rdot - qdot) + Kd * (r - q)) + C * qdot + G;

      u_sat(0) = constrain(u(0), -20, 20);
      u_sat(1) = constrain(u(1), -20, 20);
      send_var_command4(3, r(0), r(1), rdot(0), rdot(1));  
      // send_var_command4(4, qdot(0), qdot(1), rdot(0), rdot(1));
      send_var_command4(5, u(0), u(1), CONTROL_FLAG, 0);
      break;

    case EXECUTE0 :       // PD + DOB
      // generateReference1(ctrl_dt); 
      generateReference2(ctrl_dt); 

      computeDYN(M, C, G, q, qdot);
      u = M * (rddot + Dd * (rdot - qdot) + Kd * (r - q)) + C * qdot + G;
      
      // d_hat = L * (M * qdot + Z);
      // Zdot = - (u + C.transpose()*qdot - G) - d_hat;
      // Z += Zdot * ctrl_dt;

      // u_sat = saturation(u);
      u_sat(0) = constrain(u(0), -20, 20);
      u_sat(1) = constrain(u(1), -20, 20);

      send_var_command4(3, r(0), r(1), rdot(0), rdot(1));  
      send_var_command4(4, rddot(0), rddot(1), filter_qdot(0), filter_qdot(1));
      send_var_command4(5, u(0), u(1), CONTROL_FLAG, 0);

      break;

    case EXECUTE1 : {
      // generateReference1(ctrl_dt); 

      // If using matrix, computing time_Ref  = 0.641 ms. (6*6 matrix)
      // If using vector, computing time_Ref  = 0.012 ~ 0.013 ms. 

      generateReference3(ctrl_dt); 
      // computing time_DYN  = 0.012 ~ 0.0145 ms 
      computeDYN(M, C, G, q, filter_qdot);
      u = M * (rddot + Dd * (rdot - filter_qdot) + Kd * (r - q)) + C * filter_qdot + G;

      // d_hat = L * (M * qdot + Z);
      // Zdot = - (u + C.transpose()*qdot - G) - d_hat;
      // Z += Zdot * ctrl_dt;

      // u_sat = saturation(u);
      u_sat(0) = constrain(u(0), -20, 20);
      u_sat(1) = constrain(u(1), -20, 20);

      // computing time_CAN  = 0.0120 ~ 0.0124 ms 
      send_var_command4(3, r(0), r(1), rdot(0), rdot(1));  
      send_var_command4(4, rddot(0), rddot(1), filter_qdot(0), filter_qdot(1));
      send_var_command4(5, u(0), u(1), CONTROL_FLAG, 0);

      // predictive total computing time
      // (0.012 ~ 0.014 ms) + (0.012 ~ 0.0145 ms) + 5 * (0.012 ~ 0.0124 ms)
      // = (0.012 ~ 0.014) + (0.012 ~ 0.0145) + (0.060 ~ 0.062)
      // = (0.084 ~ 0.0905) ms
      break;
    }
    case EXECUTE2 :
    case EXECUTE3 : 
    case EXECUTE4 : 
      break;

    default:
      return;
  }

  // Safety check for joint limits
  if (u_sat.array().isNaN().any()) {
    // Serial.println("Warning: u contains NaN. Resetting to zero.");
    u_sat.setZero();
  }
  send_torque_command1(1, u_sat(0));
  send_torque_command2(2, u_sat(1));
}


// Main
void updateLoop() {
  using namespace Manipulator;
  using namespace Trajectory;
  using namespace CoNAC_Data;
  using namespace CoNAC_Params;
  LowPassFilter2(filter_qdot, qdot, qdot_cutoff, update_dt);
  updateState();

  // updateDynamics(update_dt);
  // send_var_command4(8, q(0), qdot(0), q(1), qdot(1));
}


void loop() {

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    handleInputCommand(input);
  }
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

  // Control Timer 설정 (500Hz)
  ControlTimer.pause();
  ControlTimer.setPeriod(ctrlPeriodMicros);
  ControlTimer.attachInterrupt(controlLoop);
  ControlTimer.refresh();
  ControlTimer.resume();

}
