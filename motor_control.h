#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "manipulator.h"
#include "trajectory.h"
#include "parameters.h"
#include <CAN.h>
#include <math.h>

extern bool isStandbyMode;

enum ControlMode {
  STANDBY = 0, // 대기 모드
  HOME = 1,   // 초기화
  EXECUTE0 = 2, // PD + DOB
  EXECUTE1 = 3, // CoNAC cycle1
  EXECUTE2 = 4, // Aux cycle1
  EXECUTE3 = 5, // CoNAC episode1
  EXECUTE4 = 6, // Aux episode1
  REST1 = 7,
  REST2 = 8
};

extern ControlMode CONTROL_FLAG;

// CAN 데이터 관련 상수
constexpr float P_MIN = -12.56f;   // 최소 위치
constexpr float P_MAX = 12.56f;    // 최대 위치
constexpr float V_MIN = -33.0f;   // 최소 속도
constexpr float V_MAX = 33.0f;    // 최대 속도
constexpr float KP_MIN = 0.0f;    // 최소 비례 게인
constexpr float KP_MAX = 500.0f;  // 최대 비례 게인
constexpr float KD_MIN = 0.0f;    // 최소 미분 게인
constexpr float KD_MAX = 5.0f;    // 최대 미분 게인
constexpr float T_MIN = -65.0f;   // 최소 토크
constexpr float T_MAX= 65.0f;    // 최대 토크


// 함수 선언
#define TIMEOUT_MS 2  // 500Hz 동기화를 위한 2ms 타임아웃
#define NUM_IDS 2  // ID 개수 

typedef struct State {
  bool received;     // 수신 완료 여부
  bool processed;    // 이번 step에서 이미 처리했는지 여부
  uint8_t data[8];
  unsigned long lastReceivedTime;
};

extern State state[NUM_IDS];
extern unsigned long receiveCounts[NUM_IDS];
extern unsigned long processedCounts[NUM_IDS];

// 함수 선언
void initializeDevice();
void onCANReceive(can_message_t *msg) ;
void updateState() ;

int float_to_uint(float x, float x_min, float x_max, unsigned int bits);

void pack_cmd(can_message_t *msg, float p_des, float v_des, float kp, float kd, float t_ff);
void unpack_reply(uint8_t* data, int8_t index);
void motor_receive1(const uint8_t* data);
void motor_receive2(const uint8_t* data);

void printFreq(); 

void set_origin_command(uint8_t controller_id);
void send_torque_command1(uint8_t motor_id, float torque);
void send_torque_command2(uint8_t motor_id, float torque);

void pack_var2(can_message_t *msg, float var1, float var2, uint16_t var_id);
void pack_var3(can_message_t *msg, float var1, float var2, float var3, uint16_t var_id);
void pack_var4(can_message_t *msg, float var1, float var2, float var3, float var4, uint16_t var_id);

void send_var_command2(uint16_t var_id, float var1, float var2);
void send_var_command3(uint16_t var_id, float var1, float var2, float var3);
void send_var_command4(uint16_t var_id, float var1, float var2, float var3, float var4);

void rebootSystem();
void updateState();
void ftbiasUpdate(); 
void handleInputCommand(const String& input);
#endif // MOTOR_CONTROL_H
