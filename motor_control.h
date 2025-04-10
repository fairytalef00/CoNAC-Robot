#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "manipulator.h"
#include "trajectory.h"
#include "parameters.h"
#include <CAN.h>
#include <math.h>

enum ControlMode {
  STANDBY = 0,
  HOME = 1,
  EXECUTE1 = 2,
  EXECUTE2 = 3,
  EXECUTE3 = 4,
  EXECUTE4 = 5,
  REST1 = 6,
  REST2 = 7
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
#define NUM_IDS 3  // ID 개수 

typedef struct State {
    bool received;  // 데이터가 수신되었는지 여부
    uint8_t data[8];
    unsigned long lastReceivedTime;  // 마지막 수신 시간
} State;
extern State state[NUM_IDS];
extern unsigned int receiveCounts[NUM_IDS]; // ✅ 수신 횟수 배열 선언

// 함수 선언
void initializeDevice();
void onCANReceive(can_message_t *msg) ;
void updateState() ;
void processStep(State state[]) ;
void resetState() ;

int float_to_uint(float x, float x_min, float x_max, unsigned int bits);

void pack_cmd(can_message_t *msg, float p_des, float v_des, float kp, float kd, float t_ff);

void unpack_reply(uint8_t* data, int8_t index);
void motor_receive1(const uint8_t* data) ;
void motor_receive2(const uint8_t* data) ;
void ft_receive1(const uint8_t* data) ;
void ft_receive2(const uint8_t* data) ;

void printFreq(); 

void set_origin_command(uint8_t controller_id);

void send_torque_command1(uint8_t motor_id, float torque);
void send_torque_command2(uint8_t motor_id, float torque);
void rebootSystem();
void updateState();
void ftbiasUpdate(); 
void handleInputCommand(const String& input);
#endif // MOTOR_CONTROL_H
