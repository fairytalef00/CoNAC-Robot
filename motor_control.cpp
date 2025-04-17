#include "motor_control.h"
#include "ctrl_wrapper.h"

bool isStandbyMode = true;
State state[NUM_IDS];
unsigned long receiveCounts[NUM_IDS] = {0};
unsigned long processedCounts[NUM_IDS] = {0};

#define BUFFER_SIZE 10  // 각 채널별 버퍼 크기 (필요에 따라 조절)

volatile unsigned long matchCount_0x2901 = 0;
volatile unsigned long matchCount_0x2902 = 0;
volatile unsigned long lastRxTime_0x2901 = 0;
volatile unsigned long lastRxTime_0x2902 = 0;

typedef struct {
    uint8_t data[8];
    unsigned long timestamp;
} CANMessageBuffer;

volatile CANMessageBuffer canBuffer[NUM_IDS][BUFFER_SIZE];
volatile int bufferWriteIndex[NUM_IDS] = {0, 0};
volatile int bufferReadIndex[NUM_IDS]  = {0, 0};
volatile int bufferCount[NUM_IDS]      = {0, 0};

ControlMode CONTROL_FLAG = STANDBY;

void initializeDevice(){

    int16_t p_zero1 = float_to_uint(0, 0, 0, 16);
    int16_t v_zero1 = float_to_uint(0, 0, 0, 16);
    int16_t t_zero1 = float_to_uint(0, 0, 0, 16);
    state[0].data[0] = p_zero1 >> 8;
    state[0].data[1] = p_zero1 & 0xFF;
    state[0].data[2] = v_zero1 >> 8;
    state[0].data[3] = v_zero1 & 0xF;
    state[0].data[4] = t_zero1 >> 8;
    state[0].data[4] = t_zero1 & 0xFF;
    state[0].received = true;      

    state[1].data[0] = p_zero1 >> 8;
    state[1].data[1] = p_zero1 & 0xFF;
    state[1].data[2] = v_zero1 >> 8;
    state[1].data[3] = v_zero1 & 0xF;
    state[1].data[4] = t_zero1 >> 8;
    state[1].data[4] = t_zero1 & 0xFF;
    state[1].received = true;      

    // CAN 인터럽트 핸들러 설정 
    CanBus.attachRxInterrupt(onCANReceive);

    delay(1000);
}

void onCANReceive(can_message_t *msg) {
    unsigned long now = millis();
    int channel = -1;
    switch (msg->id) {
        case 0x2901: channel = 0; break;
        case 0x2902: channel = 1; break;
        default: return; // 알 수 없는 ID는 무시
    }
    
    int w = bufferWriteIndex[channel];
    memcpy((void*)canBuffer[channel][w].data, msg->data, 8);
    canBuffer[channel][w].timestamp = now;
    bufferWriteIndex[channel] = (w + 1) % BUFFER_SIZE;
    if (bufferCount[channel] < BUFFER_SIZE) {
        bufferCount[channel]++;
    }
    
    // 기존 진단 카운터 업데이트 (채널별)
    switch (channel) {
        case 0: matchCount_0x2901++; lastRxTime_0x2901 = now; receiveCounts[0]++; break;
        case 1: matchCount_0x2902++; lastRxTime_0x2902 = now; receiveCounts[1]++; break;
    }
}



// =================== 주기 처리 루프 ===================
void updateState() {
    for (int channel = 0; channel < NUM_IDS; channel++) {
        while (true) {
            noInterrupts();  // 임계영역 진입: 공유 변수 접근 보호
            if (bufferCount[channel] <= 0) {
                interrupts();  // 임계영역 종료
                break;
            }
            int r = bufferReadIndex[channel];
            // 버퍼에서 데이터를 안전하게 복사
            uint8_t data[8];
            memcpy(data, (void*)canBuffer[channel][r].data, 8);
            // 인덱스 및 카운트 업데이트
            bufferReadIndex[channel] = (r + 1) % BUFFER_SIZE;
            bufferCount[channel]--;
            interrupts();  // 임계영역 종료

            // 임계영역 외부에서 unpack 처리 (시간 소요가 있는 작업은 여기서 수행)
            unpack_reply(data, channel);
            processedCounts[channel]++;
        }
    }
}


void unpack_reply(uint8_t* data, int8_t index) {
    using namespace Manipulator;
    switch (index) {
        case 0: motor_receive1(data); break;
        case 1: motor_receive2(data); break;
    }
}


// Float to uint 변환
int float_to_uint(float x, float x_min, float x_max, unsigned int bits){ 
    float span = x_max - x_min;
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    return (int) ((x- x_min)*((float)((1<<bits)/span)));
}

void motor_receive1(const uint8_t* data) { 
    int16_t pos_int = (data[0] << 8) | data[1]; 
    int16_t spd_int = (data[2] << 8) | data[3]; 
    int16_t cur_int = (data[4] << 8) | data[5]; 

    using namespace Manipulator;
    q(0) = (float)(pos_int * 0.1f) * (M_PI / 180.0f);
    qdot(0) = (float)(spd_int * 10.0f / (9.0f * 21.0f)) * (2 * M_PI / 60.0f);  // Electrical RPM → Mechanical rad/s
    tau(0) = (float)(1 / 0.75* cur_int * 0.01f);  // Torque N*m
}

void motor_receive2(const uint8_t* data) { 
    int16_t pos_int = (data[0] << 8) | data[1]; 
    int16_t spd_int = (data[2] << 8) | data[3]; 
    int16_t cur_int = (data[4] << 8) | data[5]; 

    using namespace Manipulator;
    q(1) = (float)(pos_int * 0.1f) * (M_PI / 180.0f);
    qdot(1) = (float)(spd_int * 10.0f / (9.0f * 21.0f)) * (2 * M_PI / 60.0f);  // Electrical RPM → Mechanical rad/s
    tau(1) = (float)(1 / 0.7* cur_int * 0.01f);  // Torque N*m
}


void pack_cmd(can_message_t *msg, float p_des, float v_des, float kp, float kd, float t_ff, uint8_t motor_id) {
    const uint8_t CONTROL_MODE_ID = 8;
    msg->id = (CONTROL_MODE_ID << 8) | motor_id;  // MIT Torque Control ID + Motor ID
    msg->length = 8;
    msg->format = CAN_EXT_FORMAT;
    
    int p_int = float_to_uint(0, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(0, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(0, KP_MIN, KP_MAX, 12);
    int kd_int = float_to_uint(0, KD_MIN, KD_MAX, 12);
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    msg->data[0] = kp_int >> 4;
    msg->data[1] = ((kp_int & 0xF) << 4) | (kd_int >> 8);
    msg->data[2] = kd_int & 0xFF;
    msg->data[3] = p_int >> 8;
    msg->data[4] = p_int & 0xFF; 
    msg->data[5] = v_int >> 4;
    msg->data[6] = ((v_int & 0xF) << 4) | (t_int >> 8);
    msg->data[7] = t_int & 0xFF;
}

void pack_var2(can_message_t *msg, float var1, float var2, uint16_t var_id) {
    msg->id = var_id; 
    msg->length = 8;
    msg->format = CAN_STD_FORMAT;

    // 32 // 32
    msg->data[0] = ((int32_t)(var1 * 1000) & 0xFF000000) >> 24;
    msg->data[1] = ((int32_t)(var1 * 1000) & 0x00FF0000) >> 16;
    msg->data[2] = ((int32_t)(var1 * 1000) & 0x0000FF00) >> 8;
    msg->data[3] = ((int32_t)(var1 * 1000) & 0x000000FF);

    msg->data[4] = ((int32_t)(var2 * 1000) & 0xFF000000) >> 24;
    msg->data[5] = ((int32_t)(var2 * 1000) & 0x00FF0000) >> 16;
    msg->data[6] = ((int32_t)(var2 * 1000) & 0x0000FF00) >> 8;
    msg->data[7] = ((int32_t)(var2 * 1000) & 0x000000FF);
}

void pack_var3(can_message_t *msg, float var1, float var2, float var3, uint16_t var_id) {
    msg->id = var_id; 
    msg->length = 8;
    msg->format = CAN_STD_FORMAT;

    // 32 // 16 // 16
    msg->data[0] = ((int32_t)(var1 * 1000) & 0xFF000000) >> 24;
    msg->data[1] = ((int32_t)(var1 * 1000) & 0x00FF0000) >> 16;
    msg->data[2] = ((int32_t)(var1 * 1000) & 0x0000FF00) >> 8;
    msg->data[3] = ((int32_t)(var1 * 1000) & 0x000000FF);

    msg->data[4] = ((int16_t)(var2 * 1000) & 0xFF00) >> 8;
    msg->data[5] = ((int16_t)(var2 * 1000) & 0x00FF);
    
    msg->data[6] = ((int16_t)(var3 * 1000) & 0xFF00) >> 8;
    msg->data[7] = ((int16_t)(var3 * 1000) & 0x00FF);
}

void pack_var4(can_message_t *msg, float var1, float var2, float var3, float var4, uint16_t var_id) {
    msg->id = var_id; 
    msg->length = 8;
    msg->format = CAN_STD_FORMAT;

    // 16 // 16 // 16 // 16
    msg->data[0] = ((int16_t)(var1 * 1000) & 0xFF00) >> 8;
    msg->data[1] = ((int16_t)(var1 * 1000) & 0x00FF);
    
    msg->data[2] = ((int16_t)(var2 * 1000) & 0xFF00) >> 8;
    msg->data[3] = ((int16_t)(var2 * 1000) & 0x00FF);

    msg->data[4] = ((int16_t)(var3 * 1000) & 0xFF00) >> 8;
    msg->data[5] = ((int16_t)(var3 * 1000) & 0x00FF);
    
    msg->data[6] = ((int16_t)(var4 * 1000) & 0xFF00) >> 8;
    msg->data[7] = ((int16_t)(var4 * 1000) & 0x00FF);
}


void send_var_command2(uint16_t var_id, float var1, float var2) {
    can_message_t msg;
    pack_var2(&msg, var1, var2, var_id);
    CanBus.write(msg.id, msg.data, 8, CAN_STD_FORMAT);
}

void send_var_command3(uint16_t var_id, float var1, float var2, float var3) {
    can_message_t msg;
    pack_var3(&msg, var1, var2, var3, var_id);
    CanBus.write(msg.id, msg.data, 8, CAN_STD_FORMAT);
}

void send_var_command4(uint16_t var_id, float var1, float var2, float var3, float var4) {
    can_message_t msg;
    pack_var4(&msg, var1, var2, var3, var4, var_id);
    CanBus.write(msg.id, msg.data, 8, CAN_STD_FORMAT);
}


// void send_torque_command1(uint8_t motor_id, float torque) {
//     can_message_t msg;
//     pack_cmd(&msg, 0.0, 0.0, 0.0, 0.0, 0.70f * torque, motor_id);
//     CanBus.write(msg.id, msg.data, 8, CAN_EXT_FORMAT);
// }

// ✅ 수신 주파수 출력 함수
void printFreq() {
    for (int i = 0; i < NUM_IDS; i++) {
        Serial.print("ID:");
        Serial.print(i);
        Serial.print(" ");  // ✅ ID와 주파수 사이에 공백 추가
        Serial.print(processedCounts[i]);
        Serial.print(" ");  // ✅ 다음 ID와 구분하기 위해 공백 추가
        processedCounts[i] = 0;  // ✅ 주기 초기화
        receiveCounts[i] = 0;  // ✅ 주기 초기화
    }
    Serial.println();
}

// 원점 설정 명령
void set_origin_command(uint8_t motor_id) {
    if (motor_id == 1) {
        can_message_t msg;
        msg.id = (5 << 8) | motor_id;  // Control Mode ID + Driver ID
        msg.length = 1;  // 원점 설정 명령은 1바이트 데이터
        msg.format = CAN_EXT_FORMAT;  // 확장형 CAN 프레임 사용
        msg.data[0] = (uint8_t)0x01;  // 0x00 (임시) 또는 0x01 (영구)

        CanBus.writeMessage(&msg);
    } 
    else if (motor_id == 2) {
        can_message_t msg;
        msg.id = (5 << 8) | motor_id;  // Control Mode ID + Driver ID
        msg.length = 1;  // 원점 설정 명령은 1바이트 데이터
        msg.format = CAN_EXT_FORMAT;  // 확장형 CAN 프레임 사용
        msg.data[0] = (uint8_t)0x01;  // 0x00 (임시) 또는 0x01 (영구)

        CanBus.writeMessage(&msg);
    }     
}    
// 토크 명령 전송
void send_torque_command1(uint8_t motor_id, float torque) {
    can_message_t msg;
    pack_cmd(&msg, 0.0, 0.0, 0.0, 0.0, 0.70f * torque, motor_id);
    CanBus.write(msg.id, msg.data, 8, CAN_EXT_FORMAT);
}
void send_torque_command2(uint8_t motor_id, float torque) {
    can_message_t msg;
    pack_cmd(&msg, 0.0, 0.0, 0.0, 0.0, 0.70f * torque, motor_id);
    CanBus.write(msg.id, msg.data, 8, CAN_EXT_FORMAT);
}


// 시스템 리부트
void rebootSystem() {
    NVIC_SystemReset();
}

void handleInputCommand(const String& input) {
    using namespace Manipulator;
    using namespace Trajectory;

    if (input.equalsIgnoreCase("REBOOT")) {
        delay(100); // 메시지가 출력될 시간을 확보
        NVIC_SystemReset(); // 시스템 리셋
    } 
    // ORIGIN 명령 처리
    else if (input.startsWith("origin:")) {
        int controller_id = input.substring(input.indexOf(':') + 1).toInt(); // ':' 이후 값 추출
        if (controller_id == 1 || controller_id == 2) {
            set_origin_command(controller_id); // 해당 controller_id로 명령 실행
        } else {
        }
    } 
    else if (input.equalsIgnoreCase("3")) {
        // CONTROL_NUM = 1;
        CONTROL_FLAG = EXECUTE1;
    } 
    else if (input.equalsIgnoreCase("5")) {
        // CONTROL_NUM = 3; // Traj
        CONTROL_FLAG = EXECUTE3;
    } 
    else if (input.equalsIgnoreCase("6")) {
        // CONTROL_NUM = 4; // Traj
        CONTROL_FLAG = EXECUTE4;
    } 
    else if (input.equalsIgnoreCase("4")) {
        // CONTROL_NUM = 2; // Traj
        CONTROL_FLAG = EXECUTE2;
    } 
    else if (input.equalsIgnoreCase("2")) {
        CONTROL_FLAG = EXECUTE0;
    }     
    else if (input.equalsIgnoreCase("1")) {
        CONTROL_FLAG = HOME;
    }     
    else if (input.equalsIgnoreCase("0")) {
        CONTROL_FLAG = STANDBY;
    } 


    // Ttraj 설정 처리
    else if (input.startsWith("Ttraj=")) {
        float new_Ttraj = input.substring(6).toFloat(); // "Ttraj=" 이후 부분
        if (new_Ttraj > 0) {
            Ttraj = new_Ttraj;
        } 
    }
    // cutoff 설정 처리
    else if (input.startsWith("cutoff=")) {
        double new_cutoff = static_cast<double>(input.substring(7).toFloat()); // "cutoff=" 이후 부분
        if (new_cutoff > 0) {
            cutoff = new_cutoff;
        }
    }
    else if (input.startsWith("u_ball=")) {
        double val = input.substring(7).toFloat();
        CoNAC_Params::u_ball = val;
    }
    else if (input.startsWith("rho=")) {
        double val = input.substring(4).toFloat();
        CoNAC_Params::rho = val;
    }
    else if (input.startsWith("u1_max=")) {
        double val = input.substring(7).toFloat();
        CoNAC_Params::u1_max = val;
    }
    else if (input.startsWith("u2_max=")) {
        double val = input.substring(7).toFloat();
        CoNAC_Params::u2_max = val;
    }
    else if (input.startsWith("alp=")) {
        double val = input.substring(4).toFloat();
        CoNAC_Params::alp1 = val;
        CoNAC_Params::alp2 = val;
    }
    else if (input.startsWith("beta=")) {
        double val = input.substring(5).toFloat();
        for (int i = 3; i<= 7; ++i) {         // beta[3]~beta[7] 업데이트
            CoNAC_Params::beta[i] = val;
        }
    }
    else if (input.startsWith("B=") || input.startsWith("Lambda_arr=") ||
            input.startsWith("th_max=")) {
        
        String varName = input.substring(0, input.indexOf('='));
        String data = input.substring(varName.length() + 1);

        data.replace("[", "");
        data.replace("]", "");

        if (varName == "B" || varName == "Lambda_arr") {
            int rowIdx = data.indexOf(';');
            if (rowIdx != -1) {
                String roalp1 = data.substring(0, rowIdx);
                String roalp2 = data.substring(rowIdx + 1);

                int col1 = roalp1.indexOf(',');
                int col2 = roalp2.indexOf(',');

                if (col1 != -1 && col2 != -1) {
                    float a11 = roalp1.substring(0, col1).toFloat();
                    float a12 = roalp1.substring(col1 + 1).toFloat();
                    float a21 = roalp2.substring(0, col2).toFloat();
                    float a22 = roalp2.substring(col2 + 1).toFloat();

                    if (varName == "B") {
                        CoNAC_Params::B[0] = a11;
                        CoNAC_Params::B[1] = a12;
                        CoNAC_Params::B[2] = a21;
                        CoNAC_Params::B[3] = a22;
                    } else if (varName == "Lambda_arr") {
                        CoNAC_Params::Lambda_arr[0] = a11;
                        CoNAC_Params::Lambda_arr[1] = a12;
                        CoNAC_Params::Lambda_arr[2] = a21;
                        CoNAC_Params::Lambda_arr[3] = a22;
                    }
                }
            }
        }

        else if (varName == "th_max") {
            int sep = data.indexOf(';');
            if (sep != -1) {
                CoNAC_Params::th_max[0] = data.substring(0, sep).toFloat();
                CoNAC_Params::th_max[1] = data.substring(sep + 1).toFloat();
                CoNAC_Params::th_max[2] = data.substring(sep + 1).toFloat();
            }
        }
    }
}






