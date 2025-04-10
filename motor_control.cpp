#include "motor_control.h"
#include "ctrl_wrapper.h"

State state[NUM_IDS];
unsigned int receiveCounts[NUM_IDS] = {0};
uint32_t trackedIDs[NUM_IDS] = { 0x01, 0x02, 0x04 };  

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

    // ✅ ft_receive1() 에 맞춘 초기값
    int fx_zero = 30000, fy_zero = 30000, fz_zero = 30000;
    state[2].data[0] = (fx_zero >> 8) & 0xFF;
    state[2].data[1] = fx_zero & 0xFF;
    state[2].data[2] = (fy_zero >> 8) & 0xFF;
    state[2].data[3] = fy_zero & 0xFF;
    state[2].data[4] = (fz_zero >> 8) & 0xFF;
    state[2].data[5] = fz_zero & 0xFF;
    state[2].received = true;
    // CAN 인터럽트 핸들러 설정 
    CanBus.attachRxInterrupt(onCANReceive);


    delay(1000);

}

// ✅ CAN 메시지 수신 인터럽트 핸들러
void onCANReceive(can_message_t *msg) {
    unsigned long currentTime = millis();
    
    switch (msg->id) {
        case 0x2901:
            memcpy(state[0].data, msg->data, 8);
            state[0].received = true;
            state[0].lastReceivedTime = currentTime;
            receiveCounts[0]++;
            break;
        case 0x2902:
            memcpy(state[1].data, msg->data, 8);
            state[1].received = true;
            state[1].lastReceivedTime = currentTime;
            receiveCounts[1]++;
            break;
        case 0x04:
            memcpy(state[2].data, msg->data, 8);
            state[2].received = true;
            state[2].lastReceivedTime = currentTime;
            receiveCounts[2]++;
            break;
        default:
            return;
    }
}

void updateState() {
    unsigned long currentTime = millis();
    bool anyReceived = false;  // 하나라도 데이터가 수신되었는지 체크

    for (int i = 0; i < NUM_IDS; i++) {
        if (!state[i].received) {
            if (currentTime - state[i].lastReceivedTime > TIMEOUT_MS) {
                // ✅ TIMEOUT 발생: 이전 값 유지
                state[i].received = true;  // Step 진행을 위해 true 설정
            }
        } else {
            anyReceived = true;  // 하나라도 데이터를 받았다면 Step 진행 가능
        }
    }

    // ✅ 하나라도 데이터가 도착하면 Step 처리 진행
    if (anyReceived) {
        processStep(state);
        resetState();
    }
}

// ✅ 500Hz 동기화 처리
void processStep(State state[]) {
    unpack_reply(state[0].data, 0);
    unpack_reply(state[1].data, 1);
    unpack_reply(state[2].data, 2);
}

// ✅ Step이 끝나면 상태 초기화
void resetState() {
    for (int i = 0; i < NUM_IDS; i++) {
        state[i].received = false;
    }
}

// CAN 메시지에서 모터 상태를 해석
void unpack_reply(uint8_t* data, int8_t index) {
    using namespace Manipulator;

    switch (index) {
        case 0:  
            motor_receive1(data);
            return;
        case 1: 
            motor_receive2(data);
            return;
        case 2: 
            ft_receive1(data);
            return;
        case 3: 
            ft_receive2(data);
            return;
        default: 
            return;
        
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

void ft_receive1(const uint8_t* data) { 
    uint16_t fx_raw = (data[0] << 8) | data[1];
    uint16_t fy_raw = (data[2] << 8) | data[3];
    uint16_t fz_raw = (data[4] << 8) | data[5];

    float fx = (float)fx_raw / 100.0f - 300.0f;
    float fy = (float)fy_raw / 100.0f - 300.0f;
    float fz = (float)fz_raw / 100.0f - 300.0f;

    using namespace Manipulator;
    Fraw(0) = fx - Fbias(0);
    Fraw(1) = fy - Fbias(1);
    Fraw(2) = fz - Fbias(2);
}

void ft_receive2(const uint8_t* data) {
    uint16_t tx_raw = (data[0] << 8) | data[1];
    uint16_t ty_raw = (data[2] << 8) | data[3];
    uint16_t tz_raw = (data[4] << 8) | data[5];

    float tx = (float)tx_raw / 500.0f - 50.0f;
    float ty = (float)ty_raw / 500.0f - 50.0f;
    float tz = (float)tz_raw / 500.0f - 50.0f;

    using namespace Manipulator;
    Traw(0) = tx - Tbias(0);
    Traw(1) = ty - Tbias(1);
    Traw(2) = tz - Tbias(2);
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

// ✅ 수신 주파수 출력 함수
void printFreq() {
    for (int i = 0; i < NUM_IDS; i++) {
        Serial.print("ID:");
        Serial.print(trackedIDs[i], HEX);  // ✅ 실제 CAN ID 출력
        Serial.print(" ");  
        Serial.print(receiveCounts[i]);  // ✅ 해당 ID의 수신 횟수 출력
        Serial.print(" ");  
        receiveCounts[i] = 0;  // 카운트 초기화
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
    pack_cmd(&msg, 0.0, 0.0, 0.0, 0.0, 0.75 * torque, motor_id);
    // pack_cmd(&msg, 0.0, 0.0, 0.0, 0.0, 0.6 * torque, motor_id);
    CanBus.write(msg.id, msg.data, 8, CAN_EXT_FORMAT);
}
void send_torque_command2(uint8_t motor_id, float torque) {
    can_message_t msg;
    pack_cmd(&msg, 0.0, 0.0, 0.0, 0.0, 0.70 * torque, motor_id);
    // pack_cmd(&msg, 0.0, 0.0, 0.0, 0.0, 0.5 * torque, motor_id);
    CanBus.write(msg.id, msg.data, 8, CAN_EXT_FORMAT);
}

// 시스템 리부트
void rebootSystem() {
    NVIC_SystemReset();
}

void ftbiasUpdate() {
    using namespace Manipulator;
    Fbias = Fraw + Fbias;
    Tbias = Traw + Tbias;
}

void handleInputCommand(const String& input) {
    using namespace Manipulator;
    using namespace Trajectory;

    if (input.equalsIgnoreCase("REBOOT")) {
        delay(100); // 메시지가 출력될 시간을 확보
        NVIC_SystemReset(); // 시스템 리셋
    } 

    else if (input.equalsIgnoreCase("2")) {
        // CONTROL_NUM = 1;
        CONTROL_FLAG = EXECUTE1;
    } 
    else if (input.equalsIgnoreCase("4")) {
        // CONTROL_NUM = 3; // Traj
        CONTROL_FLAG = EXECUTE3;
    } 
    else if (input.equalsIgnoreCase("5")) {
        // CONTROL_NUM = 4; // Traj
        CONTROL_FLAG = EXECUTE4;
    } 
    else if (input.equalsIgnoreCase("3")) {
        // CONTROL_NUM = 2; // Traj
        CONTROL_FLAG = EXECUTE2;
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
    else if (input.startsWith("u1_max=")) {
        double val = input.substring(7).toFloat();
        CoNAC_Params::u1_max = val;
    }
    else if (input.startsWith("u2_max=")) {
        double val = input.substring(7).toFloat();
        CoNAC_Params::u2_max = val;
    }
    else if (input.startsWith("w1=")) {
        double val = input.substring(3).toFloat();
        CoNAC_Params::w1 = val;
    }
    else if (input.startsWith("w2=")) {
        double val = input.substring(3).toFloat();
        CoNAC_Params::w2 = val;
    }
    else if (input.startsWith("B=") || input.startsWith("Lambda_arr=") ||
            input.startsWith("th_max=") || input.startsWith("beta=")) {
        
        String varName = input.substring(0, input.indexOf('='));
        String data = input.substring(varName.length() + 1);

        data.replace("[", "");
        data.replace("]", "");

        if (varName == "B" || varName == "Lambda_arr") {
            int rowIdx = data.indexOf(';');
            if (rowIdx != -1) {
                String row1 = data.substring(0, rowIdx);
                String row2 = data.substring(rowIdx + 1);

                int col1 = row1.indexOf(',');
                int col2 = row2.indexOf(',');

                if (col1 != -1 && col2 != -1) {
                    float a11 = row1.substring(0, col1).toFloat();
                    float a12 = row1.substring(col1 + 1).toFloat();
                    float a21 = row2.substring(0, col2).toFloat();
                    float a22 = row2.substring(col2 + 1).toFloat();

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

        else if (varName == "beta") {
            int sep1 = data.indexOf(';');
            int sep2 = data.indexOf(';', sep1 + 1);
            if (sep1 != -1 && sep2 != -1) {
                CoNAC_Params::beta[0] = data.substring(0, sep1).toFloat();
                CoNAC_Params::beta[1] = data.substring(sep1 + 1, sep2).toFloat();
                CoNAC_Params::beta[2] = data.substring(sep2 + 1).toFloat();
            }
        }
    }

}






