import serial
import serial.tools.list_ports
import threading
import traceback  
from PyQt5.QtCore import QMetaObject, Qt, QObject  # Ensure PyQt5 is imported for thread-safe UI calls

SERIAL_PORT = 'COM10'
BAUD_RATE = 115200
ser = None
ui_instance = None  # RobotArmControl 인스턴스를 저장

# 실시간 데이터 구조
real_time_data = {key: [] for key in (
    ['Time','CONTROL_FLAG',
     'q1', 'q2', 'qdot1', 'qdot2',
     'r1', 'r2', 'rdot1', 'rdot2',
     'u1','u2', 'u_sat1', 'u_sat2', 
     'lbd1','lbd2', 'lbd3', 'lbd4',
     'lbd5', 'lbd6', 'lbd7', 'lbd8',
     'Vn1', 'Vn2', 'Vn3',
     'zeta1', 'zeta2',
     'ctrltime',
     'A_zeta', 'beta', 
    ]
)}

previous_values = {"CONTROL_FLAG": None, "A_zeta": None, "beta": None}

def check_and_log_changes(log_widget):
    """CONTROL_FLAG, A_zeta, beta 값 변경 시 로그 출력"""
    global real_time_data, previous_values

    gain_data = {}
    for key in ["CONTROL_FLAG", "A_zeta", "beta"]:
        if real_time_data[key]:
            current_value = real_time_data[key][-1]
            if previous_values[key] != current_value:
                log(log_widget, f"{key} {previous_values[key]} -> {current_value}")
                previous_values[key] = current_value
    if update_gain_ui and gain_data:
        update_gain_ui(gain_data)


update_can_freq_ui = None
update_gain_ui = None


def register_gain_callback(callback):

    """UI에서 Gain 값을 업데이트할 콜백 함수 등록"""
    global update_gain_ui
    update_gain_ui = callback


def register_can_freq_callback(callback):
    """UI에서 CAN 주파수를 업데이트할 콜백 함수 등록"""
    global update_can_freq_ui
    update_can_freq_ui = callback
    
def register_ui_instance(ui):
    """UI 인스턴스를 등록하여 Start/Stop Recording 호출 가능"""
    global ui_instance
    ui_instance = ui

def send_serial_command(command: str, log_widget=None):
    """시리얼 포트로 명령을 전송하는 함수"""
    global ser
    if ser and ser.is_open:
        try:
            ser.write(f"{command}\n".encode('utf-8'))
            ser.flush()
            if log_widget:
                log_widget.append(f"Command sent: {command}")
        except Exception as e:
            if log_widget:
                log_widget.append(f"Serial command error: {e}")

def get_available_ports():
    """현재 사용 가능한 COM 포트 목록 반환"""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]  # COM 포트 이름 리스트 반환


def connect_serial(port, baud_rate, log_widget=None):
    """GUI에서 입력받은 포트와 속도로 시리얼 연결"""
    global ser
    
    available_ports = get_available_ports()
    if port not in available_ports:
        log(log_widget, f"❌ Port {port} not found! Available ports: {', '.join(available_ports)}")
        return  # 잘못된 포트 입력 시 연결 시도하지 않음

    try:
        if ser and ser.is_open:
            ser.close()
        ser = serial.Serial(port, baud_rate, timeout=1)  # ✅ 입력된 포트 & 속도 사용
        read_thread = threading.Thread(target=read_data, args=(log_widget,), daemon=True)
        read_thread.start()

        if ser and ser.is_open:
            actual_baud = ser.baudrate
            log(log_widget, f"✅ Connected to {port} with requested {actual_baud} baud")
        else:
            log(log_widget, "⚠️ Failed to open serial port.")
    except serial.SerialException as e:
        log(log_widget, f"❌ Serial connection failed: {e}")
        ser = None
    except ValueError as e:
        log(log_widget, f"⚠️ Invalid baud rate or port: {e}")
    except OSError as e:
        log(log_widget, f"⚠️ OS Error: {e}")
    except Exception as e:
        log(log_widget, f"⚠️ Unexpected error: {e}")

def get_serial_instance():
    """현재 시리얼 포트 객체 반환"""
    global ser
    return ser

def read_data(log_widget=None):
    """시리얼 데이터를 지속적으로 읽고 real_time_data에 저장"""
    global ser, real_time_data, ui_instance
    while True:
        if ser is None or not ser.is_open:
            if log_widget:
                log(log_widget, "⚠️ Serial port is closed. Stopping read_data() thread.")
            break

        try:
            line = ser.readline().decode("utf-8").strip()
            if not line:
                continue  
            # ✅ 1. CAN 데이터 처리 ("ID:"로 시작하는 메시지)
            if line.startswith("ID:"):
                process_can_freq_data(line)
                continue

            # ✅ 2. 숫자 배열 (탭으로 구분된 실시간 데이터)
            if "\t" in line:
                data = line.split("\t")
                try:
                    time_val = float(data[0])
                    values = list(map(float, data[1:]))
                except ValueError as e:
                    if log_widget:
                        log(log_widget, f"⚠️ 데이터 변환 오류: {data}, 오류: {e}")
                    continue

                real_time_data["Time"].append(time_val)
                keys = list(real_time_data.keys())[1:]

                if len(values) < len(keys):
                    if log_widget:
                        log(log_widget, f"⚠️ 데이터 길이 부족: {data}")
                    continue

                for i, key in enumerate(keys):
                    real_time_data[key].append(values[i])
                limit_data_length()

                # ✅ CONTROL_FLAG와 ctrltime 값을 UI로 직접 전달
                if update_gain_ui:
                    gain_data = {
                        "CONTROL_FLAG": real_time_data["CONTROL_FLAG"][-1],
                    }
                    update_gain_ui(gain_data)

                check_and_log_changes(log_widget)
                continue

            # ✅ 3. Gain 관련 메시지 ("u_ball" 또는 "alp"로 시작)
            if line.startswith("u_ball") or line.startswith("ctrl_wrapper"):
                process_gain_data(line)
                continue


        except serial.SerialException as e:
            if log_widget:
                log(log_widget, f"❌ Serial Exception: {e}")
            break  # 시리얼 오류 발생 시 종료
        
        except IndexError as e:
            if log_widget:
                log(log_widget, f"⚠️ 데이터 리스트 인덱스 오류: {e}, 데이터: {data}")
            print(f"⚠️ IndexError in read_data: {e}, 데이터: {data}")
            continue  # ⚠️ 오류 발생 시 다음 데이터 읽기 시도
        
        except Exception as e:
            error_details = traceback.format_exc()  # 전체 오류 스택 출력
            if log_widget:
                log(log_widget, f"⚠️ Unexpected error in read_data(): {error_details}")
            print(f"⚠️ Unexpected error in read_data(): {error_details}")  # ✅ 터미널에도 출력
            continue  # ⚠️ 알 수 없는 오류 발생 시에도 다시 읽기 시도

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def parse_extra_params(line):
    """
    문자열을 파싱하여 {키: [값, ...]} 형태의 딕셔너리로 반환합니다.
    예) "ctrl_wrapper 0.003262 ExecuteTime 0.000000" -> 
         { "ctrl_wrapper": [0.003262], "ExecuteTime": [0.0] }
    """
    tokens = line.split()
    extra_dict = {}
    i = 0
    while i < len(tokens):
        key = tokens[i]
        i += 1
        vals = []
        while i < len(tokens) and is_number(tokens[i]):
            vals.append(float(tokens[i]))
            i += 1
        extra_dict[key] = vals
    return extra_dict

def limit_data_length():
    """데이터 길이를 제한하여 메모리 사용을 줄이는 함수"""
    max_length = 1000
    for key in real_time_data:
        if len(real_time_data[key]) > max_length:
            real_time_data[key].pop(0)

def process_can_freq_data(line):
    """CAN 주파수 데이터 파싱 및 UI 업데이트"""
    global update_can_freq_ui

    # ✅ "ID:"를 기준으로 분할 (예: "ID:0 500 ID:1 500" → ["0 500", "1 500"])
    parts = line.split("ID:")
    
    can_freq_data = {}  # CAN 주파수 저장용 딕셔너리

    for part in parts:
        if not part.strip():  # 빈 문자열 방지
            continue

        items = part.strip().split()  # ID와 주파수 분리 (예: "0 500" → ["0", "500"])

        if len(items) != 2:
            # log(None, f"⚠️ Warning: Unexpected CAN frequency data format: {line}")
            return  # ⚠️ 형식이 맞지 않으면 무시

        try:
            can_id = int(items[0])  # CAN ID 추출
            freq = int(items[1])  # 주파수 값 추출
            can_freq_data[can_id] = freq  # 딕셔너리에 저장
        except ValueError:
            log(None, f"⚠️ Warning: ValueError in CAN frequency parsing: {items}")
            return  # ⚠️ 변환 오류 발생 시 무시

    # ✅ UI 업데이트 함수 호출
    if update_can_freq_ui:
        log(None,f"Updating UI with CAN frequency data: {can_freq_data}")  # 디버깅 메시지
        update_can_freq_ui(can_freq_data)

def process_gain_data(line):
    """Gain 관련 파라미터 파싱 후 UI 업데이트"""
    gain_data = parse_extra_params(line)
    if gain_data and update_gain_ui:
        update_gain_ui(gain_data)


# ----------- 유틸리티 함수 -----------

def log(widget, message: str):
    """로그를 기록하는 함수"""
    try:
        if widget is None:
            print(message)  # ✅ 위젯이 없을 경우 콘솔 출력
        else:
            widget.append(message)  # ✅ 위젯이 있으면 UI에 출력
    except Exception as e:
        print(f"⚠️ Error in log function: {e}")  # 오류 발생 시 콘솔에 출력

def reboot_opencr(log_widget):
    """OpenCR 리부팅 명령 전송"""
    send_serial_command("REBOOT", log_widget)
    log(log_widget, "Reboot command sent.")
    clear_real_time_data()

def clear_real_time_data():
    """실시간 데이터 초기화"""
    global real_time_data
    for key in real_time_data:
        real_time_data[key].clear()


def ft_bias_update(log_widget):
    """FT Bias 업데이트 명령 전송"""
    send_serial_command("FT_BIAS", log_widget)
    log(log_widget, "FT Bias Update command sent.")

def set_origin(log_widget, origin_id):
    """원점 설정 명령 전송"""
    send_serial_command(f"origin:{origin_id}", log_widget)
    log(log_widget, f"Set Origin {origin_id} command sent.")

def standby(log_widget):
    """스탠바이 명령 전송"""
    send_serial_command("STANDBY", log_widget)
    log(log_widget, "Standby command sent.")

def execute(log_widget):
    """실행 명령 전송"""
    send_serial_command("EXECUTE", log_widget)
    log(log_widget, "Execute command sent.")


__all__ = [
    "connect_serial",
    "get_serial_instance",
    "send_serial_command",
    "reboot_opencr",
    "ft_bias_update",
    "set_origin",
    "standby",
    "execute",
    "log",
    "register_can_freq_callback",
    "register_gain_callback",
    "register_ui_instance"
]
