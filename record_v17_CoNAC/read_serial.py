import serial
import serial.tools.list_ports
import threading
import csv

# CSV 파일 경로와 이름 설정
CSV_FILE_PATH = r"C:\Users\fairy\Desktop\data\sim_result\th_max10_Lambda420.csv"

SERIAL_PORT = 'COM10'  # 원하는 포트로 변경
BAUD_RATE = 115200     # 원하는 속도로 변경
ser = None
csv_file = None
csv_writer = None
recording = False


def get_available_ports():
    """현재 사용 가능한 COM 포트 목록 반환"""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def connect_serial(port, baud_rate):
    """시리얼 포트 연결"""
    global ser
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
    except serial.SerialException as e:
        ser = None

def start_csv_logging(filename=CSV_FILE_PATH):
    """CSV 파일 열기 및 준비"""
    global csv_file, csv_writer, recording
    try:
        csv_file = open(filename, mode='w', newline='', encoding='utf-8')
        csv_writer = csv.writer(csv_file)
        # csv_writer.writerow(["Data"])  # 헤더 작성
        recording = True

    except Exception as e:
        print(f"⚠️ Error opening CSV file: {e}")

def stop_csv_logging():
    """CSV 파일 닫기"""
    global csv_file, recording
    if csv_file:
        csv_file.close()
    recording = False

def read_serial_data():
    """시리얼 데이터를 읽어서 출력 및 CSV에 저장"""
    global ser, csv_writer, recording
    if not ser or not ser.is_open:
        print("❌ Serial port is not open.")
        return

    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(f"{line}")  # 데이터를 콘솔에 출력
                if recording and csv_writer:
                    csv_writer.writerow([line])  # 데이터 저장
    except KeyboardInterrupt:
        print("\n🛑 Stopping Serial Monitor.")
    except Exception as e:
        print(f"⚠️ Error reading serial data: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("🔌 Serial port closed.")
        stop_csv_logging()

def send_serial_command():
    """사용자 입력을 받아 시리얼 포트로 명령 전송 및 기록 제어"""
    global ser
    if not ser or not ser.is_open:
        print("❌ Serial port is not open.")
        return

    try:
        while True:
            command = input("Enter command (r: start recording, e: stop recording, or type 'exit' to quit): ")
            if command.lower() == 'exit':
                print("🛑 Exiting command sender.")
                break
            elif command.lower() == 'r':
                start_csv_logging(CSV_FILE_PATH)
            elif command.lower() == 'e':
                stop_csv_logging()
            else:
                ser.write(f"{command}\n".encode('utf-8'))
                print(f"📤 Sent: {command}")
    except KeyboardInterrupt:
        print("\n🛑 Stopping command sender.")
    except Exception as e:
        print(f"⚠️ Error sending serial command: {e}")

if __name__ == "__main__":
    # 사용 가능한 포트 출력
    print("Available Ports:", get_available_ports())

    # 시리얼 포트 연결
    connect_serial(SERIAL_PORT, BAUD_RATE)

    # 데이터 읽기 및 명령 전송을 병렬로 실행
    if ser and ser.is_open:
        print("📡 Starting Serial Monitor. Press Ctrl+C to stop.")
        read_thread = threading.Thread(target=read_serial_data, daemon=True)
        read_thread.start()

        # 사용자 입력을 통해 명령 전송 및 기록 제어
        send_serial_command()