import os
import json
import csv
import subprocess
from PyQt5.QtWidgets import (
    QMainWindow, QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
    QGroupBox, QTextEdit, QPushButton, QLineEdit, QScrollArea, 
    QLabel, QFileDialog
    )
from PyQt5.QtCore import Qt, QTimer, pyqtSlot
from serial_handler import *
from serial_handler import real_time_data 
from real_graph import RealTimeGraph 
from real_graph import TimeSeriesGraph
from real_graph import TauPlotGraph
import time
import pyqtgraph as pg
from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QOpenGLContext


class RobotArmControl(QMainWindow):
    def __init__(self):
        super().__init__()
        self.is_recording = False
        self.record_path = ""
        self.csv_file = None
        self.csv_writer = None

        self.timer = None
        self.parameters = {}
        self.dynamic_widgets = {}

        # ✅ 100ms마다 데이터 저장을 체크하는 타이머 추가
        self.record_timer = QTimer()
        self.record_timer.timeout.connect(self.record_data)

        # ✅ FPS 계산을 위한 변수 초기화 (🔹 중요)
        self.frame_count = 0  # ✅ 1초 동안 업데이트된 프레임 수
        self.last_time = time.perf_counter()  # ✅ 마지막 업데이트 시간

        self.can_freq_labels = {}  # ✅ CAN 주파수 표시를 위한 QLabel 저장
        self.gain_labels = {}
        self.init_ui()

        # ✅ UI 인스턴스를 serial_handler에 등록
        register_ui_instance(self)
        
    def init_ui(self):
        self.setWindowTitle("Robot Arm Control")
        # self.setGeometry(100, 100, 1200, 800)
        self.setStyleSheet("""
            QMainWindow { background-color: white; } 
            QPushButton { background-color: none; } 
            QLineEdit { background-color: none; }
            QTextEdit { background-color: none; }
            QComboBox { background-color: none; }
        """)

        # ✅ 현재 화면 해상도를 가져와 자동 크기 조정
        screen = QApplication.primaryScreen()
        screen_geometry = screen.availableGeometry()
        screen_width = screen_geometry.width()
        screen_height = screen_geometry.height()

        # ✅ 화면 크기에 맞춰 윈도우 크기 설정 (예: 전체 화면의 70% 사용)
        self.resize(int(screen_width * 0.7), int(screen_height * 0.7))
        self.move(int(screen_width * 0.15), int(screen_height * 0.15))

        # 메인 위젯 및 레이아웃 설정
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # 좌측 패널
        left_panel = QVBoxLayout()
        main_layout.addLayout(left_panel, 1)
        self.create_connect_section(left_panel)
        self.create_button_section(left_panel)
        self.create_fps_section(left_panel)
        self.create_can_freq_section(left_panel)
        self.create_gain_section(left_panel) 
        register_gain_callback(self.update_gain_display)
        self.create_logging_section(left_panel)
        self.create_command_section(left_panel)
        self.create_recording_section(left_panel)
        self.create_parameter_section(left_panel)
        self.create_graph_section(main_layout)

        register_can_freq_callback(self.update_can_freq_display)


    def create_connect_section(self, parent_layout):
        """포트 & 속도 입력 UI 추가 (왼쪽 패널)"""
        connect_layout = QVBoxLayout()

        # ✅ 포트 입력 필드
        port_layout = QHBoxLayout()
        self.port_input = QLineEdit()
        self.port_input.setPlaceholderText("COM10 또는 /dev/ttyUSB0")
        port_label = QLabel("Port:")
        port_layout.addWidget(port_label)
        port_layout.addWidget(self.port_input)

        # ✅ 속도 입력 필드
        baud_layout = QHBoxLayout()
        self.baud_input = QLineEdit()
        self.baud_input.setPlaceholderText("예: 115200")
        baud_label = QLabel("Baud Rate:")
        baud_layout.addWidget(baud_label)
        baud_layout.addWidget(self.baud_input)
        # UI 배치
        connect_layout.addLayout(port_layout)
        connect_layout.addLayout(baud_layout)
        parent_layout.addLayout(connect_layout)  # 왼쪽 패널에 추가


    def create_button_section(self, parent_layout):
        """버튼 그룹을 생성하여 UI에 추가"""
        button_grid = QGridLayout()
        parent_layout.addLayout(button_grid)

        buttons = [
            ("Connect", lambda: self.connect_and_start_graph()), 
            ("Reboot", lambda: reboot_opencr(self.log_text)),
            ("", None), ("", None), 
            ("Set Origin1", lambda: set_origin(self.log_text, 1)),
            ("Set Origin2", lambda: set_origin(self.log_text, 2)),
            ("FT Bias", lambda: ft_bias_update(self.log_text)),
            ("", None), 
            ("Standby", lambda: standby(self.log_text)),
            ("Execute", lambda: execute(self.log_text)),
        ]

        for i, (text, callback) in enumerate(buttons):
            self.add_button(button_grid, text, i // 2, i % 2, callback)



    def update_gain_display(self, gain_values):
        for key, vals in gain_values.items():
            text = f"{key} = {vals}"
            if key in self.gain_labels:
                self.gain_labels[key].setText(text)
            else:
                new_label = QLabel(text, self)
                new_label.setStyleSheet("font-size: 12px;")
                self.gain_labels[key] = new_label
                self.gain_group.layout().addWidget(new_label)


    def create_gain_section(self, layout):

        self.gain_group = QWidget()
        gain_layout = QVBoxLayout(self.gain_group)
        gain_layout.setAlignment(Qt.AlignTop)
        layout.addWidget(self.gain_group)

        self.gain_title = QLabel("PARAMETERS (Hz):", self)
        self.gain_title.setStyleSheet("font-weight: bold; font-size: 12px;")
        gain_layout.addWidget(self.gain_title)


        gain_keys = ["u_ball", "u_max", "alp", "beta", "th_max", "Lambda_arr", "A_zeta", "th_arr", "ctrltime", "CONTROL_FLAG"]

        for key in gain_keys:
            self.gain_labels[key] = QLabel(f"{key} = []")
            self.gain_labels[key].setStyleSheet("font-size: 12px;")
            gain_layout.addWidget(self.gain_labels[key])


    def create_can_freq_section(self, layout):
        """CAN 데이터 주파수를 표시하는 UI 생성"""
        self.can_freq_group = QWidget()
        can_freq_layout = QVBoxLayout(self.can_freq_group)
        can_freq_layout.setAlignment(Qt.AlignTop)
        layout.addWidget(self.can_freq_group)

        self.can_freq_title = QLabel("CAN Data Frequencies (Hz):", self)
        self.can_freq_title.setStyleSheet("font-weight: bold; font-size: 12px;")
        can_freq_layout.addWidget(self.can_freq_title)

        for i in range(5):  # ✅ CAN ID 5개 기준 (0, 1, 2, 3,4)
            label = QLabel(f"ID {i}: - Hz", self)
            label.setStyleSheet("font-size: 12px;")
            can_freq_layout.addWidget(label)
            self.can_freq_labels[i] = label

    def update_can_freq_display(self, can_freq_data):
        """실시간 CAN 주파수를 UI에 업데이트"""
        for i in can_freq_data.keys():  # ✅ 수신된 ID만 처리
            if i in self.can_freq_labels:  # ✅ 존재하는 ID만 UI에 업데이트
                self.can_freq_labels[i].setText(f"ID {i}: {can_freq_data[i]} Hz")
            else:
                log(None, f"⚠️ Warning: Received unexpected CAN ID {i}, ignoring it.")  # ✅ 디버깅 추가


    def create_fps_section(self, parent_layout):
        """FPS & GPU 상태 표시 UI 생성 (버튼 아래, 로그 창 위)"""
        self.fps_label = QLabel("FPS: - / GPU: OFF", self)
        self.fps_label.setStyleSheet("font-size: 12px; font-weight: bold; color: black;")
        self.fps_label.setAlignment(Qt.AlignCenter)  # ✅ 중앙 정렬

        fps_container = QWidget()
        fps_layout = QVBoxLayout(fps_container)
        fps_layout.addWidget(self.fps_label)
        fps_layout.setContentsMargins(0, 5, 0, 5)  # ✅ 여백 추가
        fps_container.setLayout(fps_layout)

        parent_layout.addWidget(fps_container)  # ✅ FPS 표시를 left_panel에 추가

        # ✅ FPS 업데이트를 위한 타이머 추가 (1000ms = 1초마다 실행)
        self.fps_timer = QTimer()
        self.fps_timer.timeout.connect(self.update_fps)  # ✅ FPS 업데이트 함수 연결
        self.fps_timer.start(1000)  # ✅ 1초마다 FPS 계산

    def create_logging_section(self, parent_layout):
        """로그 출력을 위한 UI 섹션 생성"""
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        parent_layout.addWidget(self.log_text)

    def create_command_section(self, parent_layout):
        """명령어 입력 및 전송을 위한 UI 섹션 생성"""
        command_layout = QHBoxLayout()
        parent_layout.addLayout(command_layout)

        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("Enter command...")
        command_layout.addWidget(self.command_input)

        send_command_button = QPushButton("Send")
        send_command_button.clicked.connect(self.send_command)
        command_layout.addWidget(send_command_button)

    def create_recording_section(self, parent_layout):
        """녹화 기능을 위한 UI 섹션 생성"""
        record_frame = QGroupBox("Record")
        record_layout = QVBoxLayout()
        record_frame.setLayout(record_layout)
        parent_layout.addWidget(record_frame)

        record_path_layout = QHBoxLayout()
        record_layout.addLayout(record_path_layout)

        # 폴더 선택 버튼 
        select_folder_button = QPushButton("...")
        select_folder_button.clicked.connect(self.select_folder)
        record_path_layout.addWidget(select_folder_button, 1)

        # 시작 버튼 
        start_button = QPushButton("Start")
        start_button.clicked.connect(self.start_recording)
        record_path_layout.addWidget(start_button, 1)

        # 정지 버튼 
        stop_button = QPushButton("Stop")
        stop_button.clicked.connect(self.stop_recording)
        record_path_layout.addWidget(stop_button, 1)

    def create_parameter_section(self, parent_layout):
        """파라미터 설정을 위한 UI 섹션 생성"""
        parameter_frame = QGroupBox("Parameter Settings")
        parameter_layout = QVBoxLayout()
        parameter_frame.setLayout(parameter_layout)
        parent_layout.addWidget(parameter_frame)

        parameter_button_layout = QHBoxLayout()
        parameter_layout.addLayout(parameter_button_layout)

        load_button = QPushButton("Load")
        load_button.clicked.connect(self.load_parameters)
        parameter_button_layout.addWidget(load_button)

        set_button = QPushButton("Set")
        set_button.clicked.connect(self.set_parameters)
        parameter_button_layout.addWidget(set_button)

        self.parameter_scroll_area = QScrollArea()
        self.parameter_scroll_area.setWidgetResizable(True)
        self.parameter_widget = QWidget()
        self.parameter_layout = QVBoxLayout(self.parameter_widget)
        self.parameter_scroll_area.setWidget(self.parameter_widget)
        parameter_layout.addWidget(self.parameter_scroll_area)

    def create_graph_section(self, main_layout):
        """그래프 UI 섹션 생성"""
        graph_container = QGridLayout()
        main_layout.addLayout(graph_container, 8)

        # ✅ 5개의 TimeSeriesGraph와 1개의 RealTimeGraph 생성
        self.time_series_graph1 = TimeSeriesGraph(default_selection="q")     # 1번
        self.real_time_graph = RealTimeGraph(self)                           # 3번 (RealTimeGraph)
        self.time_series_graph3 = TimeSeriesGraph(default_selection="Fext")  # 4번
        self.time_series_graph4 = TimeSeriesGraph(default_selection="u")     # 5번
        self.tau_time_graph5 = TauPlotGraph(self) # 6번


        # ✅ 위치 설정 (2x3 그리드)
        graph_container.addWidget(self.time_series_graph1, 0, 0,1,2)  # 1번
        graph_container.addWidget(self.real_time_graph, 0, 2)     # 3번 (RealTimeGraph)
        graph_container.addWidget(self.time_series_graph3, 1, 0)  # 4번
        graph_container.addWidget(self.time_series_graph4, 1, 1)  # 5번
        graph_container.addWidget(self.tau_time_graph5, 1, 2)  # 6번

        # ✅ 클릭 이벤트 연결 (RealTimeGraph만)
        self.real_time_graph.clicked.connect(self.on_graph_click)


    def add_button(self, layout, text, row, col, callback):
        """버튼 추가 함수"""
        button = QPushButton(text)
        if callback:  # 버튼 이벤트 콜백이 있는 경우
            button.clicked.connect(callback)  # ✅ 올바르게 콜백 함수 연결
        else:
            button.setEnabled(False)  # 빈칸 버튼은 비활성화
        layout.addWidget(button, row, col)

    def update_fps(self):
        """1초마다 현재 프레임 카운트를 FPS로 업데이트"""
        fps = self.frame_count  # ✅ 현재 1초 동안의 프레임 수가 곧 FPS 값
        self.frame_count = 0  # 🔹 프레임 카운트 초기화 (1초마다 초기화됨)

        gpu_status = "ON" if QOpenGLContext.currentContext() is not None else "OFF"
        self.fps_label.setText(f"FPS: {fps:.2f} / GPU: {gpu_status}")  # ✅ FPS + GPU 상태 표시

    def send_command(self):
        """명령어 입력창에서 시리얼 명령을 전송하는 함수"""
        ser = get_serial_instance()  # ✅ 전역 ser 가져오기
        command = self.command_input.text().strip()  # 입력된 명령어 가져오기

        if command:
            try:
                if ser and ser.is_open:
                    ser.write(f"{command}\n".encode('utf-8'))  # 시리얼 포트를 통해 전송
                    ser.flush()
                    log(self.log_text, f"Command sent: {command}")  # 로그 기록
                else:
                    log(self.log_text, "⚠️ Serial port is not open. Please connect first.")  
            except Exception as e:
                log(self.log_text, f"❌ Failed to send command: {e}")  # 오류 로그 기록
            finally:
                self.command_input.clear()  # 입력창 초기화

    def on_graph_click(self, x, y):
        """클릭된 그래프 좌표를 로그 출력 및 시리얼 명령 전송"""
        log(self.log_text, f"Clicked point: Y = {x:.3f}, Z = {y:.3f}")

        # xd 명령어 생성
        command = f"xd=[{x:.3f};{y:.3f}]"

        # 시리얼 포트를 통해 전송
        try:
            ser = get_serial_instance()
            if ser and ser.is_open:
                ser.write(f"{command}\n".encode('utf-8'))
                ser.flush()
                log(self.log_text, f"Command sent: {command}")
            else:
                log(self.log_text, "⚠️ Serial port is not open.")
        except Exception as e:
            log(self.log_text, f"❌ Failed to send command: {e}")

    def select_folder(self):
        """폴더 선택 대화상자를 통해 기록 경로를 설정하거나 기본 경로로 시작."""
        # 기본 경로 설정
        default_path = r"C:\Users\fairy\Desktop\work\robotarm"

        # 폴더 선택 대화상자 열기 (기본 경로로 시작)
        folder = QFileDialog.getExistingDirectory(self, "Select Folder", default_path)
        if folder:
            self.record_path = folder  # 선택된 폴더 경로를 저장
            log(self.log_text, f"Selected folder: {self.record_path}")
        else:
            # 사용자가 취소를 누를 경우, 기본 경로로 설정
            self.record_path = default_path
            log(self.log_text, "No folder selected. Using default path.")

    @pyqtSlot()
    def start_recording(self):
        """실시간 데이터를 CSV 파일에 기록 시작."""
        if not self.is_recording:
            if not hasattr(self, 'record_path') or not self.record_path:
                log(self.log_text, "Please select a folder to save the recording.")
                return

            csv_file_path = os.path.join(self.record_path, "data.csv")

            try:
                self.csv_file = open(csv_file_path, mode="w", newline="")
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(real_time_data.keys())  # 헤더 작성
                self.is_recording = True
                log(self.log_text, f"Recording started. Saving to {csv_file_path}")

                # ✅ 타이머 시작 (100ms마다 `record_data()` 실행)
                self.record_timer.start(100)

            except Exception as e:
                log(self.log_text, f"Failed to start recording: {e}")

    @pyqtSlot()
    def stop_recording(self):
        """CSV 파일에 데이터 저장 종료 및 파라미터 저장."""
        if self.is_recording:
            self.is_recording = False
            self.record_timer.stop()  # ✅ 타이머 중지

            if self.csv_file:
                self.csv_file.close()

                # ✅ Adjust the Time column to start from 0
                try:
                    csv_file_path = os.path.join(self.record_path, "data.csv")
                    with open(csv_file_path, mode="r") as file:
                        reader = list(csv.reader(file))
                        headers = reader[0]
                        rows = reader[1:]

                    if "Time" in headers:
                        time_index = headers.index("Time")
                        initial_time = float(rows[0][time_index])  # Get the first Time value
                        for row in rows:
                            row[time_index] = str(float(row[time_index]) - initial_time)  # Adjust Time

                        # Save the adjusted data back to the CSV file
                        with open(csv_file_path, mode="w", newline="") as file:
                            writer = csv.writer(file)
                            writer.writerow(headers)
                            writer.writerows(rows)

                        log(self.log_text, "Recording stopped. Data saved successfully with adjusted Time.")
                    else:
                        log(self.log_text, "⚠️ 'Time' column not found in CSV. No adjustment made.")
                except Exception as e:
                    log(self.log_text, f"Error adjusting Time column: {e}")

                # Plot.py 호출
                try:
                    current_dir = os.path.dirname(os.path.abspath(__file__))
                    plot_script_path = os.path.join(current_dir, "plot.py")
                    subprocess.run(['python', plot_script_path, csv_file_path], check=True)
                    log(self.log_text, "Plots generated and saved successfully.")
                except Exception as e:
                    log(self.log_text, f"Error generating plots: {e}")

            else:
                log(self.log_text, "No recording in progress. Nothing to stop.")

    def record_data(self):
        """실시간 데이터를 CSV 파일에 기록 (100ms마다 실행)"""
        if self.is_recording and self.csv_writer:
            try:
                # ✅ 가장 최신 데이터를 CSV에 기록
                self.csv_writer.writerow([real_time_data[key][-1] for key in real_time_data.keys()])
            except Exception as e:
                print(f"⚠️ CSV Write Error: {e}")

    def load_parameters(self):
        """JSON 파일에서 파라미터를 로드."""
        file_path, _ = QFileDialog.getOpenFileName(self, "Load Parameters", "", "JSON Files (*.json)")
        if file_path:
            try:
                with open(file_path, 'r') as json_file:
                    self.parameters = json.load(json_file)  # 파라미터를 로드
                log(self.log_text, f"Parameters loaded from {file_path}")
                self.update_parameter_gui()  # GUI 업데이트
            except Exception as e:
                log(self.log_text, f"Failed to load parameters: {e}")

    def set_parameters(self):
        """GUI에서 수정된 파라미터를 JSON 파일로 저장."""
        if not hasattr(self, 'record_path') or not self.record_path:
            log(self.log_text, "Please select a folder to save the parameters.")
            return

        # JSON 파일 경로 생성
        parameter_save_path = os.path.join(self.record_path, "parameters.json")

        try:
            # 동적 위젯에서 값을 추출해 self.parameters에 반영
            for key, widget in self.dynamic_widgets.items():
                if isinstance(widget, QLineEdit):  # 일반 입력 필드 처리
                    category, param = key.split(".")
                    value = widget.text().strip()
                    self.parameters[category][param] = float(value) if value.replace('.', '', 1).isdigit() else value
                elif isinstance(widget, QTextEdit):  # 멀티라인 텍스트 처리
                    self.parameters[key] = widget.toPlainText().strip()

            # JSON 파일로 저장
            os.makedirs(self.record_path, exist_ok=True)  # 디렉토리 생성
            with open(parameter_save_path, 'w', encoding='utf-8') as json_file:
                json.dump(self.parameters, json_file, indent=4, ensure_ascii=False)
            log(self.log_text, f"Parameters saved to {parameter_save_path}")
        except Exception as e:
            log(self.log_text, f"Failed to save parameters: {e}")

    def update_parameter_gui(self):
        """불러온 파라미터를 GUI에 반영."""
        if not isinstance(self.parameters, dict):  # self.parameters가 딕셔너리인지 확인
            log(self.log_text, "Invalid parameters format. Expected a dictionary.")
            return

        # 기존 위젯 제거
        self.dynamic_widgets.clear()  # 동적 위젯 딕셔너리 초기화
        for i in reversed(range(self.parameter_layout.count())):
            widget = self.parameter_layout.itemAt(i).widget()
            if widget:
                widget.deleteLater()

        # 카테고리별로 파라미터 표시
        for category, params in self.parameters.items():
            category_label = QLabel(f"<b>{category}</b>")  # 카테고리 이름 표시
            self.parameter_layout.addWidget(category_label)

            if isinstance(params, dict):  # 딕셔너리일 경우
                for key, value in params.items():
                    row = QWidget()
                    row_layout = QHBoxLayout(row)
                    row_layout.addWidget(QLabel(f"{key}:"))

                    # 숫자 및 일반 문자열 처리
                    entry = QLineEdit(str(value))
                    row_layout.addWidget(entry)
                    self.dynamic_widgets[f"{category}.{key}"] = entry

                    row.setLayout(row_layout)
                    self.parameter_layout.addWidget(row)

            elif isinstance(params, str):  # 긴 문자열 또는 멀티라인 텍스트 처리
                row = QWidget()
                row_layout = QVBoxLayout(row)

                # 멀티라인 텍스트
                text_edit = QTextEdit(params)
                row_layout.addWidget(text_edit)
                self.dynamic_widgets[f"{category}"] = text_edit

                row.setLayout(row_layout)
                self.parameter_layout.addWidget(row)

            else:
                log(self.log_text, f"Unsupported parameter type in category '{category}': {type(params)}")

    def update_graphs(self):
        """실시간 그래프 업데이트 + 실행 시간 측정"""
        # start_time = time.perf_counter()  # ✅ 시작 시간 기록

        self.frame_count += 1
        self.time_series_graph1.update_graph()
        # self.time_series_graph2.update_graph()
        self.real_time_graph.update_graph()
        self.time_series_graph3.update_graph()
        self.time_series_graph4.update_graph()
        self.tau_time_graph5.update_graph()
        # elapsed_time = (time.perf_counter() - start_time) * 1000  # ✅ 실행 시간(ms)
        # print(f"🔍 update_graphs 실행 시간: {elapsed_time:.2f} ms")  # ✅ 실행 시간 출력


    def connect_and_start_graph(self):
        """시리얼 연결 후 그래프 업데이트 시작"""

        port = self.port_input.text().strip()
        baud_rate = self.baud_input.text().strip()

        if not port or not baud_rate:
            log(self.log_text, f"⚠️ 포트와 속도를 입력하세요!")
            return

        try:
            baud_rate = int(baud_rate)  # 속도 값을 정수로 변환
            connect_serial(port, baud_rate, self.log_text)  # ✅ serial_handler에 전달
        except ValueError:
            log(self.log_text, f"⚠️ 유효한 숫자로 속도를 입력하세요!")
        except Exception as e:
            log(self.log_text, f"❌ Serial 연결 실패: {e}")

        if self.timer is None:
            self.timer = QTimer()
            self.timer.timeout.connect(self.update_graphs)
        self.timer.start(50)






