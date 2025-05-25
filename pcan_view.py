import can
import threading
import queue
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import time
import numpy as np
from PyQt5.QtWidgets import QGraphicsEllipseItem

# === Queue 생성 ===
can_data_queue = queue.Queue()

# === CAN 인터페이스 설정 ===
bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000)

class CanReceiver(can.Listener):
    def on_message_received(self, msg):
        if msg.arbitration_id in (3, 4, 5, 8, 0x00002901, 0x00002902):  # 필요한 모든 ID 추가
            can_data_queue.put(msg)
        

# Listener 등록
notifier = can.Notifier(bus, [CanReceiver()], timeout=0.1)  # Increase timeout to 100ms

# === PyQt 애플리케이션 생성 ===
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="CAN Real-Time Plot")
fk_win = pg.GraphicsLayoutWidget(title="2D Forward Kinematics (Large)")

data_r = [[] for _ in range(4)]
time_r = [[] for _ in range(4)]
data_motor1 = [[] for _ in range(3)]
time_motor1 = [[] for _ in range(3)]
data_motor2 = [[] for _ in range(3)]
time_motor2 = [[] for _ in range(3)]
data_v = [[] for _ in range(4)]
time_v = [[] for _ in range(4)]

# === Plot 설정 ===
# 1. q1, q2, r1, r2
plot_q = win.addPlot(title="q1, q2, r1, r2")
# plot_q.setXRange(0, 60)  # X축 범위 고정
plot_q.setYRange(-3, 3)  # Y축 범위 고정
plot_q.showGrid(x=True, y=True)   
curves_q = [
    plot_q.plot(pen=pg.mkPen('g', width=1)),
    plot_q.plot(pen=pg.mkPen('b', width=1)),
    plot_q.plot(pen=pg.mkPen('g', width=1, style=QtCore.Qt.DotLine)),
    plot_q.plot(pen=pg.mkPen('b', width=1, style=QtCore.Qt.DotLine))
]

# 2. qdot1, qdot2, rdot1, rdot2
win.nextRow()
plot_qdot = win.addPlot(title="qdot1, qdot2, rdot1, rdot2")
plot_qdot.setYRange(-3, 3)  # Y축 범위 고정
plot_qdot.showGrid(x=True, y=True)   
curves_qdot = [
    plot_qdot.plot(pen=pg.mkPen('g', width=1)),
    plot_qdot.plot(pen=pg.mkPen('b', width=1)),
    plot_qdot.plot(pen=pg.mkPen('g', width=1, style=QtCore.Qt.DotLine)),
    plot_qdot.plot(pen=pg.mkPen('b', width=1, style=QtCore.Qt.DotLine))
]

# 3. v1, v2, v3, v4
win.nextRow()
plot_v = win.addPlot(title="v1, v2, v3, v4")
plot_v.setYRange(-3, 3)  # Y축 범위 고정
plot_v.showGrid(x=True, y=True)   
plot_v.addLegend() 
curves_v = [
    plot_v.plot(pen=pg.mkPen('r', width=2), name='v1'),
    plot_v.plot(pen=pg.mkPen('g', width=2), name='v2'),
    plot_v.plot(pen=pg.mkPen('b', width=2), name='v3'),
    plot_v.plot(pen=pg.mkPen('y', width=2), name='v4')
]

# === 4.  robot arm animation 2D Plot 추가 ===
plot_fk = fk_win.addPlot(title="2D Forward Kinematics")
fk_win.resize(600, 600)
plot_fk.setLabel('bottom', 'X', units='m')  # X축 레이블 설정  
plot_fk.setLabel('left', 'Y', units='m')  # Y축 레이블 설정
plot_fk.setAspectLocked(True)  # x축, y축 스케일 동일하게
plot_fk.showGrid(x=True, y=True)    
plot_fk.setXRange(-0.3, 1.0)  # Y축 범위 고정
plot_fk.setYRange(-0.5, 0.4)  # Y축 범위 고정

# 링크 길이 설정
l1, l2 = 0.2, 0.2  # 링크 길이  
# 링크를 나타내는 선 추가
link1 = pg.PlotDataItem(pen=pg.mkPen('w', width=4))  # 첫 번째 링크
link2 = pg.PlotDataItem(pen=pg.mkPen('w', width=4))  # 두 번째 링크
plot_fk.addItem(link1)
plot_fk.addItem(link2)

# 관절을 나타내는 점 추가
joint1 = pg.ScatterPlotItem(pen=pg.mkPen('w'), brush=pg.mkBrush('w'), size=8)
joint2 = pg.ScatterPlotItem(pen=pg.mkPen('w'), brush=pg.mkBrush('w'), size=8)
end_effector = pg.ScatterPlotItem(pen=pg.mkPen('w'), brush=pg.mkBrush('w'), size=8)
plot_fk.addItem(joint1)
plot_fk.addItem(joint2)
plot_fk.addItem(end_effector)

# 4. u1, u2
win.nextRow()
plot_u = win.addPlot(title="u1, u2, ctrl_flag")
curves_u = [
    plot_u.plot(pen=pg.mkPen('g', width=2)),
    plot_u.plot(pen=pg.mkPen('b', width=2)),
    plot_u.plot(pen=pg.mkPen('y', width=2, style=QtCore.Qt.DotLine))  # ctrl_flag
]
plot_u.setYRange(-20, 20)  # Y축 범위 고정
data_u = [[] for _ in range(3)]
time_u = [[] for _ in range(3)]

start_time = time.time()  # 시작할 때 저장

# === Unpack 함수 ===
def unpack_var4(data):
    if len(data) != 8:
        raise ValueError(f"Invalid data length: {len(data)}")
    def twos_complement(value, bits):
        if value & (1 << (bits - 1)):
            value -= (1 << bits)
        return value
    var1 = twos_complement((data[0] << 8) | data[1], 16) / 1000.0
    var2 = twos_complement((data[2] << 8) | data[3], 16) / 1000.0
    var3 = twos_complement((data[4] << 8) | data[5], 16) / 1000.0
    var4 = twos_complement((data[6] << 8) | data[7], 16) / 1000.0
    return var1, var2, var3, var4

def unpack_state(data):
    if len(data) != 8:
        raise ValueError(f"Invalid data length: {len(data)}")
    
    def twos_complement(value, bits):
        if value & (1 << (bits - 1)):
            value -= (1 << bits)
        return value
    
    pos_int = twos_complement((data[0] << 8) | data[1], 16)  # 2의 보수 적용
    spd_int = twos_complement((data[2] << 8) | data[3], 16)  # 2의 보수 적용
    cur_int = twos_complement((data[4] << 8) | data[5], 16)  # 2의 보수 적용
    
    q = (float(pos_int * 0.1) * (np.pi / 180.0))  # rad
    qdot = (float(spd_int * 10.0 / (9.0 * 21.0)) * (2 * np.pi / 60.0))  # rad/s
    tau = (float(1 / 0.75 * cur_int * 0.01))  # N*m
    return q, qdot, tau

# === Plot 업데이트 함수 ===
def update_plot():
    while not can_data_queue.empty():
        msg = can_data_queue.get()
        timestamp = time.time() - start_time  # 상대 시간으로 변환

        try:
            if msg.arbitration_id == 3:
                r1, r2, rdot1, rdot2 = unpack_var4(msg.data)
                data_r[0].append(r1); time_r[0].append(timestamp)
                data_r[1].append(r2); time_r[1].append(timestamp)
                data_r[2].append(rdot1); time_r[2].append(timestamp)
                data_r[3].append(rdot2); time_r[3].append(timestamp)

            elif msg.arbitration_id == 4:
                v1, v2, v3, v4 = unpack_var4(msg.data)
                data_v[0].append(v1); time_v[0].append(timestamp)
                data_v[1].append(v2); time_v[1].append(timestamp)
                data_v[2].append(v3); time_v[2].append(timestamp)
                data_v[3].append(v4); time_v[3].append(timestamp)

            elif msg.arbitration_id == 5:
                u1, u2, ctrl_flag = unpack_var4(msg.data)[:3]
                data_u[0].append(u1); time_u[0].append(timestamp)
                data_u[1].append(u2); time_u[1].append(timestamp)
                data_u[2].append(ctrl_flag); time_u[2].append(timestamp)  # ctrl_flag

            elif msg.arbitration_id == 0x00002901:
                q1, qdot1, tau1 = unpack_state(msg.data)
                data_motor1[0].append(q1); time_motor1[0].append(timestamp)
                data_motor1[1].append(qdot1); time_motor1[1].append(timestamp)
                data_motor1[2].append(tau1); time_motor1[2].append(timestamp)  # tau1

            elif msg.arbitration_id == 0x00002902:
                q2, qdot2, tau2 = unpack_state(msg.data)
                data_motor2[0].append(q2); time_motor2[0].append(timestamp)
                data_motor2[1].append(qdot2); time_motor2[1].append(timestamp)
                data_motor2[2].append(tau2); time_motor2[2].append(timestamp)  # tau2

            # elif msg.arbitration_id == 8:
            #     q1,qdot1,q2,qdot2 = unpack_var4(msg.data)
            #     data_motor1[0].append(q1); time_motor1[0].append(timestamp)
            #     data_motor1[1].append(qdot1); time_motor1[1].append(timestamp)
            #     data_motor2[0].append(q2); time_motor2[0].append(timestamp)  # tau1
            #     data_motor2[1].append(qdot2); time_motor2[1].append(timestamp)
        except ValueError as e:
            print(f"[Warning] ID={msg.arbitration_id}, length={len(msg.data)}, raw data={msg.data.hex()}")
            continue

    # 슬라이딩 윈도우
    max_window = 10000

    def update_curves(curves, time_list, data_list):
        for i, curve in enumerate(curves):
            ydata = data_list[i][-max_window:]
            xdata = time_list[i][-max_window:]
            curve.setData(x=xdata, y=ydata)

    # curves_q: [q1, q2, r1, r2]
    update_curves(
        curves_q,
        [time_motor1[0], time_motor2[0], time_r[0], time_r[1]],
        [data_motor1[0], data_motor2[0], data_r[0], data_r[1]]
    )
    # curves_qdot: [qdot1, qdot2, rdot1, rdot2]
    update_curves(
        curves_qdot,
        [time_motor1[1], time_motor2[1], time_r[2], time_r[3]],
        [data_motor1[1], data_motor2[1], data_r[2], data_r[3]]
    )

    # curves_u: [u1, u2]
    update_curves(curves_u, time_u, data_u)

    # curves_v: [v1, v2, v3, v4]
    update_curves(curves_v, time_v, data_v)


def update_robot_arm():
    if len(data_r[0]) > 0 and len(data_r[1]) > 0:
        # 참조 모델의 관절 각도 (r1, r2)
        r1 = data_r[0][-1]
        r2 = data_r[1][-1]
        # 참조 모델의 Forward Kinematics 계산
        x1_ref = l1 * np.cos(r1)
        y1_ref = l1 * np.sin(r1)
        x2_ref = x1_ref + l2 * np.cos(r1 + r2)
        y2_ref = y1_ref + l2 * np.sin(r1 + r2)

        # 참조 모델 업데이트 (길이 2 체크)
        if not any(np.isnan([x1_ref, x2_ref, y1_ref, y2_ref])):
            ref_link1.setData([0, x1_ref], [0, y1_ref])
            ref_link2.setData([x1_ref, x2_ref], [y1_ref, y2_ref])


    if len(data_motor1[0]) > 0 and len(data_motor2[0]) > 0:
        # 실제 로봇팔의 관절 각도 (q1, q2)
        q1 = data_motor1[0][-1]
        q2 = data_motor2[0][-1]

        # 실제 로봇팔의 Forward Kinematics 계산
        x1_actual = l1 * np.cos(q1)
        y1_actual = l1 * np.sin(q1)
        x2_actual = x1_actual + l2 * np.cos(q1 + q2)
        y2_actual = y1_actual + l2 * np.sin(q1 + q2)

        # 실제 로봇팔 업데이트 (길이 2 체크)
        if not any(np.isnan([x1_actual, x2_actual, y1_actual, y2_actual])):
            link1.setData([0, x1_actual], [0, y1_actual])
            link2.setData([x1_actual, x2_actual], [y1_actual, y2_actual])
            joint1.setData([x1_actual], [y1_actual])
            joint2.setData([x2_actual], [y2_actual])
            end_effector.setData([x2_actual], [y2_actual])


# === 참조 모델을 위한 선 추가 ===
ref_link1 = pg.PlotDataItem(pen=pg.mkPen('r', width=4, style=QtCore.Qt.DotLine))  # 참조 첫 번째 링크
ref_link2 = pg.PlotDataItem(pen=pg.mkPen('r', width=4, style=QtCore.Qt.DotLine))  # 참조 두 번째 링크
plot_fk.addItem(ref_link1)
plot_fk.addItem(ref_link2)

# === Timer를 이용해 주기적으로 업데이트 ===
def update_all():
    update_plot()  # 기존 플롯 업데이트
    update_robot_arm()  # 로봇팔 애니메이션 업데이트

timer = QtCore.QTimer()
timer.timeout.connect(update_all)
timer.start(20)  # 50Hz (20ms마다 업데이트)


# === GUI 실행 ===
win.show()
fk_win.show()
app.exec_()
