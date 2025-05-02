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
        if msg.arbitration_id in (3, 4, 5):
            can_data_queue.put(msg)

# Listener 등록
notifier = can.Notifier(bus, [CanReceiver()], timeout=0.1)  # Increase timeout to 100ms

# === PyQt 애플리케이션 생성 ===
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="CAN Real-Time Plot")

# === Plot 설정 ===
# 1. q1, q2, r1, r2
plot_q = win.addPlot(title="q1, q2, r1, r2")
# plot_q.setXRange(0, 60)  # X축 범위 고정
plot_q.setYRange(-2, 2)  # Y축 범위 고정
curves_q = [
    plot_q.plot(pen=pg.mkPen('g', width=2)),
    plot_q.plot(pen=pg.mkPen('b', width=2)),
    plot_q.plot(pen=pg.mkPen('g', width=2, style=QtCore.Qt.DotLine)),
    plot_q.plot(pen=pg.mkPen('b', width=2, style=QtCore.Qt.DotLine))
]
data_q = [[] for _ in range(4)]
time_q = [[] for _ in range(4)]


# === 6.  robot arm animation 2D Plot 추가 ===
win.nextColumn()
plot_fk = win.addPlot(title="2D Forward Kinematics")
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
plot_u = win.addPlot(title="u1, u2")
curves_u = [
    plot_u.plot(pen=pg.mkPen('r', width=2)),
    plot_u.plot(pen=pg.mkPen('b', width=2))
]
plot_u.setYRange(-20, 20)  # Y축 범위 고정
data_u = [[] for _ in range(2)]
time_u = [[] for _ in range(2)]

# === 5. u1-u2 2D Plot 추가 ===
win.nextColumn()
plot_u2d = win.addPlot(title="u1 vs u2")
plot_u2d.setLabel('bottom', 'u1', units='')
plot_u2d.setLabel('left', 'u2', units='')
plot_u2d.setAspectLocked(True)  # x축, y축 스케일 동일하게
plot_u2d.showGrid(x=True, y=True)

# u_ball 원, 고정선 추가
u_ball = 11.0
u2_max = 3.5
x_limit = (u_ball**2 - u2_max**2) ** 0.5

# 원
circle = QGraphicsEllipseItem(-u_ball, -u_ball, 2*u_ball, 2*u_ball)
circle.setPen(pg.mkPen('w', style=QtCore.Qt.DotLine))
plot_u2d.addItem(circle)


# 수평선 (y=u2_max)
hline = pg.InfiniteLine(pos=u2_max, angle=0, pen=pg.mkPen('w'))
plot_u2d.addItem(hline)

# 수직선 (x=±sqrt(u_ball^2 - u2_max^2))
vline1 = pg.InfiniteLine(pos=x_limit, angle=90, pen=pg.mkPen('w'))
vline2 = pg.InfiniteLine(pos=-x_limit, angle=90, pen=pg.mkPen('w'))
plot_u2d.addItem(vline1)
plot_u2d.addItem(vline2)

# 실시간 점 (scatter)
line = pg.PlotDataItem(pen=pg.mkPen('r', width=2))  # 실시간 선 추가
plot_u2d.addItem(line)
scatter = pg.ScatterPlotItem(pen=pg.mkPen('r'), brush=pg.mkBrush('w'), size=1)
plot_u2d.addItem(scatter)
plot_u2d.setYRange(-1, 6)  # Y축 범위 고정
plot_u2d.setXRange(-1, 12)  # Y축 범위 고정
# 데이터 버퍼
data_u2d = []


# 3. th1, th2, th3
win.nextRow()
plot_th = win.addPlot(title="th1, th2, th3")
plot_th.setYRange(-1, 10)  # Y축 범위 고정
curves_th = [
    plot_th.plot(pen=pg.mkPen('r', width=2)),
    plot_th.plot(pen=pg.mkPen('g', width=2)),
    plot_th.plot(pen=pg.mkPen('b', width=2))
]
data_th = [[] for _ in range(3)]
time_th = [[] for _ in range(3)]

# 2. lbd4, lbd6, lbd8
win.nextColumn()
plot_lbd = win.addPlot(title="lbd4, lbd6, lbd8")
curves_lbd = [
    plot_lbd.plot(pen=pg.mkPen('r', width=2)),
    plot_lbd.plot(pen=pg.mkPen('g', width=2)),
    plot_lbd.plot(pen=pg.mkPen('b', width=2))
]

data_lbd = [[] for _ in range(3)]
time_lbd = [[] for _ in range(3)]

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

def unpack_var5(data):
    if len(data) != 8:
        raise ValueError(f"Invalid data length: {len(data)}")
    def twos_complement(value, bits):
        if value & (1 << (bits - 1)):
            value -= (1 << bits)
        return value
    # var1 = twos_complement(((data[0] << 4) | (data[1] >> 4)) & 0xFFF, 12) / 1000.0
    # var2 = twos_complement(((data[1] & 0x0F) << 8) | data[2], 12) / 1000.0
    # var3 = twos_complement(((data[3] << 4) | (data[4] >> 4)) & 0xFFF, 12) / 1000.0
    # var4 = twos_complement(((data[4] & 0x0F) << 8) | data[5], 12) / 1000.0

    def int_to_float(value, min, max, bits):
        span = max - min
        return (value * span / (2047) + min + max)/2.0

    var1 = int_to_float((data[0] << 4) | (data[1] >> 4), -1.5, 1.5, 12)    
    var2 = int_to_float(((data[1] & 0x0F) << 8) | data[2], -3, 1.5, 12)
    var3 = int_to_float(((data[3] << 4) | (data[4] >> 4)) & 0xFFF, -1.5, 1.5, 12)
    var4 = int_to_float(((data[4] & 0x0F) << 8) | data[5], -3, 1.5, 12)

    var5 = twos_complement((data[6] << 8) | data[7], 16) / 1000.0
    return var1, var2, var3, var4, var5

def unpack_var5_2(data):
    if len(data) != 8:
        raise ValueError(f"Invalid data length: {len(data)}")
    def twos_complement(value, bits):
        if value & (1 << (bits - 1)):
            value -= (1 << bits)
        return value
    var1 = twos_complement(((data[0] << 4) | (data[1] >> 4)) & 0xFFF, 12) / 1000.0
    var2 = twos_complement(((data[1] & 0x0F) << 8) | data[2], 12) / 1000.0
    var3 = twos_complement(((data[3] << 4) | (data[4] >> 4)) & 0xFFF, 12) / 1000.0
    var4 = twos_complement(((data[4] & 0x0F) << 8) | data[5], 12) / 1000.0
    var5 = twos_complement((data[6] << 8) | data[7], 16) / 1000.0
    return var1, var2, var3, var4, var5

# === Plot 업데이트 함수 ===
def update_plot():
    while not can_data_queue.empty():
        msg = can_data_queue.get()
        timestamp = time.time() - start_time  # 상대 시간으로 변환

        try:
            if msg.arbitration_id == 3:
                q1, q2, r1, r2, lbd3 = unpack_var5(msg.data)
                data_q[0].append(q1); time_q[0].append(timestamp)
                data_q[1].append(q2); time_q[1].append(timestamp)
                data_q[2].append(r1); time_q[2].append(timestamp)
                data_q[3].append(r2); time_q[3].append(timestamp)
                data_lbd[0].append(np.log(1+lbd3)); time_lbd[0].append(timestamp)

            elif msg.arbitration_id == 4:
                lbd5, lbd7, u1, u2 = unpack_var4(msg.data)
                data_lbd[1].append(np.log(1+lbd5)); time_lbd[1].append(timestamp)
                data_lbd[2].append(np.log(1+lbd7)); time_lbd[2].append(timestamp)
                data_u[0].append(u1); time_u[0].append(timestamp)
                data_u[1].append(u2); time_u[1].append(timestamp)

            elif msg.arbitration_id == 5:
                th1, th2, th3, ctrl_flag, ctrl_time = unpack_var5_2(msg.data)
                data_th[0].append(th1); time_th[0].append(timestamp)
                data_th[1].append(th2); time_th[1].append(timestamp)
                data_th[2].append(th3); time_th[2].append(timestamp)

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

    update_curves(curves_q, time_q, data_q)
    update_curves(curves_lbd, time_lbd, data_lbd)
    update_curves(curves_th, time_th, data_th)
    update_curves(curves_u, time_u, data_u)

        # === u1-u2 Scatter 업데이트 ===
    if len(data_u[0]) >= 1 and len(data_u[1]) >= 1:
        u1_val = data_u[0][-1]
        u2_val = data_u[1][-1]
        data_u2d.append((u1_val, u2_val))

        # 버퍼를 20개로 유지
        if len(data_u2d) > 20:
            data_u2d.pop(0)

        scatter.setData([p[0] for p in data_u2d], [p[1] for p in data_u2d])

        # 실시간 선 업데이트
        if len(data_u2d) > 1:
            line.setData([p[0] for p in data_u2d], [p[1] for p in data_u2d])

def update_robot_arm():
    if len(data_q[0]) > 0 and len(data_q[1]) > 0:
        # 실제 로봇팔의 관절 각도 (q1, q2)
        q1 = data_q[0][-1]
        q2 = data_q[1][-1]

        # 참조 모델의 관절 각도 (r1, r2)
        r1 = data_q[2][-1] if len(data_q[2]) > 0 else 0
        r2 = data_q[3][-1] if len(data_q[3]) > 0 else 0

        # 실제 로봇팔의 Forward Kinematics 계산
        x1_actual = l1 * np.cos(q1)
        y1_actual = l1 * np.sin(q1)
        x2_actual = x1_actual + l2 * np.cos(q1 + q2)
        y2_actual = y1_actual + l2 * np.sin(q1 + q2)

        # 참조 모델의 Forward Kinematics 계산
        x1_ref = l1 * np.cos(r1)
        y1_ref = l1 * np.sin(r1)
        x2_ref = x1_ref + l2 * np.cos(r1 + r2)
        y2_ref = y1_ref + l2 * np.sin(r1 + r2)

        # 실제 로봇팔 업데이트
        link1.setData([0, x1_actual], [0, y1_actual])  # 첫 번째 링크
        link2.setData([x1_actual, x2_actual], [y1_actual, y2_actual])  # 두 번째 링크
        joint1.setData([x1_actual], [y1_actual])  # 첫 번째 관절
        joint2.setData([x2_actual], [y2_actual])  # 두 번째 관절
        end_effector.setData([x2_actual], [y2_actual])  # 끝단

        # 참조 모델 업데이트 (선 스타일을 점선으로 설정)
        ref_link1.setData([0, x1_ref], [0, y1_ref])  # 첫 번째 링크
        ref_link2.setData([x1_ref, x2_ref], [y1_ref, y2_ref])  # 두 번째 링크

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
app.exec_()
