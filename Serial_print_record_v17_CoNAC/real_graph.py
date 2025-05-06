import pyqtgraph as pg
import numpy as np
from pyqtgraph.Qt import QtWidgets, QtCore
from serial_handler import real_time_data
from PyQt5.QtGui import QPolygonF, QBrush, QColor, QPen
from PyQt5.QtWidgets import QComboBox, QGraphicsLineItem,QGraphicsPolygonItem
from PyQt5.QtWidgets import QGraphicsRectItem, QGraphicsTextItem
from PyQt5 import QtGui
from PyQt5.QtCore import Qt, QPointF

class RealTimeGraph(QtWidgets.QWidget):
    clicked = QtCore.pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()


    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 10, 20, 0)

        # ✅ OpenGL을 사용하도록 설정
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setAntialiasing(True)  # ✅ 부드러운 선 적용
        self.graph_widget.useOpenGL(True)  # ✅ 개별 Plot에서 OpenGL 활성화

        # self.graph_widget.setFixedSize(500, 500)
        layout.addWidget(self.graph_widget)

        self.graph_widget.showGrid(x=True, y=True, alpha=0.3)  # 🔹 눈금 표시

        self.graph_widget.showAxes(True, showValues=True, size=50) 
        self.graph_widget.setBackground('w')
        self.graph_widget.getViewBox().setAspectLocked(True)

        # ✅ ViewBox 테두리도 굵게 설정 (그래프 전체 박스)
        self.graph_widget.getViewBox().setBorder(pg.mkPen('black', width=1))

        # ✅ X, Y 라벨 설정 (Bold 적용)
        font = QtGui.QFont("Arial", 12, QtGui.QFont.Bold)  # ✅ 볼드 폰트 설정
        self.graph_widget.setLabel('left', '<b>Z (m)</b>', color='black', size='12pt')  # ✅ HTML 태그 사용
        self.graph_widget.setLabel('bottom', '<b>Y (m)</b>', color='black', size='12pt')  # ✅ HTML 태그 사용

        # ✅ Bold 스타일 적용 (QFont 사용)
        self.graph_widget.getAxis("left").setStyle(tickFont=font)
        self.graph_widget.getAxis("bottom").setStyle(tickFont=font)

        # ✅ 축 색상 설정
        self.graph_widget.getAxis("left").setPen(pg.mkPen('black'))
        self.graph_widget.getAxis("bottom").setPen(pg.mkPen('black'))

        self.graph_widget.setRange(xRange=[-0.4, 0.4], yRange=[-0.4,0.4])


        # ✅ 작업 범위 원 초기화
        l1, l2 = 0.2, 0.16  # 링크 길이 (고정된 값)
        outer_radius = l1 + l2
        inner_radius = abs(l1 - l2)

        theta = np.linspace(0, 2 * np.pi, 100)
        outer_x, outer_y = outer_radius * np.cos(theta), outer_radius * np.sin(theta)
        inner_x, inner_y = inner_radius * np.cos(theta), inner_radius * np.sin(theta)

        self.outer_circle = self.graph_widget.plot(outer_x, outer_y, pen=pg.mkPen(QColor('gray'), width=2, style=Qt.DashLine))
        self.inner_circle = self.graph_widget.plot(inner_x, inner_y, pen=pg.mkPen(QColor('gray'), width=2, style=Qt.DashLine))


        # 링크와 조인트 점 초기화
        self.link_plot = self.graph_widget.plot([], [], pen=pg.mkPen('k', width=10))
        self.joint1_plot = self.graph_widget.plot([], [], pen=None, symbol='o', symbolBrush='b', symbolSize=30)
        self.joint2_plot = self.graph_widget.plot([], [], pen=None, symbol='o', symbolBrush='b', symbolSize=30)

        # 목표 위치, 궤적
        self.xd_plot = self.graph_widget.plot(
            [], [], 
            pen=None, 
            symbol='o', 
            symbolBrush=None,  # 내부를 투명하게 설정
            symbolPen=pg.mkPen('r', width=3),  # 붉은색 외곽선 추가
            symbolSize=10
        )
        

        # Reference (기준) 조인트 궤적 추가
        self.ref_q1_plot = self.graph_widget.plot([], [], pen=pg.mkPen('red', width=3, style=QtCore.Qt.DotLine))
        self.ref_q2_plot = self.graph_widget.plot([], [], pen=pg.mkPen('red', width=3, style=QtCore.Qt.DotLine))

        # ✅ 클릭 이벤트 연결
        self.graph_widget.scene().sigMouseClicked.connect(self.on_graph_click)


    def update_graph(self):
        """실시간 그래프 업데이트"""

        if len(real_time_data['q1']) == 0 or len(real_time_data['q2']) == 0:
            return        
        q1, q2 = real_time_data['q1'][-1], real_time_data['q2'][-1]

        l1, l2 = 0.2, 0.18
        
        # 링크 좌표 계산
        y1 = l1 * np.cos(q1)
        z1 = l1 * np.sin(q1)
        y2 = y1 + l2 * np.cos(q1 + q2)
        z2 = z1 + l2 * np.sin(q1 + q2)


        # 링크 업데이트
        self.link_plot.setData([0, y1, y2], [0, z1, z2])
        
        # 조인트 점 업데이트
        self.joint1_plot.setData([y1], [z1])
        self.joint2_plot.setData([y2], [z2])


        # ✅ 목표 위치 xd 표시 (qd를 사용하여 forward kinematics로 계산)
        if 'qd1' in real_time_data and 'qd2' in real_time_data and len(real_time_data['qd1']) > 0 and len(real_time_data['qd2']) > 0:
            qd1, qd2 = real_time_data['qd1'][-1], real_time_data['qd2'][-1]
            xd1 = l1 * np.cos(qd1) + l2 * np.cos(qd1 + qd2)
            xd2 = l1 * np.sin(qd1) + l2 * np.sin(qd1 + qd2)
            self.xd_plot.setData([xd1], [xd2])

        # 🔹 기준(reference) 조인트 위치 계산
        if 'r1' in real_time_data and 'r2' in real_time_data and len(real_time_data['r1']) > 0 and len(real_time_data['r2']) > 0:
            r1, r2 = real_time_data['r1'][-1], real_time_data['r2'][-1]
            ry1 = l1 * np.cos(r1)
            rz1 = l1 * np.sin(r1)
            ry2 = ry1 + l2 * np.cos(r1 + r2)
            rz2 = rz1 + l2 * np.sin(r1 + r2)

            # ✅ 기준(reference) 조인트 궤적 업데이트
            self.ref_q1_plot.setData([0, ry1], [0, rz1])  # Reference Joint 1
            self.ref_q2_plot.setData([ry1, ry2], [rz1, rz2])  # Reference Joint 2              


    def on_graph_click(self, event):
        """그래프 클릭 이벤트를 감지하고 시그널을 전송"""
        pos = event.scenePos()
        if self.graph_widget.sceneBoundingRect().contains(pos):
            mouse_point = self.graph_widget.getViewBox().mapSceneToView(pos)
            x, y = mouse_point.x(), mouse_point.y()
            self.clicked.emit(x, y) 

class TauPlotGraph(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
        self.tau1_data = []
        self.tau2_data = []


    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 10, 20, 0)
        # Add a gray circle with radius 10 centered at the origin
        theta = np.linspace(0, 2 * np.pi, 100)
        radius = 11.5
        u1_max = 10.78
        u2_max = 4.0  # 추가된 u2_max 값

        circle_x = radius * np.cos(theta)
        circle_y = radius * np.sin(theta)

        # ✅ OpenGL을 사용하도록 설정
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setAntialiasing(True)  # ✅ 부드러운 선 적용
        self.graph_widget.useOpenGL(True)  # ✅ 개별 Plot에서 OpenGL 활성화

        layout.addWidget(self.graph_widget)

        self.graph_widget.showGrid(x=True, y=True, alpha=0.3)  # 🔹 눈금 표시
        self.graph_widget.setBackground('w')
        self.graph_widget.getViewBox().setAspectLocked(False)

        self.graph_widget.plot(circle_x, circle_y, pen=pg.mkPen(QColor('gray'), width=2, style=Qt.DashLine))
        
        # Ensure x and y axes have the same length
        self.graph_widget.getViewBox().setAspectLocked(True)

        # ✅ X, Y 라벨 설정
        font = QtGui.QFont("Arial", 12, QtGui.QFont.Bold)  # ✅ 볼드 폰트 설정
        self.graph_widget.setLabel('left', '<b>Tau2</b>', color='black', size='12pt')  # ✅ HTML 태그 사용
        self.graph_widget.setLabel('bottom', '<b>Tau1</b>', color='black', size='12pt')  # ✅ HTML 태그 사용

        # ✅ Bold 스타일 적용 (QFont 사용)
        self.graph_widget.getAxis("left").setStyle(tickFont=font)
        self.graph_widget.getAxis("bottom").setStyle(tickFont=font)

        # ✅ 축 색상 설정
        self.graph_widget.getAxis("left").setPen(pg.mkPen('black'))
        self.graph_widget.getAxis("bottom").setPen(pg.mkPen('black'))

        # Add two vertical dashed lines at x = -6.5 and x = 6.5
        line_pen = pg.mkPen(QColor('black'), width=1, style=Qt.DashLine)
        
        # 데이터 점 초기화 (파란 작은 점, 바깥 테두리 없음)
        self.scatter_plot = pg.ScatterPlotItem(size=4, brush=pg.mkBrush(0, 0, 255, 120), pen=None)
        self.graph_widget.addItem(self.scatter_plot)

        # Add the circle and vertical lines again to ensure they persist
        self.circle_item = self.graph_widget.plot(circle_x, circle_y, pen=pg.mkPen(QColor('gray'), width=2, style=Qt.DashLine))
        self.line_item1 = pg.InfiniteLine(pos=-u1_max, angle=90, pen=line_pen)
        self.line_item2 = pg.InfiniteLine(pos=u1_max, angle=90, pen=line_pen)
        self.line_item3 = pg.InfiniteLine(pos=-u2_max, angle=0, pen=line_pen)  # 추가된 y = -u2_max 직선
        self.line_item4 = pg.InfiniteLine(pos=u2_max, angle=0, pen=line_pen)   # 추가된 y = u2_max 직선
        self.graph_widget.addItem(self.line_item1)
        self.graph_widget.addItem(self.line_item2)
        self.graph_widget.addItem(self.line_item3)
        self.graph_widget.addItem(self.line_item4)


    def update_graph(self):
        """실시간 Tau1 vs Tau2 그래프 업데이트"""
        if 'CONTROL_FLAG' not in real_time_data or 'u1' not in real_time_data or 'u2' not in real_time_data:
            return

        CONTROL_FLAG = real_time_data['CONTROL_FLAG'][-1] if 'CONTROL_FLAG' in real_time_data and len(real_time_data['CONTROL_FLAG']) > 0 else None
        tau1 = real_time_data['u1'][-10:] if len(real_time_data['u1']) >= 10 else real_time_data['u1']
        tau2 = real_time_data['u2'][-10:] if len(real_time_data['u2']) >= 10 else real_time_data['u2']

        # Control flag가 2 또는 3일 때 데이터 누적
        if CONTROL_FLAG in [2,3,4,5,6]:
            self.tau1_data.append(tau1)
            self.tau2_data.append(tau2)

        # Control flag가 7 또는 8일 때 데이터 초기화
        elif CONTROL_FLAG in [0, 7, 8]:
            self.tau1_data = []
            self.tau2_data = []

        # 그래프 업데이트
        # Flatten the data before passing to setData
        tau1_flat = np.hstack(self.tau1_data) if self.tau1_data else []
        tau2_flat = np.hstack(self.tau2_data) if self.tau2_data else []

        # 점과 점을 선으로 연결 (최대 10개의 점만 유지)
        if len(tau1_flat) > 1 and len(tau2_flat) > 1:
            self.scatter_plot.setData(tau1_flat[-20:], tau2_flat[-20:])
        elif len(tau1_flat) == 1 and len(tau2_flat) == 1:
            # 단일 점만 있을 경우 점만 표시
            self.scatter_plot.setData([tau1_flat[0]], [tau2_flat[0]])
        if CONTROL_FLAG in [2, 3, 4, 5, 6]:
            self.tau1_data.append(tau1)
            self.tau2_data.append(tau2)

        # Control flag가 7 또는 8일 때 데이터 초기화
        elif CONTROL_FLAG in [0, 7, 8]:
            self.tau1_data = []
            self.tau2_data = []

        # 그래프 업데이트
        # Flatten the data before passing to setData
        tau1_flat = np.hstack(self.tau1_data) if self.tau1_data else []
        tau2_flat = np.hstack(self.tau2_data) if self.tau2_data else []

        # 그래프 초기화
        self.graph_widget.clear()

        # 원과 수직선 다시 추가
        self.graph_widget.addItem(self.circle_item)
        self.graph_widget.addItem(self.line_item1)
        self.graph_widget.addItem(self.line_item2)
        self.graph_widget.addItem(self.line_item3)
        self.graph_widget.addItem(self.line_item4)



        # 점과 점을 선으로 연결 (최대 10개의 점만 유지)
        if len(tau1_flat) > 1 and len(tau2_flat) > 1:
            self.graph_widget.plot(tau1_flat[-10:], tau2_flat[-10:], pen=pg.mkPen('blue', width=3.5))
        elif len(tau1_flat) == 1 and len(tau2_flat) == 1:
            # 단일 점만 있을 경우 점만 표시
            self.graph_widget.plot([tau1_flat[0]], [tau2_flat[0]], pen=None, symbol='o', symbolBrush='blue')

class TimeSeriesGraph(QtWidgets.QWidget):
    def __init__(self, parent=None, default_selection="q"):
        super().__init__(parent)
        self.line_styles = {
            # Desired values (점선)
            'qd1': {'color': 'red', 'linestyle': 'dashed'},
            'qd2': {'color': 'blue', 'linestyle': 'dashed'},
            'u1': {'color': 'red', 'linestyle': 'dashed'},
            'u2': {'color': 'blue', 'linestyle': 'dashed'},
            'u_sat1': {'color': 'red', 'linestyle': 'solid'},
            'u_sat2': {'color': 'blue', 'linestyle': 'solid'},

            'zeta1': {'color': 'red', 'linestyle': 'dashed'},
            'zeta2': {'color': 'blue', 'linestyle': 'dashed'},

            'qdotd1': {'color': 'red', 'linestyle': 'dashed'},
            'qdotd2': {'color': 'blue', 'linestyle': 'dashed'},
            'r1': {'color': 'red', 'linestyle': 'dashed'},
            'r2': {'color': 'blue', 'linestyle': 'dashed'},
            'rdot1': {'color': 'red', 'linestyle': 'dashed'},
            'rdot2': {'color': 'blue', 'linestyle': 'dashed'},

            # State values (실선)
            'q1': {'color': 'red', 'linestyle': 'solid'},
            'q2': {'color': 'blue', 'linestyle': 'solid'},
            'tau1': {'color': 'red', 'linestyle': 'solid'},
            'tau2': {'color': 'blue', 'linestyle': 'solid'},
            'qdot1': {'color': 'red', 'linestyle': 'solid'},
            'qdot2': {'color': 'blue', 'linestyle': 'solid'},
            'Vn1': {'color': 'red', 'linestyle': 'solid'},
            'Vn2': {'color': 'green', 'linestyle': 'solid'},
            'Vn3': {'color': 'blue', 'linestyle': 'solid'},
            'lbd1': {'color': 'red', 'linestyle': 'solid'},
            'lbd2': {'color': 'purple', 'linestyle': 'solid'},
            'lbd3': {'color': 'blue', 'linestyle': 'solid'},
            'lbd4': {'color': 'orange', 'linestyle': 'solid'},
            'lbd5': {'color': 'green', 'linestyle': 'solid'},
            'lbd6': {'color': 'black', 'linestyle': 'solid'}, 
            'lbd7': {'color': 'cyan', 'linestyle': 'solid'},
            'lbd8': {'color': 'magenta', 'linestyle': 'solid'},

            'ctrltime': {'color': 'magenta', 'linestyle': 'solid'},
        }

        self.init_ui(default_selection)


    def init_ui(self, default_selection):
        # ✅ 전체 레이아웃을 가로 정렬로 변경
        layout = QtWidgets.QHBoxLayout(self)

        # ✅ OpenGL을 사용하도록 설정
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setAntialiasing(True)  # ✅ 부드러운 선 적용
        self.graph_widget.useOpenGL(True)  # ✅ 개별 Plot에서 OpenGL 활성화       

        # ✅ 그래프 스타일 설정
        self.graph_widget.getViewBox().setMenuEnabled(False)  # 🔹 그래프 드래그 메뉴 비활성화
        self.graph_widget.getViewBox().setMouseEnabled(False, False)  # 🔹 x, y축 이동 방지
        self.graph_widget.getViewBox().setBackgroundColor('w')  # 🔹 배경을 흰색으로 유지
        self.graph_widget.showGrid(x=True, y=True, alpha=0.3)  # 🔹 눈금 표시
        self.graph_widget.showAxes(True, showValues=True, size=50) 
        self.graph_widget.setBackground('w')
        self.graph_widget.getViewBox().setBorder(pg.mkPen('black', width=1))
        
        # ✅ x축 및 y축 라벨 (글씨 크기 및 색상 변경)
        # self.graph_widget.setLabel('bottom', '<b>Time (s)</b>', color='black', size='12pt')
        font = QtGui.QFont("Arial", 10, QtGui.QFont.Bold)
        self.graph_widget.getAxis("left").setTickFont(font)
        self.graph_widget.getAxis("bottom").setTickFont(font)
        self.graph_widget.getAxis("left").setPen(pg.mkPen('black'))
        self.graph_widget.getAxis("bottom").setPen(pg.mkPen('black'))

        # ✅ 그래프를 왼쪽에 추가
        layout.addWidget(self.graph_widget, stretch=20)

        # ✅ 오른쪽 패널 생성 (Y 선택 드롭다운 + 범례 포함)
        self.right_panel = QtWidgets.QVBoxLayout()

        # ✅ Y축 선택 드롭다운 추가 (오른쪽 상단)
        self.dropdown = QComboBox()
        self.dropdown.addItems([
            'q', 
            'qdot',
            'zeta',
            'u','u1', 'u2',
            'Vn',
            'lbd',
            'ctrltime'
        ])
        self.dropdown.setCurrentText(default_selection)
        self.dropdown.currentTextChanged.connect(self.on_selection_change)
        self.right_panel.addWidget(self.dropdown)

        # ✅ 오른쪽 패널을 전체 레이아웃에 추가
        layout.addLayout(self.right_panel, stretch=1)
        self.setLayout(layout) 

        self.data_plots = {}
        self.update_graph()


    def on_selection_change(self, selection):
        print(f"🔄 Dropdown 변경됨: {selection}")
        self.update_graph()  # self.update_graphs()가 아니라 현재 그래프만 갱신

    def update_graph(self):
        """실시간 데이터 업데이트"""
        self.graph_widget.clear()

        # ✅ 항상 드롭다운에서 현재 선택된 값 사용
        selected_key = self.dropdown.currentText()

        # ✅ 선택된 값에 따른 Y축 범위 설정
        y_ranges = {
            'q1': (-3, 3),
            'q2': (-3, 3),
            'q': (-3, 2),
            'qdot': (-10, 10),
            'qdot1': (-10, 10),
            'qdot2': (-10, 10),
            'x1': (0.0, 0.4),
            'x2': (0.0, 0.4),
            'x': (0.0, 0.4),
            'yz': (-0.4, 0.4),
            'u': (-20, 20),
            'tau': (-10, 10),
            'Vn': (0, 20),
            'lbd': (0, 20),
            'ctrltime': (0, 5e-3),
            'd_hat': (-5, 5),
            'Z' : (-5, 5),
        }

        # ✅ ViewBox 설정: 마우스 휠로 Y축 확대/축소 가능하게 설정
        viewbox = self.graph_widget.getViewBox()
        viewbox.setMouseEnabled(x=False, y=True)  # 🔹 Y축 마우스 휠 줌 활성화
        viewbox.disableAutoRange('y')  # 🔹 Y축 자동 조정 비활성화 (휠 조작을 위해 필요)

        # ✅ 선택된 키가 존재하면 Y축 초기 범위 설정 (단, 이후 강제 변경하지 않음)
        if selected_key in y_ranges:
            if not hasattr(self, "y_range_initialized") or not self.y_range_initialized:
                viewbox.setYRange(*y_ranges[selected_key], padding=0.1)
                self.y_range_initialized = True  # ✅ 초기화 이후에는 변경하지 않음

        paired_selections = {
            'q': ['r1', 'q1', 'r2', 'q2'],
            'qdot': ['rdot1', 'qdot1', 'rdot2', 'qdot2'],
            'zeta': ['zeta1', 'zeta2'],
            'd_hat': ['d_hat1', 'd_hat2'],
            'Z': ['Z1', 'Z2'],
            'x': ['xd1', 'x1', 'xd2', 'x2'],
            'yz': ['x1', 'x2'],
            'u': ['u1','u_sat1','u2','u_sat2'],
            'u1': ['u1','u_sat1'],
            'u2': ['u2','u_sat2'],
            'lbd': ['lbd1', 'lbd2', 'lbd3','lbd4','lbd5','lbd6','lbd7','lbd8'],
            'Vn': ['Vn1', 'Vn2','Vn3'],
            'ctrltime': ['ctrltime'],  # Add ctrltime to paired_selections
        }
        # ✅ ctrltime 데이터를 10^-6 단위에서 10^6 단위로 변환
        # if selected_key == 'ctrltime' and 'ctrltime' in real_time_data:
        #     real_time_data['ctrltime'] = [value * 1e3 for value in real_time_data['ctrltime']]
        if selected_key in paired_selections:
            for data_key in paired_selections[selected_key]:
                if data_key in real_time_data and len(real_time_data[data_key]) > 0:
                    color = self.line_styles.get(data_key, {}).get('color', 'black')
                    linestyle = self.line_styles.get(data_key, {}).get('linestyle', 'solid')
                    pen = pg.mkPen(color, width=2, style=pg.QtCore.Qt.DashLine if linestyle == 'dashed' else pg.QtCore.Qt.SolidLine)

                    x_data = real_time_data['Time'][-400:]
                    y_data = real_time_data[data_key][-400:]

                    # ✅ Ensure x_data and y_data have the same length
                    min_length = min(len(x_data), len(y_data))
                    x_data = x_data[:min_length]
                    y_data = y_data[:min_length]
                    # ✅ 길이 체크
                    if len(x_data) < 2 or len(y_data) < 2:
                        continue

                    # Special handling for ctrltime if needed
                    if data_key == 'ctrltime':
                        pen = pg.mkPen('magenta', width=2)  # Ensure ctrltime has a distinct style

                    self.graph_widget.plot(x_data, y_data, pen=pen)
                    self.graph_widget.plot(x_data, y_data, pen=pen)