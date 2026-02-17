import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QPushButton, QComboBox, QLabel, QSlider,
                             QTextEdit, QGroupBox, QScrollArea, QGridLayout,
                             QSplitter, QFrame, QMessageBox, QDialog, QFileDialog,
                             QProgressDialog)
from PyQt5.QtCore import Qt, QTimer, QMutex, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QFont, QPalette, QColor
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
import re
from collections import deque

# 设置matplotlib中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# 线程安全的数据处理器
class DataProcessor(QObject):
    data_ready = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.mutex = QMutex()
        self.data_queue = deque(maxlen=10)
        
    def add_data(self, data_dict):
        self.mutex.lock()
        self.data_queue.append(data_dict)
        self.mutex.unlock()
        
    def process_data(self):
        self.mutex.lock()
        if self.data_queue:
            data = self.data_queue.popleft()
            self.mutex.unlock()
            self.data_ready.emit(data)
        else:
            self.mutex.unlock()

# S型曲线显示组件（保持不变）
class SCurveWidget(FigureCanvas):
    def __init__(self, title, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        super().__init__(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.title = title
        self.setup_plot()
        
    def setup_plot(self):
        self.ax.set_title(self.title)
        self.ax.set_xlabel('Input')
        self.ax.set_ylabel('Output')
        self.ax.grid(True, linestyle='--', alpha=0.7)
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        
    def update_curve(self, exp_param):
        x = np.linspace(-1, 1, 200)
        if abs(exp_param) < 0.001:
            y = x
        else:
            base_pow_a = 100.0 ** exp_param
            y = np.zeros_like(x)
            for i, xi in enumerate(x):
                abs_x = abs(xi)
                sign_x = 1.0 if xi > 0 else -1.0
                if abs_x < 0.001:
                    y[i] = 0.0
                else:
                    numerator = (base_pow_a ** abs_x / base_pow_a) - (1.0 / base_pow_a)
                    denominator = 1.0 - (1.0 / base_pow_a)
                    y[i] = sign_x * (numerator / denominator)
        self.ax.clear()
        self.setup_plot()
        self.ax.plot(x, y, 'b-', linewidth=2)
        self.ax.text(0.05, 0.95, f'EXP = {exp_param:.3f}', 
                    transform=self.ax.transAxes, fontsize=10,
                    verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        self.draw_idle()

# 实时数据绘图窗口
class RealTimePlotWindow(QDialog):
    closed = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Monitor")
        self.setGeometry(200, 200, 1200, 800)
        
        self.buffer_size = 100
        self.time_data = deque(maxlen=self.buffer_size)
        self.steering_data = deque(maxlen=self.buffer_size)
        self.gain_data = deque(maxlen=self.buffer_size)
        self.servo_data = deque(maxlen=self.buffer_size)
        self.accel_x_data = deque(maxlen=self.buffer_size)
        self.accel_y_data = deque(maxlen=self.buffer_size)
        self.gyro_data = deque(maxlen=self.buffer_size)
        self.correction_data = deque(maxlen=self.buffer_size)
        
        self.last_plot_time = 0
        self.plot_interval = 200
        
        self.setup_ui()
        
    def setup_ui(self):
        layout = QVBoxLayout(self)
        self.fig = Figure(figsize=(12, 8), dpi=80)
        self.canvas = FigureCanvas(self.fig)
        self.ax1 = self.fig.add_subplot(221)
        self.ax2 = self.fig.add_subplot(222)
        self.ax3 = self.fig.add_subplot(223)
        self.ax4 = self.fig.add_subplot(224)
        layout.addWidget(self.canvas)
        
        button_layout = QHBoxLayout()
        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self.clear_data)
        self.pause_btn = QPushButton("Continue/Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.clicked.connect(self.toggle_pause)
        button_layout.addWidget(self.clear_btn)
        button_layout.addWidget(self.pause_btn)
        button_layout.addStretch()
        layout.addLayout(button_layout)
        
        self.setup_plots()
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.paused = False
        
    def setup_plots(self):
        self.ax1.set_title('PWM')
        self.ax1.set_xlabel('time')
        self.ax1.set_ylabel('PWM Value')
        self.ax1.grid(True, linestyle='--', alpha=0.7)
        self.ax1.set_ylim(1000, 2000)
        
        self.ax2.set_title('Acceleration')
        self.ax2.set_xlabel('time')
        self.ax2.set_ylabel('Acceleration(g)')
        self.ax2.grid(True, linestyle='--', alpha=0.7)
        self.ax2.set_ylim(-2, 2)
        
        self.ax3.set_title('Angular Velocity')
        self.ax3.set_xlabel('time')
        self.ax3.set_ylabel('Angular Velocity (dps)')
        self.ax3.grid(True, linestyle='--', alpha=0.7)
        self.ax3.set_ylim(-200, 200)
        
        self.ax4.set_title('compensation')
        self.ax4.set_xlabel('time')
        self.ax4.set_ylabel('compensation')
        self.ax4.grid(True, linestyle='--', alpha=0.7)
        self.ax4.set_ylim(-1, 1)
        
        self.line_steering, = self.ax1.plot([], [], 'r-', label='Steering input', linewidth=1)
        self.line_gain, = self.ax1.plot([], [], 'g-', label='Gain input', linewidth=1)
        self.line_servo, = self.ax1.plot([], [], 'b-', label='Servo output', linewidth=1)
        self.ax1.legend(loc='upper right')
        
        self.line_accel_x, = self.ax2.plot([], [], 'r-', label='Acceleration X', linewidth=1)
        self.line_accel_y, = self.ax2.plot([], [], 'g-', label='Acceleration Y', linewidth=1)
        self.ax2.legend(loc='upper right')
        
        self.line_gyro, = self.ax3.plot([], [], 'b-', label='Angular Velocity Z', linewidth=1)
        self.ax3.legend(loc='upper right')
        
        self.line_correction, = self.ax4.plot([], [], 'm-', label='Compensation', linewidth=1)
        self.ax4.legend(loc='upper right')
        
        self.fig.tight_layout()
        
    def add_data(self, data_dict):
        if self.paused:
            return
        if len(self.time_data) == 0:
            self.time_data.append(0)
        else:
            self.time_data.append(self.time_data[-1] + 1)
        for key, value in data_dict.items():
            if key == 'steering':
                self.steering_data.append(value)
            elif key == 'gain':
                self.gain_data.append(value)
            elif key == 'servo':
                self.servo_data.append(value)
            elif key == 'accel_x':
                self.accel_x_data.append(value)
            elif key == 'accel_y':
                self.accel_y_data.append(value)
            elif key == 'gyro':
                self.gyro_data.append(value)
            elif key == 'correction':
                self.correction_data.append(value)
    
    def update_plots(self):
        if self.paused or len(self.time_data) == 0:
            return
        try:
            min_length = len(self.time_data)
            if len(self.steering_data) > 0:
                data_length = min(min_length, len(self.steering_data))
                self.line_steering.set_data(list(self.time_data)[-data_length:], list(self.steering_data)[-data_length:])
            if len(self.gain_data) > 0:
                data_length = min(min_length, len(self.gain_data))
                self.line_gain.set_data(list(self.time_data)[-data_length:], list(self.gain_data)[-data_length:])
            if len(self.servo_data) > 0:
                data_length = min(min_length, len(self.servo_data))
                self.line_servo.set_data(list(self.time_data)[-data_length:], list(self.servo_data)[-data_length:])
            self.ax1.relim()
            self.ax1.autoscale_view()
            
            if len(self.accel_x_data) > 0:
                data_length = min(min_length, len(self.accel_x_data))
                self.line_accel_x.set_data(list(self.time_data)[-data_length:], list(self.accel_x_data)[-data_length:])
            if len(self.accel_y_data) > 0:
                data_length = min(min_length, len(self.accel_y_data))
                self.line_accel_y.set_data(list(self.time_data)[-data_length:], list(self.accel_y_data)[-data_length:])
            self.ax2.relim()
            self.ax2.autoscale_view()
            
            if len(self.gyro_data) > 0:
                data_length = min(min_length, len(self.gyro_data))
                self.line_gyro.set_data(list(self.time_data)[-data_length:], list(self.gyro_data)[-data_length:])
            self.ax3.relim()
            self.ax3.autoscale_view()
            
            if len(self.correction_data) > 0:
                data_length = min(min_length, len(self.correction_data))
                self.line_correction.set_data(list(self.time_data)[-data_length:], list(self.correction_data)[-data_length:])
            self.ax4.relim()
            self.ax4.autoscale_view()
            
            self.canvas.draw_idle()
        except Exception as e:
            print(f"Plot error: {e}")
    
    def clear_data(self):
        self.time_data.clear()
        self.steering_data.clear()
        self.gain_data.clear()
        self.servo_data.clear()
        self.accel_x_data.clear()
        self.accel_y_data.clear()
        self.gyro_data.clear()
        self.correction_data.clear()
        
    def toggle_pause(self):
        self.paused = not self.paused
        if self.paused:
            self.pause_btn.setText("Continue")
            self.plot_timer.stop()
        else:
            self.pause_btn.setText("Pause")
            if not self.plot_timer.isActive():
                self.plot_timer.start(self.plot_interval)
    
    def showEvent(self, event):
        super().showEvent(event)
        if not self.paused and not self.plot_timer.isActive():
            self.plot_timer.start(self.plot_interval)
    
    def closeEvent(self, event):
        self.plot_timer.stop()
        plt.close(self.fig)
        self.closed.emit()
        event.accept()

# 主窗口
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.param_values = {}
        self.waiting_for_params = False
        self.received_params = {}
        self.connection_established = False
        self.param_request_count = 0
        
        self.receive_buffer = ""
        self.last_display_time = 0
        self.display_interval = 100
        self.slider_update_pending = False
        
        self.plot_window = None
        
        self.data_processor = DataProcessor()
        self.data_processor.data_ready.connect(self.handle_sensor_data)
        
        self.setup_ui()
        self.setup_parameters()
        
    def setup_ui(self):
        self.setWindowTitle("Arduino Drift System Tuner")
        self.setGeometry(100, 100, 2200, 1100)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        left_panel = QWidget()
        left_panel.setMaximumWidth(1200)
        left_layout = QVBoxLayout(left_panel)
        
        # 串口连接部分
        serial_group = QGroupBox("Serial Connection")
        serial_layout = QGridLayout(serial_group)
        
        self.port_combo = QComboBox()
        self.refresh_ports()
        
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        
        self.connect_btn = QPushButton("Open Port")
        self.connect_btn.clicked.connect(self.toggle_connection)
        
        self.report_btn = QPushButton("Report: OFF")
        self.report_btn.setCheckable(True)
        self.report_btn.clicked.connect(self.toggle_report)
        
        self.plot_btn = QPushButton("Show Monitor")
        self.plot_btn.clicked.connect(self.toggle_plot_window)
        
        # 布局：第0行：端口标签、下拉框、刷新按钮、连接按钮
        serial_layout.addWidget(QLabel("Port:"), 0, 0)
        serial_layout.addWidget(self.port_combo, 0, 1)
        serial_layout.addWidget(self.refresh_btn, 0, 2)
        serial_layout.addWidget(self.connect_btn, 0, 3)
        # 第1行：报告按钮占两列，监视器按钮占两列
        serial_layout.addWidget(self.report_btn, 1, 0, 1, 2)
        serial_layout.addWidget(self.plot_btn, 1, 2, 1, 2)
        
        left_layout.addWidget(serial_group)
        
        # 参数滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        self.param_widget = QWidget()
        self.param_layout = QGridLayout(self.param_widget)
        scroll_area.setWidget(self.param_widget)
        left_layout.addWidget(scroll_area)
        
        # 右侧显示面板
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        
        # S型曲线显示
        curves_group = QGroupBox("S-Curves")
        curves_layout = QHBoxLayout(curves_group)
        self.gyro_curve = SCurveWidget("Gyro S-Curve (GYRO_EXP)")
        self.output_curve = SCurveWidget("Output S-Curve (OUTPUT_EXP)")
        curves_layout.addWidget(self.gyro_curve)
        curves_layout.addWidget(self.output_curve)
        right_layout.addWidget(curves_group)
        
        # 串口数据显示
        console_group = QGroupBox("Serial Output")
        console_layout = QVBoxLayout(console_group)
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setFont(QFont("Courier", 9))
        palette = self.console.palette()
        palette.setColor(QPalette.Base, QColor(30, 30, 30))
        palette.setColor(QPalette.Text, QColor(220, 220, 220))
        self.console.setPalette(palette)
        console_layout.addWidget(self.console)
        
        # 手动命令输入
        manual_cmd_group = QGroupBox("Manual Command")
        manual_cmd_layout = QVBoxLayout(manual_cmd_group)
        self.manual_cmd_input = QTextEdit()
        self.manual_cmd_input.setMaximumHeight(200)
        self.manual_cmd_input.setFont(QFont("Consolas", 10))
        manual_cmd_layout.addWidget(self.manual_cmd_input)
        send_cmd_layout = QHBoxLayout()
        send_cmd_layout.addStretch()
        self.send_cmd_btn = QPushButton("Send Command")
        self.send_cmd_btn.clicked.connect(self.send_manual_command)
        send_cmd_layout.addWidget(self.send_cmd_btn)
        manual_cmd_layout.addLayout(send_cmd_layout)
        right_layout.addWidget(manual_cmd_group)
        right_layout.addWidget(console_group)
        
        # 分割左右面板
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([700, 1300])
        main_layout.addWidget(splitter)
        
        # 定时器
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_data)
        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self.data_processor.process_data)
        self.data_timer.start(100)
        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self.update_display)
        self.display_timer.start(100)
        self.port_timer = QTimer()
        self.port_timer.timeout.connect(self.refresh_ports)
        self.port_timer.start(2000)
        
    def setup_parameters(self):
        # 参数定义（保持不变）
        self.parameters = [
            {"name": "BOARD_ROTATION", "min": 0, "max": 360, "default": 270, "decimals": 0,
             "description": "Installation direction of IMU, change this value if the lateral acceleration response is wrong."},
            {"name": "K_GAIN", "min": -0.02, "max": 0.02, "default": 0.003, "decimals": 4,
            "description": "The preset gain for gyro output, the total gain is this value multiply RC gain input. Use a negative value if servo turns in wrong direction."},
            {"name": "DEFAULT_GAIN", "min": 0, "max": 500, "default": 200, "decimals": 0,
            "description": "This value will replace RC gain input if the gain channel isn't connected."},
            {"name": "STEER_BY_ACC_RATE", "min": 0, "max": 20, "default": 0.5, "decimals": 1,
            "description": "Lateral acceleration will generate a linear decay of the countersteer angle. This value changes the rate of this decay."},
            {"name": "COUNTER_STEER_RANGE", "min": 0, "max": 1.0, "default": 0.95, "decimals": 2,
            "description": "This value represents the maximum ratio of gyro output in total output."},
            {"name": "SERVO_LIMIT_LEFT", "min": 0, "max": 1.0, "default": 1.0, "decimals": 2,
            "description": "Left servo rotation limit."},
            {"name": "SERVO_LIMIT_RIGHT", "min": 0, "max": 1.0, "default": 1.0, "decimals": 2,
            "description": "Right servo rotation limit."},
            {"name": "LOOP_FREQUENCY", "min": 50, "max": 1000, "default": 100, "decimals": 0,
            "description": "The calculation frequency of the core algorithm."},
            {"name": "IMU_FILTER", "min": 1, "max": 200, "default": 30, "decimals": 0,
            "description": "Cut-off frequency of the lowpass filter of IMU readings."},
            {"name": "SERVO_FILTER", "min": 1, "max": 200, "default": 120, "decimals": 0,
            "description": "Cut-off frequency of the lowpass filter of servo output."},
            {"name": "ANGACC_FILTER", "min": 1, "max": 200, "default": 30, "decimals": 0,
            "description": "Cut-off frequency of an additional lowpass filter of angular acceleration"},
            {"name": "STEER_BY_ANGACC_RATE", "min": 0, "max": 5, "default": 1, "decimals": 2,
            "description": "This value determines the ratio of angular acceleration response in gyro output. Angular acceleration is used to make the response faster, may cause viberation."},
            {"name": "GYRO_EXP", "min": -1, "max": 1, "default": -0.18, "decimals": 2,
            "description": "Exponential curve adjustment of the gyro output."},
            {"name": "OUTPUT_EXP", "min": -1, "max": 1, "default": 0, "decimals": 2,
            "description": "Exponential curve adjustment of the total output, which is gyro output plus RC steering input."},
            {"name": "STEER_BY_ANGVEL_RATE", "min": 0, "max": 5, "default": 1.1, "decimals": 2,
            "description": "This value determines the ratio of angular velocity response in gyro output. Reduce it if vibraton presents, increase is if the car spins easily."},
            {"name": "STEER_BY_ANG_RATE", "min": 0, "max": 5, "default": 1.0, "decimals": 2,
            "description": "This value determines the ratio of direction response in gyro output. It helps keeping the wheel direction while the drifting direction is changing. Reduce it if it is hard to change derection, increase it if the wheels shake during direction change."},
            {"name": "STEER_BY_ANG_LIMIT", "min": 10, "max": 90, "default": 90.0, "decimals": 0,
            "description": "The limit of angle respons."},
            {"name": "ANGVEL_ZERO", "min": -5, "max": 5, "default": 0.0, "decimals": 2,
            "description": "Manual trim for imu's angular velocity reading."},
            {"name": "ANG_HALF_LIFE", "min": 0.001, "max": 2, "default": 0.15, "decimals": 2,
            "description": "A exponential decay is used to avoid the angle response hinders a direction change. This value determins the half life of the decay. Increase it if the angle response doesn't work, decrease it if the angle response hinders direction changes."}
        ]
        
        # 创建参数控件（保持不变）
        row = 0
        self.sliders = {}
        self.value_labels = {}
        self.description_labels = {}
        self.fold_buttons = {}
        
        for param in self.parameters:
            name_label = QLabel(param["name"])
            self.param_layout.addWidget(name_label, row, 0)
            
            value_label = QLabel(str(param["default"]))
            value_label.setMinimumWidth(80)
            self.param_layout.addWidget(value_label, row, 1)
            self.value_labels[param["name"]] = value_label
            
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(1000)
            slider.setValue(int((param["default"] - param["min"]) / (param["max"] - param["min"]) * 1000))
            slider.setMinimumWidth(300)
            slider.setMaximumWidth(400)
            slider.param_name = param["name"]
            slider.param_min = param["min"]
            slider.param_max = param["max"]
            slider.param_decimals = param["decimals"]
            slider.value_label = value_label
            slider.sliderReleased.connect(self.on_slider_released)
            slider.valueChanged.connect(lambda value, s=slider: self.on_slider_changed(s))
            self.param_layout.addWidget(slider, row, 2, 1, 3)
            self.sliders[param["name"]] = slider
            
            fold_btn = QPushButton("▼ Expand")
            fold_btn.setFixedWidth(110)
            fold_btn.setCheckable(True)
            fold_btn.setChecked(False)
            fold_btn.param_name = param["name"]
            fold_btn.clicked.connect(self.toggle_description)
            self.param_layout.addWidget(fold_btn, row, 5)
            self.fold_buttons[param["name"]] = fold_btn
            
            row += 1
            
            description_label = QLabel(param["description"])
            description_label.setWordWrap(True)
            description_label.setStyleSheet("color: #666666; font-size: 11pt; padding: 0px; border: 0px solid #ddd; background-color: #f9f9f9;")
            description_label.setMinimumWidth(400)
            description_label.setMaximumWidth(600)
            description_label.setVisible(False)
            self.param_layout.addWidget(description_label, row, 0, 1, 6)
            self.description_labels[param["name"]] = description_label
            
            self.param_values[param["name"]] = param["default"]
            row += 1
        
        self.gyro_curve.update_curve(0)
        self.output_curve.update_curve(0)
        
    def toggle_description(self):
        # 保持不变
        button = self.sender()
        param_name = button.param_name
        if param_name in self.description_labels:
            description_label = self.description_labels[param_name]
            is_visible = description_label.isVisible()
            if is_visible:
                description_label.setVisible(False)
                button.setText("▼ Expand")
                button.setChecked(False)
            else:
                description_label.setVisible(True)
                button.setText("▲ Collapse")
                button.setChecked(True)
        self.gyro_curve.update_curve(0)
        self.output_curve.update_curve(0)
        
    def refresh_ports(self):
        current_port = self.port_combo.currentText()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current_port in ports:
            self.port_combo.setCurrentText(current_port)
            
    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.close_serial()
        else:
            self.open_serial()
    
    def open_serial(self):
        if self.port_combo.count() == 0:
            QMessageBox.warning(self, "Warning", "No available serial ports!")
            return
        port_name = self.port_combo.currentText()
        try:
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05
            )
            if self.serial_port.is_open:
                self.connect_btn.setText("Close Port")
                self.timer.start(50)
                self.console.append(f"Connected to {port_name}")
                self.receive_buffer = ""
                QTimer.singleShot(2000, self.after_connection)
            else:
                QMessageBox.critical(self, "Error", f"Cannot open port {port_name}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to open port: {str(e)}")
    
    def close_serial(self):
        if self.serial_port and self.serial_port.is_open:
            self.timer.stop()
            self.data_timer.stop()
            self.serial_port.close()
            self.connect_btn.setText("Open Port")
            self.console.append("Port closed")
            self.connection_established = False
    
    def after_connection(self):
        self.connection_established = True
        self.console.append("Arduino ready, starting initialization...")
        QTimer.singleShot(1000, self.request_params)
    
    def read_data(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                data = self.serial_port.read(self.serial_port.in_waiting or 1)
                if data:
                    try:
                        text = data.decode('utf-8')
                        self.receive_buffer += text
                        if ("nRF52840" in text or "gyrp" in text) and not self.connection_established:
                            self.after_connection()
                        if self.waiting_for_params:
                            self.parse_param_line(text)
                        if self.report_btn.isChecked():
                            self.parse_sensor_data(text)
                    except UnicodeDecodeError:
                        hex_text = ' '.join([f'{b:02X}' for b in data])
                        self.receive_buffer += f"[HEX] {hex_text} "
            except Exception as e:
                print(f"Error reading serial: {e}")
    
    def parse_sensor_data(self, text):
        lines = text.split('\n')
        for line in lines:
            line = line.strip()
            if not line:
                continue
            try:
                steering_match = re.search(r'IN_STEER:(\d+)', line)
                gain_match = re.search(r'IN_GAIN:(\d+)', line)
                servo_match = re.search(r'OUT_SERVO:(\d+)', line)
                accel_match = re.search(r'Accel:\(([^,]+),([^)]+)\)g', line)
                gyro_match = re.search(r'ω:([\d.-]+)dps', line)
                correction_match = re.search(r'Correction:([\d.-]+)', line)
                data_dict = {}
                if steering_match:
                    data_dict['steering'] = int(steering_match.group(1))
                if gain_match:
                    data_dict['gain'] = int(gain_match.group(1))
                if servo_match:
                    data_dict['servo'] = int(servo_match.group(1))
                if accel_match:
                    data_dict['accel_x'] = float(accel_match.group(1))
                    data_dict['accel_y'] = float(accel_match.group(2))
                if gyro_match:
                    data_dict['gyro'] = float(gyro_match.group(1))
                if correction_match:
                    data_dict['correction'] = float(correction_match.group(1))
                if data_dict and self.plot_window and self.plot_window.isVisible():
                    self.data_processor.add_data(data_dict)
            except Exception as e:
                print(f"Error reading data: {e}")
    
    def handle_sensor_data(self, data_dict):
        if self.plot_window and self.plot_window.isVisible():
            self.plot_window.add_data(data_dict)
    
    def update_display(self):
        if self.receive_buffer:
            self.console.insertPlainText(self.receive_buffer)
            scrollbar = self.console.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
            self.receive_buffer = ""
        if self.slider_update_pending:
            self.slider_update_pending = False
    
    def toggle_report(self):
        if self.report_btn.isChecked():
            self.send_command("report on")
            self.report_btn.setText("Report: ON")
            if not self.plot_window:
                self.toggle_plot_window()
        else:
            self.send_command("report off")
            self.report_btn.setText("Report: OFF")
            
    def toggle_plot_window(self):
        if self.plot_window is None:
            self.plot_window = RealTimePlotWindow(self)
            self.plot_window.closed.connect(self.on_plot_window_closed)
            self.plot_window.show()
            self.plot_btn.setText("Close Monitor")
        else:
            if self.plot_window.isVisible():
                self.plot_window.hide()
                self.plot_btn.setText("Show Monitor")
            else:
                self.plot_window.show()
                self.plot_btn.setText("Close Monitor")
    
    def on_plot_window_closed(self):
        self.plot_btn.setText("Show Monitor")
    
    # 固件刷写相关方法已全部删除
    
    def send_command(self, command):
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "Warning", "Please open serial port first!")
            return
        try:
            self.serial_port.write((command + '\n').encode('utf-8'))
            self.console.append(f"[SENT] {command}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Send failed: {str(e)}")
    
    def send_manual_command(self):
        text = self.manual_cmd_input.toPlainText()
        if not text:
            QMessageBox.warning(self, "Warning", "Command cannot be empty!")
            return
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "Warning", "Please open serial port first!")
            return
        try:
            self.serial_port.write(text.encode('utf-8'))
            self.console.append(f"[SENT] {text}")
            self.manual_cmd_input.clear()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Send failed: {str(e)}")
            
    def request_params(self):
        if not self.serial_port or not self.serial_port.is_open:
            return
        self.waiting_for_params = True
        self.received_params = {}
        self.param_request_count += 1
        if self.param_request_count > 3:
            self.console.append("Warning: Multiple parameter requests sent without response")
            self.console.append("Try sending 'params' command manually")
            self.waiting_for_params = False
            return
        self.send_command("params")
        self.console.append("Requesting current parameters...")
        
    def parse_param_line(self, text):
        lines = text.split('\n')
        for line in lines:
            line = line.strip()
            if not line:
                continue
            try:
                match = re.match(r'^\s*(\w+)\s*=\s*([-+]?\d*\.?\d+)\s*$', line)
                if match:
                    param_name = match.group(1)
                    param_value = float(match.group(2))
                    if param_name in self.sliders:
                        self.received_params[param_name] = param_value
                        self.update_slider_from_param(param_name, param_value)
                        if len(self.received_params) == len(self.parameters):
                            self.waiting_for_params = False
                            self.param_request_count = 0
                            self.console.append("\nAll parameters loaded successfully")
            except Exception as e:
                print(f"Failed to read parameters: {e}")
                        
    def update_slider_from_param(self, param_name, value):
        if param_name not in self.sliders:
            return
        slider = self.sliders[param_name]
        param_min = slider.param_min
        param_max = slider.param_max
        ratio = (value - param_min) / (param_max - param_min)
        slider_value = int(ratio * 1000)
        slider.blockSignals(True)
        slider.setValue(slider_value)
        slider.blockSignals(False)
        if slider.param_decimals == 0:
            display_value = str(int(value))
        else:
            display_value = f"{value:.{slider.param_decimals}f}"
        slider.value_label.setText(display_value)
        self.param_values[param_name] = value
        if param_name == "GYRO_EXP":
            QTimer.singleShot(50, lambda: self.gyro_curve.update_curve(value))
        elif param_name == "OUTPUT_EXP":
            QTimer.singleShot(50, lambda: self.output_curve.update_curve(value))
            
    def on_slider_changed(self, slider):
        ratio = slider.value() / 1000.0
        value = slider.param_min + ratio * (slider.param_max - slider.param_min)
        if slider.param_decimals == 0:
            display_value = f"{int(value)}"
        else:
            display_value = f"{value:.{slider.param_decimals}f}"
        slider.value_label.setText(display_value)
        if slider.param_name == "GYRO_EXP":
            QTimer.singleShot(100, lambda: self.gyro_curve.update_curve(value))
        elif slider.param_name == "OUTPUT_EXP":
            QTimer.singleShot(100, lambda: self.output_curve.update_curve(value))
            
    def on_slider_released(self):
        slider = self.sender()
        ratio = slider.value() / 1000.0
        value = slider.param_min + ratio * (slider.param_max - slider.param_min)
        self.param_values[slider.param_name] = value
        if self.serial_port and self.serial_port.is_open:
            if slider.param_decimals == 0:
                value_str = str(int(value))
            else:
                value_str = f"{value:.{slider.param_decimals}f}"
            command = f"set {slider.param_name} {value_str}"
            self.send_command(command)
            
    def closeEvent(self, event):
        self.close_serial()
        if self.plot_window:
            self.plot_window.close()
        self.timer.stop()
        self.data_timer.stop()
        self.display_timer.stop()
        self.port_timer.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    window = MainWindow()
    window.show()
    try:
        sys.exit(app.exec_())
    except Exception as e:
        print(f"Error: {e}")