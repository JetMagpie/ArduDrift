import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QPushButton, QComboBox, QLabel, QSlider, 
                             QTextEdit, QGroupBox, QScrollArea, QGridLayout,
                             QSplitter, QFrame, QMessageBox, QDialog)
from PyQt5.QtCore import Qt, QTimer, QMutex, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QPalette, QColor
import matplotlib
# 在导入其他matplotlib模块之前设置后端
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
import re
from collections import deque

# 设置matplotlib使用支持中文的字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# 线程安全的数据处理器
class DataProcessor(QObject):
    data_ready = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.mutex = QMutex()
        self.data_queue = deque(maxlen=10)  # 小队列减少内存压力
        
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

# S型曲线显示组件
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
        x = np.linspace(-1, 1, 200)  # 增加采样点以获得更平滑的曲线
        
        if abs(exp_param) < 0.001:
            # 当exp_param接近0时，使用线性函数
            y = x
        else:
            # 使用与Arduino代码相同的S型曲线算法
            base_pow_a = 100.0 ** exp_param
            
            # 对每个x值单独计算
            y = np.zeros_like(x)
            for i, xi in enumerate(x):
                abs_x = abs(xi)
                sign_x = 1.0 if xi > 0 else -1.0
                
                if abs_x < 0.001:
                    y[i] = 0.0
                else:
                    # 计算分子部分：((100^a)^(|x|) / (100^a)) - (1/(100^a))
                    numerator = (base_pow_a ** abs_x / base_pow_a) - (1.0 / base_pow_a)
                    # 计算分母部分：1 - (1/(100^a))
                    denominator = 1.0 - (1.0 / base_pow_a)
                    # 最终结果
                    y[i] = sign_x * (numerator / denominator)
            
        self.ax.clear()
        self.setup_plot()
        self.ax.plot(x, y, 'b-', linewidth=2)
        
        # 添加参数值显示
        self.ax.text(0.05, 0.95, f'EXP = {exp_param:.3f}', 
                    transform=self.ax.transAxes, fontsize=10,
                    verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        self.draw_idle()  # 使用draw_idle而不是draw

# 实时数据绘图窗口
class RealTimePlotWindow(QDialog):
    # 添加关闭信号
    closed = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("实时传感器数据监控")
        self.setGeometry(200, 200, 1200, 800)
        
        # 数据缓冲区 - 保存最近的数据点，减少缓冲区大小
        self.buffer_size = 100  # 从200减少到100
        self.time_data = deque(maxlen=self.buffer_size)
        self.steering_data = deque(maxlen=self.buffer_size)
        self.gain_data = deque(maxlen=self.buffer_size)
        self.servo_data = deque(maxlen=self.buffer_size)
        self.accel_x_data = deque(maxlen=self.buffer_size)
        self.accel_y_data = deque(maxlen=self.buffer_size)
        self.gyro_data = deque(maxlen=self.buffer_size)
        self.correction_data = deque(maxlen=self.buffer_size)
        
        # 绘图优化
        self.last_plot_time = 0
        self.plot_interval = 200  # 增加绘图间隔到200ms
        
        self.setup_ui()
        
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # 创建matplotlib图形
        self.fig = Figure(figsize=(12, 8), dpi=80)  # 降低DPI减少内存使用
        self.canvas = FigureCanvas(self.fig)
        
        # 创建4个子图
        self.ax1 = self.fig.add_subplot(221)  # PWM数据
        self.ax2 = self.fig.add_subplot(222)  # 加速度数据
        self.ax3 = self.fig.add_subplot(223)  # 角速度数据
        self.ax4 = self.fig.add_subplot(224)  # 修正量数据
        
        layout.addWidget(self.canvas)
        
        # 控制按钮
        button_layout = QHBoxLayout()
        self.clear_btn = QPushButton("清除数据")
        self.clear_btn.clicked.connect(self.clear_data)
        self.pause_btn = QPushButton("暂停/继续")
        self.pause_btn.setCheckable(True)
        self.pause_btn.clicked.connect(self.toggle_pause)
        
        button_layout.addWidget(self.clear_btn)
        button_layout.addWidget(self.pause_btn)
        button_layout.addStretch()
        
        layout.addLayout(button_layout)
        
        # 初始化图形
        self.setup_plots()
        
        # 定时器用于更新图形 - 降低更新频率
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        
        self.paused = False
        
    def setup_plots(self):
        # 设置PWM数据图
        self.ax1.set_title('PWM输入输出')
        self.ax1.set_xlabel('时间 (样本数)')
        self.ax1.set_ylabel('PWM值')
        self.ax1.grid(True, linestyle='--', alpha=0.7)
        self.ax1.set_ylim(1000, 2000)
        
        # 设置加速度数据图
        self.ax2.set_title('加速度数据')
        self.ax2.set_xlabel('时间 (样本数)')
        self.ax2.set_ylabel('加速度 (g)')
        self.ax2.grid(True, linestyle='--', alpha=0.7)
        self.ax2.set_ylim(-2, 2)
        
        # 设置角速度数据图
        self.ax3.set_title('角速度数据')
        self.ax3.set_xlabel('时间 (样本数)')
        self.ax3.set_ylabel('角速度 (dps)')
        self.ax3.grid(True, linestyle='--', alpha=0.7)
        self.ax3.set_ylim(-200, 200)
        
        # 设置修正量数据图
        self.ax4.set_title('修正量数据')
        self.ax4.set_xlabel('时间 (样本数)')
        self.ax4.set_ylabel('修正量')
        self.ax4.grid(True, linestyle='--', alpha=0.7)
        self.ax4.set_ylim(-1, 1)
        
        # 创建初始空线条
        self.line_steering, = self.ax1.plot([], [], 'r-', label='转向输入', linewidth=1)
        self.line_gain, = self.ax1.plot([], [], 'g-', label='增益输入', linewidth=1)
        self.line_servo, = self.ax1.plot([], [], 'b-', label='舵机输出', linewidth=1)
        self.ax1.legend(loc='upper right')
        
        self.line_accel_x, = self.ax2.plot([], [], 'r-', label='加速度X', linewidth=1)
        self.line_accel_y, = self.ax2.plot([], [], 'g-', label='加速度Y', linewidth=1)
        self.ax2.legend(loc='upper right')
        
        self.line_gyro, = self.ax3.plot([], [], 'b-', label='角速度Z', linewidth=1)
        self.ax3.legend(loc='upper right')
        
        self.line_correction, = self.ax4.plot([], [], 'm-', label='修正量', linewidth=1)
        self.ax4.legend(loc='upper right')
        
        self.fig.tight_layout()
        
    def add_data(self, data_dict):
        """添加新的数据点"""
        if self.paused:
            return
            
        # 添加时间戳（使用样本计数）
        if len(self.time_data) == 0:
            self.time_data.append(0)
        else:
            self.time_data.append(self.time_data[-1] + 1)
        
        # 添加各项数据，使用默认值防止KeyError
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
        """更新所有图形"""
        if self.paused or len(self.time_data) == 0:
            return
            
        try:
            # 确保所有数据序列长度一致
            min_length = len(self.time_data)
            
            # 更新PWM数据图 - 修复切片操作
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
            
            # 更新加速度数据图 - 修复切片操作
            if len(self.accel_x_data) > 0:
                data_length = min(min_length, len(self.accel_x_data))
                self.line_accel_x.set_data(list(self.time_data)[-data_length:], list(self.accel_x_data)[-data_length:])
            if len(self.accel_y_data) > 0:
                data_length = min(min_length, len(self.accel_y_data))
                self.line_accel_y.set_data(list(self.time_data)[-data_length:], list(self.accel_y_data)[-data_length:])
            self.ax2.relim()
            self.ax2.autoscale_view()
            
            # 更新角速度数据图 - 修复切片操作
            if len(self.gyro_data) > 0:
                data_length = min(min_length, len(self.gyro_data))
                self.line_gyro.set_data(list(self.time_data)[-data_length:], list(self.gyro_data)[-data_length:])
            self.ax3.relim()
            self.ax3.autoscale_view()
            
            # 更新修正量数据图 - 修复切片操作
            if len(self.correction_data) > 0:
                data_length = min(min_length, len(self.correction_data))
                self.line_correction.set_data(list(self.time_data)[-data_length:], list(self.correction_data)[-data_length:])
            self.ax4.relim()
            self.ax4.autoscale_view()
            
            # 使用draw_idle而不是draw，减少CPU使用
            self.canvas.draw_idle()
            
        except Exception as e:
            print(f"绘图错误: {e}")
    
    def clear_data(self):
        """清除所有数据"""
        self.time_data.clear()
        self.steering_data.clear()
        self.gain_data.clear()
        self.servo_data.clear()
        self.accel_x_data.clear()
        self.accel_y_data.clear()
        self.gyro_data.clear()
        self.correction_data.clear()
        
    def toggle_pause(self):
        """切换暂停状态"""
        self.paused = not self.paused
        if self.paused:
            self.pause_btn.setText("继续")
            self.plot_timer.stop()
        else:
            self.pause_btn.setText("暂停")
            if not self.plot_timer.isActive():
                self.plot_timer.start(self.plot_interval)
    
    def showEvent(self, event):
        """窗口显示时启动定时器"""
        super().showEvent(event)
        if not self.paused and not self.plot_timer.isActive():
            self.plot_timer.start(self.plot_interval)
    
    def closeEvent(self, event):
        """窗口关闭时清理资源"""
        self.plot_timer.stop()
        plt.close(self.fig)
        # 发出关闭信号
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
        
        # 性能优化变量
        self.receive_buffer = ""  # 接收缓冲区
        self.last_display_time = 0  # 上次显示时间
        self.display_interval = 100  # 显示间隔(ms)
        self.slider_update_pending = False  # 滑块更新标志
        
        # 实时绘图窗口
        self.plot_window = None
        
        # 数据处理器
        self.data_processor = DataProcessor()
        self.data_processor.data_ready.connect(self.handle_sensor_data)
        
        self.setup_ui()
        self.setup_parameters()
        
    def setup_ui(self):
        self.setWindowTitle("Arduino Drift System Tuner")
        self.setGeometry(100, 100, 2000, 800)
        
        # 中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QHBoxLayout(central_widget)
        
        # 左侧参数面板
        left_panel = QWidget()
        left_panel.setMaximumWidth(1000)
        left_layout = QVBoxLayout(left_panel)
        
        # 串口连接部分
        serial_group = QGroupBox("Serial Connection")
        serial_layout = QGridLayout(serial_group)
        
        self.port_combo = QComboBox()
        self.refresh_ports()
        
        self.refresh_btn = QPushButton("Refresh Ports")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        
        self.connect_btn = QPushButton("Open Port")
        self.connect_btn.clicked.connect(self.toggle_connection)
        
        self.report_btn = QPushButton("Report: OFF")
        self.report_btn.setCheckable(True)
        self.report_btn.clicked.connect(self.toggle_report)
        
        self.plot_btn = QPushButton("打开实时图表")
        self.plot_btn.clicked.connect(self.toggle_plot_window)
        
        serial_layout.addWidget(QLabel("Port:"), 0, 0)
        serial_layout.addWidget(self.port_combo, 0, 1)
        serial_layout.addWidget(self.refresh_btn, 0, 2)
        serial_layout.addWidget(self.connect_btn, 0, 3)
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
        
        # 设置深色主题
        palette = self.console.palette()
        palette.setColor(QPalette.Base, QColor(30, 30, 30))
        palette.setColor(QPalette.Text, QColor(220, 220, 220))
        self.console.setPalette(palette)
        
        console_layout.addWidget(self.console)
        
        # 手动命令输入
        manual_cmd_group = QGroupBox("Manual Command")
        manual_cmd_layout = QVBoxLayout(manual_cmd_group)
        
        self.manual_cmd_input = QTextEdit()
        self.manual_cmd_input.setMaximumHeight(100)
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
        
        # 定时器用于读取串口数据 - 调整频率为50ms
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_data)
        
        # 数据处理定时器
        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self.data_processor.process_data)
        self.data_timer.start(100)  # 每100ms处理一次数据
        
        # 显示更新定时器 - 单独处理显示更新
        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self.update_display)
        self.display_timer.start(100)  # 100ms更新一次显示
        
        # 刷新端口定时器
        self.port_timer = QTimer()
        self.port_timer.timeout.connect(self.refresh_ports)
        self.port_timer.start(2000)
        
    def setup_parameters(self):
        # 参数定义 - 与Arduino代码中的参数对应
        self.parameters = [
            {"name": "BOARD_ROTATION", "min": 0, "max": 360, "default": 270, "decimals": 0},
            {"name": "K_GAIN", "min": 0.001, "max": 0.1, "default": 0.015, "decimals": 3},
            {"name": "DEFAULT_GAIN", "min": 0, "max": 500, "default": 200, "decimals": 0},
            {"name": "STEER_BY_ACC_RATE", "min": 0, "max": 20, "default": 2, "decimals": 1},
            {"name": "COUNTER_STEER_RANGE", "min": 0, "max": 1.0, "default": 0.85, "decimals": 2},
            {"name": "SERVO_LIMIT_LEFT", "min": 0, "max": 1.0, "default": 1.0, "decimals": 2},
            {"name": "SERVO_LIMIT_RIGHT", "min": 0, "max": 1.0, "default": 0.9, "decimals": 2},
            {"name": "LOOP_FREQUENCY", "min": 50, "max": 1000, "default": 500, "decimals": 0},
            {"name": "IMU_FILTER", "min": 1, "max": 500, "default": 20, "decimals": 0},
            {"name": "SERVO_FILTER", "min": 1, "max": 500, "default": 20, "decimals": 0},
            {"name": "ANGACC_FILTER", "min": 1, "max": 500, "default": 10, "decimals": 0},
            {"name": "STEER_BY_ANGACC_RATE", "min": 0, "max": 10, "default": 1, "decimals": 1},
            {"name": "GYRO_EXP", "min": -1, "max": 1, "default": 0, "decimals": 2},
            {"name": "OUTPUT_EXP", "min": -1, "max": 1, "default": 0, "decimals": 2}
        ]
        
        # 创建参数控件
        row = 0
        self.sliders = {}
        self.value_labels = {}
        
        for param in self.parameters:
            name_label = QLabel(param["name"])
            self.param_layout.addWidget(name_label, row, 0)
            
            value_label = QLabel(str(param["default"]))
            value_label.setMinimumWidth(80)  # 增加标签宽度以适应更长的数字
            self.param_layout.addWidget(value_label, row, 1)
            self.value_labels[param["name"]] = value_label
            
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(1000)
            slider.setValue(int((param["default"] - param["min"]) / (param["max"] - param["min"]) * 1000))
            
            # 增加滑块长度 - 设置为原来的三倍
            slider.setMinimumWidth(300)  # 原来是大约100，现在设为300
            slider.setMaximumWidth(400)
            
            slider.param_name = param["name"]
            slider.param_min = param["min"]
            slider.param_max = param["max"]
            slider.param_decimals = param["decimals"]
            slider.value_label = value_label
            
            slider.sliderReleased.connect(self.on_slider_released)
            slider.valueChanged.connect(lambda value, s=slider: self.on_slider_changed(s))
            
            self.param_layout.addWidget(slider, row, 2)
            self.sliders[param["name"]] = slider
            
            self.param_values[param["name"]] = param["default"]
            
            row += 1
            
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
            # 优化串口配置
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05  # 减少超时时间
            )
            
            if self.serial_port.is_open:
                self.connect_btn.setText("Close Port")
                self.timer.start(50)  # 增加读取间隔到50ms
                self.console.append(f"Connected to {port_name}")
                
                # 清空缓冲区
                self.receive_buffer = ""
                
                # 等待Arduino初始化
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
        """连接稳定后的初始化"""
        self.connection_established = True
        self.console.append("Arduino ready, starting initialization...")
        
        # 延迟一点时间后请求参数
        QTimer.singleShot(1000, self.request_params)
    
    def read_data(self):
        # 优化读取逻辑 - 只读取数据，不更新UI
        if self.serial_port and self.serial_port.is_open:
            try:
                # 读取所有可用数据
                data = self.serial_port.read(self.serial_port.in_waiting or 1)
                if data:
                    # 尝试以UTF-8解码，如果失败则显示十六进制
                    try:
                        text = data.decode('utf-8')
                        # 将数据添加到缓冲区，而不是直接更新UI
                        self.receive_buffer += text
                        
                        # 检查是否收到Arduino的启动消息
                        if ("APM2.8" in text or "Arduino" in text or "陀螺仪" in text) and not self.connection_established:
                            self.after_connection()
                        
                        # 如果正在等待参数返回，尝试解析参数行
                        if self.waiting_for_params:
                            self.parse_param_line(text)
                            
                        # 如果启用了报告功能，尝试解析传感器数据
                        if self.report_btn.isChecked():
                            self.parse_sensor_data(text)
                            
                    except UnicodeDecodeError:
                        # 显示十六进制格式
                        hex_text = ' '.join([f'{b:02X}' for b in data])
                        self.receive_buffer += f"[HEX] {hex_text} "
            except Exception as e:
                # 忽略读取错误，避免频繁的错误报告
                print(f"串口读取错误: {e}")
    
    def parse_sensor_data(self, text):
        """解析传感器数据并发送到数据处理器"""
        lines = text.split('\n')
        for line in lines:
            line = line.strip()
            if not line:
                continue
                
            try:
                # 解析传感器数据格式，例如：
                # IN_STEER:1500 IN_GAIN:1500 OUT_SERVO:1500 | Accel:(0.12,0.34)g | ω:12.3dps | Correction:0.123
                
                # 使用正则表达式匹配各个字段
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
                
                # 如果有有效数据且绘图窗口存在，则发送数据到处理器
                if data_dict and self.plot_window and self.plot_window.isVisible():
                    self.data_processor.add_data(data_dict)
                    
            except Exception as e:
                print(f"数据解析错误: {e}")
    
    def handle_sensor_data(self, data_dict):
        """处理从数据处理器接收到的传感器数据"""
        if self.plot_window and self.plot_window.isVisible():
            self.plot_window.add_data(data_dict)
    
    def update_display(self):
        """定期更新显示，减少UI更新频率"""
        if self.receive_buffer:
            # 将缓冲区内容添加到控制台
            self.console.insertPlainText(self.receive_buffer)
            
            # 自动滚动到底部
            scrollbar = self.console.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
            
            # 清空缓冲区
            self.receive_buffer = ""
            
        # 处理滑块更新
        if self.slider_update_pending:
            self.slider_update_pending = False
            # 这里可以添加滑块批量更新的逻辑
    
    def toggle_report(self):
        if self.report_btn.isChecked():
            self.send_command("report on")
            self.report_btn.setText("Report: ON")
            # 如果绘图窗口不存在，自动创建
            if not self.plot_window:
                self.toggle_plot_window()
        else:
            self.send_command("report off")
            self.report_btn.setText("Report: OFF")
            
    def toggle_plot_window(self):
        """切换实时绘图窗口的显示状态"""
        if self.plot_window is None:
            self.plot_window = RealTimePlotWindow(self)
            # 连接关闭信号
            self.plot_window.closed.connect(self.on_plot_window_closed)
            self.plot_window.show()
            self.plot_btn.setText("关闭实时图表")
        else:
            if self.plot_window.isVisible():
                self.plot_window.hide()
                self.plot_btn.setText("打开实时图表")
            else:
                self.plot_window.show()
                self.plot_btn.setText("关闭实时图表")
    
    def on_plot_window_closed(self):
        """处理绘图窗口关闭事件"""
        self.plot_btn.setText("打开实时图表")
        # 注意：我们不需要将 plot_window 设置为 None，因为对话框仍然存在，只是隐藏了
            
    def send_command(self, command):
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "Warning", "Please open serial port first!")
            return
        
        try:
            # 发送命令
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
            # 发送数据
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
        # 使用正则表达式匹配 "参数名 = 值" 格式
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
                        
                        # 检查是否已收到所有参数
                        if len(self.received_params) == len(self.parameters):
                            self.waiting_for_params = False
                            self.param_request_count = 0
                            self.console.append("\nAll parameters loaded successfully")
            except Exception as e:
                print(f"参数解析错误: {e}")
                        
    def update_slider_from_param(self, param_name, value):
        if param_name not in self.sliders:
            return
            
        slider = self.sliders[param_name]
        param_min = slider.param_min
        param_max = slider.param_max
        
        ratio = (value - param_min) / (param_max - param_min)
        slider_value = int(ratio * 1000)
        
        # 使用信号阻塞，避免触发valueChanged事件
        slider.blockSignals(True)
        slider.setValue(slider_value)
        slider.blockSignals(False)
        
        if slider.param_decimals == 0:
            display_value = str(int(value))
        else:
            display_value = f"{value:.{slider.param_decimals}f}"
        slider.value_label.setText(display_value)
        
        self.param_values[param_name] = value
        
        # 延迟更新S型曲线，避免频繁重绘
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
        
        # 延迟更新S型曲线
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
        """程序关闭时清理所有资源"""
        self.close_serial()
        if self.plot_window:
            self.plot_window.close()
        # 停止所有定时器
        self.timer.stop()
        self.data_timer.stop()
        self.display_timer.stop()
        self.port_timer.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 设置应用程序属性以减少崩溃
    app.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    window = MainWindow()
    window.show()
    
    try:
        sys.exit(app.exec_())
    except Exception as e:
        print(f"应用程序错误: {e}")