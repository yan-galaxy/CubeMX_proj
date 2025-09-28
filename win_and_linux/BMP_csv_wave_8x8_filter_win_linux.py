import sys
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QSplitter, QDialog, 
                             QVBoxLayout, QHBoxLayout, QComboBox, QPushButton, QMessageBox, QWidget, QLabel, QTabWidget, QScrollArea)
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import serial
from serial.tools import list_ports  # 跨平台串口检测
from queue import Queue
from PyQt5.QtCore import QThread, pyqtSignal
from scipy.ndimage import zoom
from scipy import signal,ndimage  # 导入信号处理模块
import os
import threading
from datetime import datetime

# 不准修改我的注释！！！不准删！！！
class FilterHandler:
    """滤波器处理类，封装高通和低通滤波的相关方法"""
    
    def __init__(self, fs, high_cutoff, low_cutoff, order=4):
        """
        初始化滤波器参数
        
        参数:
        fs: 采样频率
        high_cutoff: 高通滤波器截止频率
        low_cutoff: 低通滤波器截止频率
        order: 滤波器阶数，默认为4
        """
        self.fs = fs
        self.nyquist = 0.5 * fs
        
        # 设计高通滤波器
        normal_high_cutoff = high_cutoff / self.nyquist
        self.b_high, self.a_high = signal.butter(order, normal_high_cutoff, 
                                                btype='high', analog=False)
        
        # 设计低通滤波器
        normal_low_cutoff = low_cutoff / self.nyquist
        self.b_low, self.a_low = signal.butter(order, normal_low_cutoff, 
                                              btype='low', analog=False)
        
        # 初始化滤波器状态变量
        self.zi_high = None
        self.zi_low = None
    
    def initialize_states(self, initial_value):
        """初始化滤波器状态，使用第一个数据点作为初值"""
        self.zi_high = signal.lfilter_zi(self.b_high, self.a_high) * initial_value
        self.zi_low = signal.lfilter_zi(self.b_low, self.a_low) * 0
    
    def apply_high_pass(self, data_point):
        """对单个数据点应用高通滤波"""
        filtered_point, self.zi_high = signal.lfilter(self.b_high, self.a_high, 
                                                    [data_point], zi=self.zi_high)
        return filtered_point[0]
    
    def apply_low_pass(self, data_point):
        """对单个数据点应用低通滤波"""
        filtered_point, self.zi_low = signal.lfilter(self.b_low, self.a_low, 
                                                   [data_point], zi=self.zi_low)
        return filtered_point[0]
    

class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 用于图像更新
    waveform_ready = pyqtSignal(list)  # 用于波形更新
    error_signal = pyqtSignal(str)  # 串口错误信号（跨平台错误提示）

    def __init__(self, port, normalization_low=0.0, normalization_high=15.0):
        super().__init__()
        self.port = port
        self.baudrate = 460800
        self.ser = None
        self.running = False
        self.buffer = bytearray()
        self.FRAME_HEADER = b'\x55\xAA\xBB\xCC'
        self.FRAME_TAIL = b'\xAA\x55\x66\x77'
        
        # 添加初始化帧相关变量
        self.init_frames = []  # 用于存储初始化帧
        self.init_frame_count = 0  # 初始化帧计数器
        self.max_init_frames = 50  # 最大初始化帧数
        self.matrix_init = None  # 初始化矩阵
        self.dead_value = 0.2  # 死点值
        # 用于存储最近帧数据的缓冲区
        self.recent_frames = []  # 存储最近的帧数据用于校准
        self.max_recent_frames = 50  # 最多存储50帧

        # 初始化滤波器（64个通道）
        self.filter_handlers = {
            i: FilterHandler(fs=66.7, high_cutoff=0.2, low_cutoff=10.0, order=1)
            for i in range(64)
        }
        self.filters_initialized = False  # 滤波器初始化标志

        self.normalization_low = normalization_low
        self.normalization_high = normalization_high

        self.save_dir = "数据文件"
        self.raw_data_queue = Queue()
        self.writer_thread = None
        self.is_saving = False
        self.data_buffer = []
        # CSV 相关变量
        self.csv_file = None
        self.csv_path = None
        self.init_raw_data_saving()

    def init_raw_data_saving(self):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.save_dir, f"data_{timestamp}.csv")
        
        # 初始化 CSV 文件并写入表头
        self.csv_file = open(self.csv_path, 'w', newline='')
        header = ','.join([f'Channel_{i}' for i in range(64)]) + '\n'
        self.csv_file.write(header)
        self.csv_file.flush()

        self.is_saving = True
        self.writer_thread = threading.Thread(target=self.write_raw_data_to_file, daemon=True)
        self.writer_thread.start()

    def write_raw_data_to_file(self):
        while self.is_saving:
            if not self.raw_data_queue.empty():
                data = self.raw_data_queue.get()
                if self.csv_file:
                    line = ','.join([f'{x:.6f}' for x in data]) + '\n'
                    self.csv_file.write(line)
                    self.csv_file.flush()  # 确保数据写入磁盘
            else:
                self.msleep(100)

    def stop(self):
        self.running = False
        self.is_saving = False
        if self.writer_thread and self.writer_thread.is_alive():
            self.writer_thread.join()
        print('保存完毕')

        # 关闭 CSV 文件
        if self.csv_file:
            self.csv_file.close()

    def reset_initialization(self):
        """重置初始化相关变量，以便重新计算初始值"""
        self.matrix_init = np.mean(self.recent_frames, axis=0)
        # print("已重置初始值计算，将重新收集前%d帧用于计算新的初始值" % self.max_init_frames)

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            while self.running:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    self.buffer.extend(data)
                    start_index = self.buffer.find(self.FRAME_HEADER)
                    while start_index != -1:
                        end_index = self.buffer.find(self.FRAME_TAIL, start_index + len(self.FRAME_HEADER))
                        if end_index != -1:
                            frame = self.buffer[start_index:end_index + len(self.FRAME_TAIL)]
                            payload = frame[len(self.FRAME_HEADER):-len(self.FRAME_TAIL)]
                            parsed = np.frombuffer(payload, dtype=np.float32)

                            if len(parsed) == 64:
                                # 保存原始数据到CSV
                                self.raw_data_queue.put(parsed.copy())

                                # 将当前帧添加到recent_frames缓冲区用于校准
                                self.recent_frames.append(parsed.copy())
                                # 如果recent_frames超过最大长度，则移除最旧的帧
                                if len(self.recent_frames) > self.max_recent_frames:
                                    self.recent_frames.pop(0)

                                # 收集前max_init_frames帧用于初始化
                                if self.init_frame_count < self.max_init_frames:
                                    self.init_frames.append(parsed.copy())
                                    self.init_frame_count += 1
                                    
                                    # 当收集到足够的帧时，计算平均值作为matrix_init
                                    if self.init_frame_count == self.max_init_frames:
                                        self.matrix_init = np.mean(self.init_frames, axis=0)
                                        print(f"初始化完成，使用{self.max_init_frames}帧计算初始值")
                                        
                                # 如果已经完成初始化，进行数据处理
                                elif self.matrix_init is not None:
                                    # 直接使用原始数据，不进行滤波处理
                                    raw_results = []
                                    for i in range(64):
                                        raw_results.append(parsed[i])
                                    
                                    # 使用matrix_init进行归零处理
                                    zeroed_results = np.array(raw_results) - self.matrix_init
                                    deaded_results = zeroed_results - self.dead_value
                                    clipped_result = np.clip(deaded_results, self.normalization_low, self.normalization_high)
                                    normalized_result = (clipped_result - self.normalization_low) / (self.normalization_high - self.normalization_low)
                                    
                                    # 发送数据到界面显示
                                    # self.waveform_ready.emit(raw_results)  # 发送原始波形数据
                                    self.waveform_ready.emit(zeroed_results.tolist())  # 发送归零后的波形数据
                                    self.data_ready.emit(normalized_result.tolist())  # 发送归一化后的图像数据
                            
                            self.buffer = self.buffer[end_index + len(self.FRAME_TAIL):]
                            start_index = self.buffer.find(self.FRAME_HEADER)
                        else:
                            break
        except Exception as e:
            # 发送错误信号到主线程，弹出可视化提示
            error_msg = f"串口通信异常: {str(e)}"
            self.error_signal.emit(error_msg)
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print('串口关闭')


class SerialSelectionDialog(QDialog):
    """跨平台串口选择对话框（Windows/Ubuntu通用）"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("选择串口")
        self.setFixedSize(300, 150)  # 固定窗口大小，避免跨平台布局错乱
        self.selected_port = None  # 存储用户选择的串口

        # 1. 布局初始化
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        # 2. 串口选择下拉框
        self.port_combo = QComboBox()
        self.port_combo.setPlaceholderText("请选择可用串口")
        self.layout.addWidget(self.port_combo)

        # 3. 加载可用串口（跨平台）
        self.load_available_ports()

        # 4. 按钮布局（确认/取消）
        self.button_layout = QVBoxLayout()
        self.confirm_btn = QPushButton("确认选择")
        self.cancel_btn = QPushButton("取消")
        self.button_layout.addWidget(self.confirm_btn)
        self.button_layout.addWidget(self.cancel_btn)
        self.layout.addLayout(self.button_layout)

        # 5. 按钮信号连接
        self.confirm_btn.clicked.connect(self.on_confirm)
        self.cancel_btn.clicked.connect(self.reject)

    def load_available_ports(self):
        """加载当前系统所有可用串口（Windows: COMx / Ubuntu: /dev/ttyUSBx）"""
        self.port_combo.clear()
        available_ports = list(list_ports.comports())  # 跨平台串口检测
        
        if not available_ports:
            # 无可用串口时提示
            QMessageBox.warning(self, "警告", "未检测到可用串口！\n请检查设备连接后重试")
            self.confirm_btn.setEnabled(False)  # 禁用确认按钮
            return
        
        # 添加可用串口到下拉框（显示端口名+描述，方便用户识别）
        for port in available_ports:
            port_info = f"{port.device} - {port.description}"  # 例：COM3 - USB Serial Port
            self.port_combo.addItem(port_info, port.device)  # 存储真实端口名（如COM3）作为用户数据

        # 默认选择第一个串口
        self.port_combo.setCurrentIndex(0)

    def on_confirm(self):
        """确认选择，保存串口并关闭对话框"""
        self.selected_port = self.port_combo.currentData()  # 获取真实端口名（如COM3 / /dev/ttyUSB0）
        self.accept()  # 关闭对话框并返回QDialog.Accepted


class RoutineWaveformVisualizer:
    def __init__(self, parent_widget):
        self.parent_widget = parent_widget
        self.selected_group = 0  # 默认选择第1组(通道1-8)
        self.reset_callback = None  # 重置回调函数

        # 设置边框样式 - 黑色边框和背景
        self.parent_widget.setStyleSheet("""
            QWidget {
                background-color: black;
                border: 2px solid black;
            }
        """)
        
        # 创建一个垂直布局来包含所有控件
        self.main_layout = QVBoxLayout()
        self.parent_widget.setLayout(self.main_layout)
        
        # 创建绘图区域
        self.plot_widget = pg.PlotWidget()
        self.plot = self.plot_widget.getPlotItem()
        self.plot.setTitle(f"通道组 {self.selected_group+1} (通道 {self.selected_group*8+1}-{self.selected_group*8+8})")
        self.plot.setLabels(left='数值', bottom='帧编号')
        self.plot.showGrid(x=True, y=True)
        # self.plot.setYRange(90.5, 120.0)
        self.plot.setYRange(-1.0, 30.0)
        self.main_layout.addWidget(self.plot_widget)

        # 创建包含控件的水平布局
        self.control_layout = QHBoxLayout()
        
        # 添加组选择标签和下拉框
        self.group_selector = QComboBox()
        self.group_selector.addItems([f"组 {i+1} (通道 {i*8+1}-{i*8+8})" for i in range(8)])
        self.group_selector.setCurrentIndex(self.selected_group)
        self.group_selector.currentIndexChanged.connect(self.change_group)
        self.group_selector.setStyleSheet("""
            QComboBox {
                color: white; 
                background-color: #222222;
                border: 1px solid #555555;
                padding: 2px;
                min-width: 150px;
            }
            QComboBox QAbstractItemView {
                background-color: #222222;
                color: white;
            }
        """)
        self.control_layout.addWidget(self.group_selector)
        
        # 添加重新计算初始值按钮
        self.reset_button = QPushButton("校准")
        self.reset_button.clicked.connect(self.on_reset_clicked)
        self.reset_button.setStyleSheet("""
            QPushButton {
                color: white;
                background-color: #222222;
                border: 1px solid #555555;
                padding: 2px 10px;
                min-width: 100px;
            }
            QPushButton:hover {
                background-color: #333333;
            }
        """)
        self.control_layout.addWidget(self.reset_button)
        self.control_layout.addStretch()  # 添加弹性空间
        
        self.main_layout.addLayout(self.control_layout)

        # 为8个通道创建曲线
        self.curves = []
        colors = ['y', 'c', 'm', 'r', 'g', 'b', 'w', 'orange']  # 8种不同颜色
        for i in range(8):
            curve = self.plot.plot(pen=colors[i % len(colors)], name=f'通道 {i+1}')
            self.curves.append(curve)
        
        # 为每个通道创建数据缓冲区
        self.show_data = [[0] * 400 for _ in range(8)]

    def set_reset_callback(self, callback):
        """设置重置回调函数"""
        self.reset_callback = callback

    def on_reset_clicked(self):
        """处理重置按钮点击事件"""
        if self.reset_callback:
            self.reset_callback()

    def change_group(self, index):
        """更改显示的通道组"""
        self.selected_group = index
        self.plot.setTitle(f"通道组 {self.selected_group+1} (通道 {self.selected_group*8+1}-{self.selected_group*8+8})")

    def update_plot(self, new_row):
        if len(new_row) == 64:
            # 获取当前组的8个通道数据
            start_channel = self.selected_group * 8
            group_data = new_row[start_channel:start_channel+8]
            
            # 更新每个通道的数据缓冲区
            for i in range(8):
                channel_data = group_data[i]
                self.show_data[i] = self.show_data[i][1:] + [channel_data]
                
                # 更新曲线显示
                x_data = np.arange(len(self.show_data[i]))
                y_data = np.array(self.show_data[i])
                self.curves[i].setData(x_data, y_data)
            
            self.plot.setTitle(f"通道组 {self.selected_group+1} (通道 {self.selected_group*8+1}-{self.selected_group*8+8})")


class MatrixVisualizer:
    def __init__(self, layout, interplotation=False, rotation_angle=0, flip_horizontal=False, flip_vertical=False):
        self.layout = layout
        self.interplotation = interplotation
        self.rotation_angle = rotation_angle
        self.flip_horizontal = flip_horizontal
        self.flip_vertical = flip_vertical

        self.image_item = pg.ImageItem()
        self.plot = pg.PlotItem(title="8x8传感器数据矩阵")
        self.plot.addItem(self.image_item)
        self.plot.setLabels(left='Y轴', bottom='X轴')
        self.layout.addItem(self.plot, 0, 0)

        self.color_map = pg.colormap.get('viridis')
        self.image_item.setColorMap(self.color_map)

        self.data = np.zeros((8, 8))  # 8x8的初始数据
        self.fps_label = pg.LabelItem(justify='left')
        self.layout.addItem(self.fps_label, 1, 0)

        self.frame_count = 0
        self.start_time = QtCore.QTime.currentTime()

    def receive_data(self, new_row):
        if len(new_row) == 64:
            self.data = np.array(new_row).reshape(8, 8)

            if self.rotation_angle == 90:
                self.data = np.rot90(self.data, 1)
            elif self.rotation_angle == 180:
                self.data = np.rot90(self.data, 2)
            elif self.rotation_angle == 270:
                self.data = np.rot90(self.data, 3)

            if self.flip_horizontal:
                self.data = np.fliplr(self.data)
            if self.flip_vertical:
                self.data = np.flipud(self.data)

            if self.interplotation:
                interpolated_data = zoom(self.data, (5, 5), order=3)
                # 添加高斯模糊处理
                self.gaussian_sigma = 0.5
                interpolated_data = ndimage.gaussian_filter(interpolated_data, sigma=self.gaussian_sigma)
                self.data = interpolated_data

            self.image_item.setImage(self.data, levels=(0.0, 1.0))
            self.frame_count += 1

    def update_fps(self):
        current_time = QtCore.QTime.currentTime()
        if self.start_time.msecsTo(current_time) >= 1000:
            fps = self.frame_count
            self.fps_label.setText(f"FPS: {fps}")
            self.frame_count = 0
            self.start_time = current_time

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.worker = None  # 串口线程（需在选择串口后初始化）

        # 1. 先显示串口选择对话框（必须先选串口，再初始化主界面）
        self.select_serial_port()
        if not self.selected_port:
            # 用户取消选择或无可用串口，直接退出
            sys.exit(0)

        # 2. 初始化主界面（选择串口成功后才执行）
        self.init_main_ui()

    def select_serial_port(self):
        """显示串口选择对话框，获取用户选择的端口"""
        dialog = SerialSelectionDialog()
        if dialog.exec_() == QDialog.Accepted:
            self.selected_port = dialog.selected_port
        else:
            self.selected_port = None

    def init_main_ui(self):
        self.setWindowTitle("图像与波形双视图")
        self.resize(1500, 800)

        self.central_widget = QSplitter()
        self.setCentralWidget(self.central_widget)

        # 创建图像显示组件
        self.image_layout = pg.GraphicsLayoutWidget()
        self.image_visualizer = MatrixVisualizer(self.image_layout, interplotation=True, rotation_angle=270, flip_horizontal=False, flip_vertical=True)
        self.central_widget.addWidget(self.image_layout)

        # 创建波形显示组件
        self.waveform_layout = QWidget()
        self.waveform_visualizer = RoutineWaveformVisualizer(self.waveform_layout)
        # 设置重置回调函数
        self.waveform_visualizer.set_reset_callback(self.reset_initial_value)
        self.central_widget.addWidget(self.waveform_layout)

        # 初始化串口线程
        self.worker = SerialWorker(port=self.selected_port)
        self.worker.data_ready.connect(self.image_visualizer.receive_data)
        self.worker.waveform_ready.connect(self.waveform_visualizer.update_plot)
        self.worker.error_signal.connect(self.show_serial_error)  # 绑定错误提示
        self.worker.start()

        # 设置定时器更新FPS
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.image_visualizer.update_fps)
        self.timer.start(200)

    def reset_initial_value(self):
        """重新计算初始值"""
        if self.worker:
            self.worker.reset_initialization()
    def show_serial_error(self, error_msg):
        """显示串口错误提示（可视化弹窗）"""
        QMessageBox.critical(self, "串口错误", error_msg)
        self.close()  # 错误后关闭主窗口

    def closeEvent(self, event):
        self.hide()
        if self.worker:
            self.worker.stop()
            self.worker.wait()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec_())