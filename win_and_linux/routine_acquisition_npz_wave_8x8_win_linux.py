import sys
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QSplitter, QDialog, 
                             QVBoxLayout, QHBoxLayout, QComboBox, QPushButton, QMessageBox, QWidget)
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import serial
from serial.tools import list_ports  # 跨平台串口检测
from queue import Queue
from PyQt5.QtCore import QThread, pyqtSignal
from scipy.ndimage import zoom
import os
import threading
from datetime import datetime

# 不准改我的注释！！！不准删！！！
class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 用于图像更新
    waveform_ready = pyqtSignal(list)  # 用于波形更新
    error_signal = pyqtSignal(str)  # 串口错误信号（跨平台错误提示）

    def __init__(self, port, normalization_low=0, normalization_high=2000):
        super().__init__()
        self.port = port  # 外部传入选择的串口（不再硬编码）
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
        self.dead_value = 20.0  # 死点值
        # 用于存储最近帧数据的缓冲区
        self.recent_frames = []  # 存储最近的帧数据用于校准
        self.max_recent_frames = 50  # 最多存储50帧

        self.normalization_low = normalization_low
        self.normalization_high = normalization_high

        self.save_dir = "Routine_acq_raw_data_8x8"
        self.raw_data_queue = Queue()
        self.writer_thread = None
        self.is_saving = False
        self.data_buffer = []
        self.init_raw_data_saving()

    def init_raw_data_saving(self):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.npz_path = os.path.join(self.save_dir, f"raw_data_8x8_{timestamp}.npz")
        self.is_saving = True
        self.writer_thread = threading.Thread(target=self.write_raw_data_to_file, daemon=True)
        self.writer_thread.start()

    def write_raw_data_to_file(self):
        while self.is_saving:
            if not self.raw_data_queue.empty():
                data = self.raw_data_queue.get()
                self.data_buffer.append(data.tolist())
            else:
                self.msleep(100)

    def stop(self):
        self.running = False
        self.is_saving = False
        if self.writer_thread and self.writer_thread.is_alive():
            self.writer_thread.join()
        print('正在保存缓冲区数据到npz文件')
        if len(self.data_buffer) > 0:
            metadata = {
                'description': '常规采集8x8传感器数据',
                'format_version': '1.0',
                'normalization_range': [self.normalization_low, self.normalization_high],
                'timestamp': datetime.now().isoformat()
            }
            np.savez_compressed(self.npz_path, data=np.array(self.data_buffer), **metadata)
        print('保存完毕')

    def reset_initialization(self):
        """重置初始化相关变量，以便重新计算初始值"""
        if len(self.recent_frames) > 0:
            self.matrix_init = np.mean(self.recent_frames, axis=0)
            print("已校准零位")

    def run(self):
        try:
            # 跨平台串口连接（自动适配Windows COMx / Ubuntu /dev/ttyUSBx）
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
                            parsed = np.frombuffer(payload, dtype=np.uint16)

                            if len(parsed) == 1000:
                                self.raw_data_queue.put(parsed.copy())
                                frames = parsed.reshape(10, 100)
                                average = np.mean(frames, axis=0).reshape(10, 10)
                                average = average[2:, 2:]
                                matrix_8x8 = average

                                # 更新的映射表，之前的每个小圆的左右反了
                                mapping = [
                                    [(0,1), (0,0), (0,5), (0,4), (1,1), (1,0), (1,5), (1,4)],
                                    [(0,2), (0,3), (0,6), (0,7), (1,2), (1,3), (1,6), (1,7)],
                                    [(2,1), (2,0), (2,5), (2,4), (3,1), (3,0), (3,5), (3,4)],
                                    [(2,2), (2,3), (2,6), (2,7), (3,2), (3,3), (3,6), (3,7)],
                                    [(4,1), (4,0), (4,5), (4,4), (5,1), (5,0), (5,5), (5,4)],
                                    [(4,2), (4,3), (4,6), (4,7), (5,2), (5,3), (5,6), (5,7)],
                                    [(6,1), (6,0), (6,5), (6,4), (7,1), (7,0), (7,5), (7,4)],
                                    [(6,2), (6,3), (6,6), (6,7), (7,2), (7,3), (7,6), (7,7)],
                                ]

                                # 创建目标8x8矩阵
                                target_matrix = np.zeros((8, 8))

                                for i in range(8):
                                    for j in range(8):
                                        x, y = mapping[i][j]
                                        target_matrix[i][j] = matrix_8x8[x][y]

                                average = target_matrix.flatten()

                                # 将当前帧添加到recent_frames缓冲区用于校准
                                self.recent_frames.append(average.copy())
                                # 如果recent_frames超过最大长度，则移除最旧的帧
                                if len(self.recent_frames) > self.max_recent_frames:
                                    self.recent_frames.pop(0)

                                # 收集前max_init_frames帧用于初始化
                                if self.init_frame_count < self.max_init_frames:
                                    self.init_frames.append(average.copy())
                                    self.init_frame_count += 1
                                    
                                    # 当收集到足够的帧时，计算平均值作为matrix_init
                                    if self.init_frame_count == self.max_init_frames:
                                        self.matrix_init = np.mean(self.init_frames, axis=0)
                                        print(f"初始化完成，使用{self.max_init_frames}帧计算初始值")
                                        
                                # 如果已经完成初始化，进行数据处理
                                elif self.matrix_init is not None:
                                    # 使用matrix_init进行归零处理
                                    zeroed_results = np.array(average) - self.matrix_init
                                    deaded_results = zeroed_results - self.dead_value
                                    clipped_result = np.clip(deaded_results, self.normalization_low, self.normalization_high)
                                    normalized_result = (clipped_result - self.normalization_low) / (self.normalization_high - self.normalization_low)
                                    
                                    self.waveform_ready.emit(average.tolist())  # 发送原始波形数据
                                    # self.waveform_ready.emit(zeroed_results.tolist())  # 发送归零后的波形数据
                                    self.data_ready.emit(normalized_result.tolist())  # 发送归一化后的图像数据

                            self.buffer = self.buffer[end_index + len(self.FRAME_TAIL):]
                            start_index = self.buffer.find(self.FRAME_HEADER)
                        else:
                            break
        except Exception as e:
            # 发送错误信号到主线程，弹出可视化提示
            error_msg = f"串口通信异常: {str(e)}\nUbuntu用户请检查是否加入dialout组"
            self.error_signal.emit(error_msg)
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print('串口关闭')

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
        self.plot.setYRange(-1.0, 4095.0)
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

        # self.data = np.zeros((10, 10))
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
        """初始化主界面（图像+波形显示）"""
        self.setWindowTitle("图像与波形双视图")
        self.resize(1500, 800)

        self.central_widget = QSplitter()
        self.setCentralWidget(self.central_widget)

        # 创建图像显示组件
        self.image_layout = pg.GraphicsLayoutWidget()
        self.image_visualizer = MatrixVisualizer(self.image_layout, interplotation=False, rotation_angle=90, flip_horizontal=False, flip_vertical=False)
        self.central_widget.addWidget(self.image_layout)

        # 创建波形显示组件
        self.waveform_layout = QWidget()
        self.waveform_visualizer = RoutineWaveformVisualizer(self.waveform_layout)
        # 设置重置回调函数
        self.waveform_visualizer.set_reset_callback(self.reset_initial_value)
        self.central_widget.addWidget(self.waveform_layout)

        # 初始化串口线程（传入用户选择的端口）
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
        """窗口关闭时释放资源"""
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