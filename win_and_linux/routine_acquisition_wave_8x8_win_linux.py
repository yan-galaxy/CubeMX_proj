import sys
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QSplitter, QDialog, 
                             QVBoxLayout, QHBoxLayout, QComboBox, QPushButton, QMessageBox, QWidget, QLabel, QCheckBox)
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
import csv

# 数据保存选项的宏定义
SAVE_DATA_DEFAULT = True  # 默认保存数据

# 不准改我的注释！！！不准删！！！
class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 用于图像更新
    waveform_ready = pyqtSignal(list)  # 用于波形更新
    error_signal = pyqtSignal(str)  # 串口错误信号（跨平台错误提示）

    def __init__(self, port, save_data=True, normalization_low=0, normalization_high=2000):
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
        self.data_queue = Queue()
        self.writer_thread = None
        self.is_saving = False
        self.data_buffer = []
        self.save_data = save_data
        if self.save_data:
            self.init_raw_data_saving()

    def init_raw_data_saving(self):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # // Deleted: self.npz_path = os.path.join(self.save_dir, f"raw_data_8x8_{timestamp}.npz")
        self.csv_path = os.path.join(self.save_dir, f"raw_data_8x8_{timestamp}.csv")
        self.is_saving = True
        self.writer_thread = threading.Thread(target=self.write_raw_data_to_file, daemon=True)
        self.writer_thread.start()

    def write_raw_data_to_file(self):
        # with open(self.csv_path, 'w', newline='') as csvfile:
        #     writer = csv.writer(csvfile)
        #     # 写入8x8矩阵的列标题
        #     headers = []
        #     for i in range(8):
        #         for j in range(8):
        #             headers.append(f"point{8*i+j}")
        #     writer.writerow(headers)
            
        #     while self.is_saving:
        #         if not self.data_queue.empty():
        #             # 现在接收的是10帧的数组，形状为(10, 8, 8)
        #             mapped_frames = self.data_queue.get()
        #             # 将10帧数据展平为(10, 64)的形状
        #             flattened_data = mapped_frames.reshape(10, 64)
        #             # 将所有帧数据写入CSV（不使用循环）
        #             writer.writerows(flattened_data.tolist())
        #         else:
        #             self.msleep(100)

        # 新的按组顺序保存数据到CSV的逻辑
        with open(self.csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # 创建按组排列的列标题
            headers = []
            # 预计算索引映射关系
            index_mapping = []
            for i in range(16):  # 16组
                row = 3 - i // 4  # 行号 (0-3)
                col = 3 - i % 4   # 列号 (0-3)
                # 每组四个点的索引
                idx1 = (row * 2 + 1) * 8 + (col + 1) * 2 - 1    # 左上
                idx2 = (row * 2 + 1) * 8 + (col + 1) * 2 - 2    # 右上
                idx3 = (row * 2 + 0) * 8 + (col + 1) * 2 - 1    # 左下
                idx4 = (row * 2 + 0) * 8 + (col + 1) * 2 - 2    # 右下
                
                headers.extend([f"group{i+1}_p1", f"group{i+1}_p2", f"group{i+1}_p3", f"group{i+1}_p4"])
                
                index_mapping.extend([idx1, idx2, idx3, idx4])

            writer.writerow(headers)
            
            while self.is_saving:
                if not self.data_queue.empty():
                    # 接收10帧的数组，形状为(10, 8, 8)
                    mapped_frames = self.data_queue.get()
                    # 将数据重塑为(10, 64)
                    flattened_data = mapped_frames.reshape(10, 64)
                    
                    # 使用向量化操作重新排列数据，避免使用循环
                    # 构建索引数组进行高级索引
                    grouped_data = flattened_data[:, index_mapping]
                    
                    # 将重新排列的数据写入CSV
                    writer.writerows(grouped_data.tolist())
                else:
                    self.msleep(100)

    def stop(self):
        self.running = False
        if self.save_data:
            self.is_saving = False
            if self.writer_thread and self.writer_thread.is_alive():
                self.writer_thread.join()
        # print('正在保存缓冲区数据到npz文件')
        # // Deleted: if len(self.data_buffer) > 0:
        # // Deleted:     metadata = {
        # // Deleted:         'description': '常规采集8x8传感器数据',
        # // Deleted:         'format_version': '1.0',
        # // Deleted:         'normalization_range': [self.normalization_low, self.normalization_high],
        # // Deleted:         'timestamp': datetime.now().isoformat()
        # // Deleted:     }
        # // Deleted:     np.savez_compressed(self.npz_path, data=np.array(self.data_buffer), **metadata)
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
                                
                                frames = parsed.reshape(10, 100)
                                
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

                                # 创建目标8x8矩阵 (使用向量化操作替代循环)
                                # 提取映射坐标
                                src_coords = [(x, y) for row in mapping for x, y in row]
                                src_x_coords = [coord[0] for coord in src_coords]
                                src_y_coords = [coord[1] for coord in src_coords]
                                
                                # 对所有10帧数据进行映射处理（不使用循环）
                                frames_data = frames.reshape(10, 10, 10)
                                frames_data = frames_data[:, 2:, 2:]  # 裁剪为8x8
                                
                                # 使用高级索引一次性处理所有帧的映射
                                mapped_frames = frames_data[:, src_x_coords, src_y_coords].reshape(10, 8, 8)
                                
                                # 将完整的10帧映射数据发送给CSV线程
                                if self.save_data:
                                    self.data_queue.put(mapped_frames.copy())
                                
                                # 计算平均值用于显示
                                target_matrix = np.mean(mapped_frames, axis=0)
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
        self.selected_group = 0  # 默认选择第1组
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
        self.plot.setTitle(f"组 {self.selected_group+1} (Row {(self.selected_group//4)+1} Col {(self.selected_group%4)+1})")
        self.plot.setLabels(left='数值', bottom='帧编号')
        self.plot.showGrid(x=True, y=True)
        self.plot.setYRange(-1.0, 4095.0)
        self.main_layout.addWidget(self.plot_widget)

        # 创建包含控件的水平布局
        self.control_layout = QHBoxLayout()
        
        # 添加组选择标签和下拉框
        self.group_selector = QComboBox()
        group_names = []
        for i in range(16):
            row = (i // 4) + 1
            col = (i % 4) + 1
            group_names.append(f"组 {i+1} (Row {row} Col {col})")
        self.group_selector.addItems(group_names)
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

        # 为4个通道创建曲线
        self.curves = []
        colors = ['y', 'c', 'm', 'r']  # 4种不同颜色
        for i in range(4):
            curve = self.plot.plot(pen=colors[i % len(colors)], name=f'通道 {i+1}')
            self.curves.append(curve)
        
        # 为每个通道创建数据缓冲区
        self.show_data = [[0] * 400 for _ in range(4)]

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
        row = (self.selected_group // 4) + 1
        col = (self.selected_group % 4) + 1
        self.plot.setTitle(f"组 {self.selected_group+1} (Row {row} Col {col})")

    def update_plot(self, new_row):
        if len(new_row) == 64:
            # 定义4x4组的映射关系，每组4个点
            # i=0时idx1为63，idx2为62，idx3为55，idx4为54，此为第一行第一列的组
            group_mapping = []
            for i in range(16):
                row = 3 - i // 4  # 行号 (0-3)
                col = 3 - i % 4   # 列号 (0-3)
                # 每组四个点的索引，按照8x8矩阵从左到右、从上到下排列
                idx1 = (row * 2 +1) * 8 + (col+1) * 2 -1    # 左上
                idx2 = (row * 2 +1) * 8 + (col+1) * 2 -2    # 右上
                idx3 = (row * 2 +0) * 8 + (col+1) * 2 -1    # 左下
                idx4 = (row * 2 +0) * 8 + (col+1) * 2 -2    # 右下
                group_mapping.append([idx1, idx2, idx3, idx4])
            
            # 获取当前组的4个通道数据
            group_indices = group_mapping[self.selected_group]
            group_data = [new_row[i] for i in group_indices]

            # group_data = [new_row[0],new_row[1],new_row[8],new_row[9]]
            # group_data = [new_row[63],new_row[62],new_row[55],new_row[54]]
            
            # 更新每个通道的数据缓冲区
            for i in range(4):
                channel_data = group_data[i]
                self.show_data[i] = self.show_data[i][1:] + [channel_data]
                
                # 更新曲线显示
                x_data = np.arange(len(self.show_data[i]))
                y_data = np.array(self.show_data[i])
                self.curves[i].setData(x_data, y_data)
            
            row = (self.selected_group // 4) + 1
            col = (self.selected_group % 4) + 1
            self.plot.setTitle(f"组 {self.selected_group+1} (Row {row} Col {col})")

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
        
        # 添加运行时间标签
        self.runtime_label = pg.LabelItem(justify='right')
        self.layout.addItem(self.runtime_label, 1, 0)

        self.frame_count = 0
        self.start_time = QtCore.QTime.currentTime()
        # 初始化运行时间
        self.runtime_start_time = None
        self.runtime_elapsed = 0

    def receive_data(self, new_row):
        if len(new_row) == 64:
            # 设置运行时间起始点（仅在第一次接收数据时）
            if self.runtime_start_time is None:
                self.runtime_start_time = QtCore.QTime.currentTime()
                self.runtime_elapsed = 0
            
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
        
        # 更新运行时间显示
        if self.runtime_start_time is not None:
            elapsed_ms = self.runtime_start_time.msecsTo(current_time)
            total_seconds = (self.runtime_elapsed + elapsed_ms) // 1000
            minutes = total_seconds // 60
            seconds = total_seconds % 60
            self.runtime_label.setText(f"运行时间: {minutes:02d}:{seconds:02d}")

class SerialSelectionDialog(QDialog):
    """跨平台串口选择对话框（Windows/Ubuntu通用）"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("选择串口")
        self.setFixedSize(300, 200)  # 调整窗口大小以容纳新控件
        self.selected_port = None  # 存储用户选择的串口
        self.save_data = SAVE_DATA_DEFAULT  # 存储用户是否选择保存数据

        # 1. 布局初始化
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        # 2. 串口选择下拉框
        self.port_combo = QComboBox()
        self.port_combo.setPlaceholderText("请选择可用串口")
        self.layout.addWidget(self.port_combo)

        # 3. 添加保存数据选项
        self.save_data_checkbox = QCheckBox("保存数据到文件")
        self.save_data_checkbox.setChecked(SAVE_DATA_DEFAULT)
        self.layout.addWidget(self.save_data_checkbox)

        # 4. 加载可用串口（跨平台）
        self.load_available_ports()

        # 5. 按钮布局（确认/取消）
        self.button_layout = QVBoxLayout()
        self.confirm_btn = QPushButton("确认选择")
        self.cancel_btn = QPushButton("取消")
        self.button_layout.addWidget(self.confirm_btn)
        self.button_layout.addWidget(self.cancel_btn)
        self.layout.addLayout(self.button_layout)

        # 6. 按钮信号连接
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
        self.save_data = self.save_data_checkbox.isChecked()  # 获取是否保存数据的选项
        self.accept()  # 关闭对话框并返回QDialog.Accepted

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.worker = None  # 串口线程（需在选择串口后初始化）
        self.save_data = SAVE_DATA_DEFAULT  # 是否保存数据

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
            self.save_data = dialog.save_data
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
        self.worker = SerialWorker(port=self.selected_port, save_data=self.save_data)
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