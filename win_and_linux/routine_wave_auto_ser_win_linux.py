import sys
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)  # 忽略弃用警告
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QSplitter, QDialog, 
                             QVBoxLayout, QHBoxLayout, QComboBox, QPushButton, QMessageBox, QWidget, QLabel, QCheckBox, QSpacerItem, QSizePolicy, QDoubleSpinBox, QSpinBox)
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import serial
from serial.tools import list_ports
from queue import Queue
from PyQt5.QtCore import QThread, pyqtSignal
from scipy.ndimage import zoom
from scipy import signal, ndimage
import os
import threading
from datetime import datetime
import csv


# 数据保存选项的宏定义（同步）
SAVE_DATA_DEFAULT = False  # 默认保存数据 True False
# 不准改我的注释！！！不准删！！！


class FilterHandler:
    """滤波器处理类，封装低通滤波的相关方法"""
    
    def __init__(self, fs, low_cutoff, order=4):
        self.fs = fs
        self.nyquist = 0.5 * fs
        normal_low_cutoff = low_cutoff / self.nyquist
        self.b_low, self.a_low = signal.bessel(order, normal_low_cutoff, 
                                              btype='low', analog=False)
        self.zi_low = None
    
    def initialize_states(self, initial_value):
        self.zi_low = signal.lfilter_zi(self.b_low, self.a_low) * initial_value
    
    def apply_low_pass(self, data_point):
        filtered_point, self.zi_low = signal.lfilter(self.b_low, self.a_low, 
                                                   [data_point], zi=self.zi_low)
        return filtered_point[0]


class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 用于图像更新
    waveform_ready = pyqtSignal(list)  # 用于波形更新
    error_signal = pyqtSignal(str)  # 串口错误信号（跨平台错误提示）

    def __init__(self, port, save_data=True, normalization_low=0, normalization_high=700):
        super().__init__()
        self.port = port
        self.baudrate = 460800
        self.ser = None
        self.running = False
        self.buffer = bytearray()
        self.FRAME_HEADER = b'\x55\xAA\xBB\xCC'
        self.FRAME_TAIL = b'\xAA\x55\x66\x77'
        self.matrix_init = None  # 初始化矩阵（用于归零）
        self.dead_value = 0  # 死点值
        self.normalization_low = normalization_low
        self.normalization_high = normalization_high

        # 1. 校准用最近帧缓冲区（排除分组，保留校准核心）
        self.recent_frames = []  # 存储最近帧用于校准
        self.max_recent_frames = 50  # 最多存储50帧
        self.init_frames = []  # 初始帧收集（用于首次初始化）
        self.init_frame_count = 0
        self.max_init_frames = 30  # 收集x帧用于初始化

        # 2. 数据保存配置（支持选择是否保存、CSV存储）
        self.save_data = save_data
        self.save_dir = "Routine_acq_raw_data"  # 10x10专属保存目录
        self.data_queue = Queue()
        self.writer_thread = None
        self.is_saving = False

        # 滤波器功能
        self.filter_handlers = {i: FilterHandler(fs=100.0, low_cutoff=10.0, order=8) for i in range(100)}
        self.filters_initialized = False

        # 初始化保存线程（仅当需要保存时）
        if self.save_data:
            self.init_raw_data_saving()

    # 3. 数据保存初始化（CSV存储）
    def init_raw_data_saving(self):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.save_dir, f"raw_data_10x10_{timestamp}.csv")
        self.is_saving = True
        self.writer_thread = threading.Thread(target=self.write_raw_data_to_file, daemon=True)
        self.writer_thread.start()

    # 4. CSV数据写入（适配10x10数据，无分组）
    def write_raw_data_to_file(self):
        with open(self.csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # 10x10数据表头：point0 ~ point99（按行优先排序）
            headers = [f"point{i}" for i in range(100)]
            writer.writerow(headers)
            
            while self.is_saving:
                if not self.data_queue.empty():
                    # 接收10帧数据（10,100），直接展平写入
                    frames = self.data_queue.get()
                    flattened_data = frames.reshape(10, 100)
                    writer.writerows(flattened_data.tolist())
                else:
                    self.msleep(100)

    # 5. 校准功能（重置初始值）
    def reset_initialization(self):
        """使用最近帧重新计算初始值，实现校准"""
        if len(self.recent_frames) > 0:
            self.matrix_init = np.mean(self.recent_frames, axis=0) + self.dead_value
            print("已校准零位")

    # 6. 停止逻辑（适配CSV保存）
    def stop(self):
        self.running = False
        # 停止保存线程
        if self.save_data:
            self.is_saving = False
            if self.writer_thread and self.writer_thread.is_alive():
                self.writer_thread.join()
        print('保存完毕')

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
                            parsed = np.frombuffer(payload, dtype=np.uint16)

                            if len(parsed) == 1000:
                                # 处理10帧数据（10,100）
                                frames = parsed.reshape(10, 100)
                                average = np.mean(frames, axis=0)  # 单帧平均值（100）

                                # 7. 最近帧缓冲区更新（用于校准）
                                self.recent_frames.append(average.copy())
                                if len(self.recent_frames) > self.max_recent_frames:
                                    self.recent_frames.pop(0)

                                # 首次初始化：收集max_init_frames帧计算初始值
                                if self.matrix_init is None and self.init_frame_count < self.max_init_frames:
                                    self.init_frames.append(average.copy())
                                    self.init_frame_count += 1
                                    if self.init_frame_count == self.max_init_frames:
                                        self.matrix_init = np.mean(self.init_frames, axis=0) + self.dead_value
                                        print(f"初始化完成，使用{self.max_init_frames}帧计算初始值")

                                # 8. 数据保存（仅当开启时）
                                if self.save_data:
                                    self.data_queue.put(frames.copy())

                                # 保留原有滤波功能
                                if not self.filters_initialized and self.matrix_init is not None:
                                    for i in range(100):
                                        self.filter_handlers[i].initialize_states(average[i])
                                    self.filters_initialized = True

                                # 数据处理（归零、裁剪、归一化）
                                if self.matrix_init is not None and self.filters_initialized:
                                    filtered_results = [self.filter_handlers[i].apply_low_pass(average[i]) for i in range(100)]
                                    zeroed_results = np.array(filtered_results) - self.matrix_init
                                    clipped_result = np.clip(zeroed_results, self.normalization_low, self.normalization_high)
                                    normalized_result = (clipped_result - self.normalization_low) / (self.normalization_high - self.normalization_low)

                                    # print(f"归一化数据范围: 最小值={normalized_result.min():.4f}, 最大值={normalized_result.max():.4f}")
                                    # 发送信号更新界面
                                    self.waveform_ready.emit(filtered_results)  # 滤波后原始数据（用于波形）
                                    self.data_ready.emit(normalized_result.tolist())  # 归一化数据（用于图像）

                            # 缓冲区裁剪
                            self.buffer = self.buffer[end_index + len(self.FRAME_TAIL):]
                            start_index = self.buffer.find(self.FRAME_HEADER)
                        else:
                            break
        except Exception as e:
            error_msg = f"串口通信异常: {str(e)}\nUbuntu用户请检查是否加入dialout组"
            self.error_signal.emit(error_msg)
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print('串口关闭')


# 9. 波形可视化（支持10行选择+校准按钮+界面样式，排除分组）
class RoutineWaveformVisualizer:
    def __init__(self, parent_widget):
        self.parent_widget = parent_widget
        self.selected_row = 0  # 默认选择第1行（10x10的行索引0-9）
        self.reset_callback = None

        # 界面样式（黑色背景+控件样式）
        self.parent_widget.setStyleSheet("""
            QWidget {
                background-color: black;
                border: 2px solid black;
            }
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

        # 主布局（垂直）
        self.main_layout = QVBoxLayout()
        self.parent_widget.setLayout(self.main_layout)

        # 绘图区域
        self.plot_widget = pg.PlotWidget()
        self.plot = self.plot_widget.getPlotItem()
        self.plot.setTitle(f"第{self.selected_row+1}行波形（10个点）")
        self.plot.setLabels(left='数值', bottom='帧编号')
        self.plot.showGrid(x=True, y=True)
        self.plot.setYRange(0, 4096)  # 适配16位数据
        self.main_layout.addWidget(self.plot_widget)

        # 控件布局（水平：行选择+校准按钮）
        self.control_layout = QHBoxLayout()
        # 行选择下拉框（10行：第1行~第10行）
        self.row_selector = QComboBox()
        row_names = [f"第{i+1}列" for i in range(10)]
        self.row_selector.addItems(row_names)
        self.row_selector.setCurrentIndex(self.selected_row)
        self.row_selector.currentIndexChanged.connect(self.change_row)
        self.control_layout.addWidget(self.row_selector)

        # 校准按钮
        self.reset_button = QPushButton("校准零位")
        self.reset_button.clicked.connect(self.on_reset_clicked)
        self.control_layout.addWidget(self.reset_button)
        self.control_layout.addStretch()  # 右对齐留白
        self.main_layout.addLayout(self.control_layout)

        # 10行波形曲线（每行10个点，10种颜色）
        self.curves = []
        colors = [
            'y',        # 黄色（明亮）
            'c',        # 青色（鲜明）
            'm',        # 品红（艳丽）
            'r',        # 红色（醒目）
            'lime',     # 亮绿（比普通绿色更鲜艳）
            'royalblue',# 宝蓝（比普通蓝色更亮）
            'orange',   # 橙色（明快）
            'purple',   # 紫色（浓郁）
            'crimson',  # 绯红（比红色更深艳）
            'aqua'      # 水蓝（比青色更透亮）
        ]
        for i in range(10):
            curve = self.plot.plot(pen=colors[i], name=f'点{i+1}')
            self.curves.append(curve)

        # 波形数据缓冲区（每个点300帧历史）
        self.show_data = [[0] * 300 for _ in range(10)]

    # 校准按钮回调（绑定到SerialWorker的reset_initialization）
    def set_reset_callback(self, callback):
        self.reset_callback = callback

    def on_reset_clicked(self):
        if self.reset_callback:
            self.reset_callback()

    # 切换显示的行（10x10的行，无分组）
    def change_row(self, index):
        self.selected_row = index
        self.plot.setTitle(f"第{self.selected_row+1}行波形（10个点）")
        # 重置当前行的波形数据显示
        self.show_data = [[0] * 300 for _ in range(10)]

    # 更新波形（显示选中行的10个点）
    def update_plot(self, new_row):
        if len(new_row) == 100:
            # 提取选中行的10个点数据（行优先：第selected_row行的索引为selected_row*10 ~ (selected_row+1)*10 -1）
            row_start_idx = self.selected_row * 10
            row_data = new_row[row_start_idx : row_start_idx + 10]

            # 更新每个点的历史数据并刷新曲线
            for i in range(10):
                self.show_data[i] = self.show_data[i][1:] + [row_data[i]]
                x_data = np.arange(len(self.show_data[i]))
                y_data = np.array(self.show_data[i])
                self.curves[i].setData(x_data, y_data)


# 10. 图像可视化（添加运行时间标签 + 参数调节功能）
class MatrixVisualizer:
    def __init__(self, layout, interplotation=False, rotation_angle=0, flip_horizontal=False, flip_vertical=False,
                 zoom_factor=7, gaussian_sigma=0.5):
        self.layout = layout
        # 初始化参数
        self.interplotation = interplotation
        self.rotation_angle = rotation_angle
        self.flip_horizontal = flip_horizontal
        self.flip_vertical = flip_vertical
        # 插值密度和高斯核参数
        self.zoom_factor = zoom_factor  # 插值缩放因子（长宽同步）
        self.gaussian_sigma = gaussian_sigma  # 高斯滤波核参数

        self.current_data = np.zeros((10, 10))

        self.image_item = pg.ImageItem()
        self.plot = pg.PlotItem(title="10x10传感器数据矩阵")
        self.plot.addItem(self.image_item)
        self.plot.setLabels(left='Y轴', bottom='X轴')
        self.layout.addItem(self.plot, 0, 0)

        self.color_map = pg.colormap.get('viridis')
        self.image_item.setColorMap(self.color_map)

        # FPS标签 + 运行时间标签
        self.fps_label = pg.LabelItem(justify='left')
        self.runtime_label = pg.LabelItem(justify='right')
        self.layout.addItem(self.fps_label, 1, 0)
        self.layout.addItem(self.runtime_label, 1, 1)

        self.frame_count = 0
        self.start_time = QtCore.QTime.currentTime()
        self.runtime_start_time = None
        self.runtime_elapsed = 0

    # 插值密度设置
    def set_zoom_factor(self, factor):
        self.zoom_factor = factor
        if self.interplotation:  # 仅当启用插值时刷新
            self.refresh_image()

    # 高斯核参数设置
    def set_gaussian_sigma(self, sigma):
        self.gaussian_sigma = sigma
        if self.interplotation:  # 仅当启用插值时刷新
            self.refresh_image()

    # 原有参数setter保持不变
    def set_interpolation(self, enable):
        self.interplotation = enable
        self.refresh_image()

    def set_rotation_angle(self, angle):
        self.rotation_angle = angle
        self.refresh_image()

    def set_flip_horizontal(self, enable):
        self.flip_horizontal = enable
        self.refresh_image()

    def set_flip_vertical(self, enable):
        self.flip_vertical = enable
        self.refresh_image()

    # 刷新图像逻辑（修改插值和高斯滤波参数引用）
    def refresh_image(self):
        data = self.current_data.copy()

        # 应用旋转
        if self.rotation_angle == 90:
            data = np.rot90(data, 1)
        elif self.rotation_angle == 180:
            data = np.rot90(data, 2)
        elif self.rotation_angle == 270:
            data = np.rot90(data, 3)

        # 应用翻转
        if self.flip_horizontal:
            data = np.fliplr(data)
        if self.flip_vertical:
            data = np.flipud(data)

        # 应用插值（使用动态参数）
        if self.interplotation:
            # 长宽同步使用zoom_factor
            data = zoom(data, (self.zoom_factor, self.zoom_factor), order=3)
            # 使用动态高斯核参数
            data = ndimage.gaussian_filter(data, sigma=self.gaussian_sigma)

        self.image_item.setImage(data, levels=(0.0, 1.0))

    def receive_data(self, new_row):
        if len(new_row) == 100:
            if self.runtime_start_time is None:
                self.runtime_start_time = QtCore.QTime.currentTime()
                self.runtime_elapsed = 0

            self.current_data = np.array(new_row).reshape(10, 10)
            self.refresh_image()
            self.frame_count += 1

    def update_fps(self):
        current_time = QtCore.QTime.currentTime()
        if self.start_time.msecsTo(current_time) >= 1000:
            fps = self.frame_count
            self.fps_label.setText(f"FPS: {fps}")
            self.frame_count = 0
            self.start_time = current_time

        if self.runtime_start_time is not None:
            elapsed_ms = self.runtime_start_time.msecsTo(current_time)
            total_seconds = (self.runtime_elapsed + elapsed_ms) // 1000
            minutes = total_seconds // 60
            seconds = total_seconds % 60
            self.runtime_label.setText(f"运行时间: {minutes:02d}:{seconds:02d}")


# 11. 串口选择对话框
class SerialSelectionDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("选择串口")
        self.setFixedSize(500, 300)
        self.selected_port = None
        self.save_data = SAVE_DATA_DEFAULT

        self.setStyleSheet("""
            /* 窗口：圆角+轻微阴影，背景用柔和浅灰 */
            QDialog {
                background-color: #f5f5f5;
                border-radius: 12px; /* 大圆角 */
                /*box-shadow: 0 4px 8px rgba(0,0,0,0.1);*/  /* 柔和阴影增层次感  报警告 */
            }

            /* 下拉框：圆角+细边框，hover时边框变色 */
            QComboBox {
                background-color: white;
                border: 2px solid #dddddd;
                border-radius: 8px; /* 圆角 */
                padding: 6px 10px;
                margin: 8px 0;
                min-width: 250px;
            }
            QComboBox:hover {
                border-color: #88c9ff; /* 浅蓝 hover色 */
            }
            /* 下拉列表：同样圆角+柔和背景 */
            QComboBox QAbstractItemView {
                background-color: white;
                border-radius: 8px;
                border: 2px solid #dddddd;
                padding: 4px;
            }

            /* 复选框：圆角勾选框，颜色柔和 */
            QCheckBox {
                margin: 8px 0;
                spacing: 8px; /* 文字与框间距 */
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
                border-radius: 6px; /* 圆角勾选框 */
                border: 2px solid #dddddd;
                background-color: white;
            }
            QCheckBox::indicator:checked {
                background-color: #88c9ff; /* 浅蓝勾选色 */
                border-color: #88c9ff;
                image: url(:/qt-project.org/styles/commonstyle/images/check.png); /* 默认勾选图标 */
            }

            /* 按钮：圆角+渐变，hover时加深 */
            QPushButton {
                background-color: #88c9ff;
                border: none;
                border-radius: 8px; /* 大圆角 */
                padding: 8px 0;
                margin: 4px 0;
                color: white;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #66b3ff; /* hover加深色 */
            }
            QPushButton:disabled {
                background-color: #cccccc;
            }
        """)
        # --------------------------------------------------------------------------

        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(25, 20, 25, 20)  # 内边距，让控件不贴边
        self.setLayout(self.layout)

        # 串口选择下拉框
        self.port_combo = QComboBox()
        self.port_combo.setPlaceholderText("请选择可用串口")
        self.layout.addWidget(self.port_combo)

        # 保存数据复选框
        self.save_data_checkbox = QCheckBox("保存数据到CSV")
        self.save_data_checkbox.setChecked(SAVE_DATA_DEFAULT)
        self.layout.addWidget(self.save_data_checkbox)

        # 按钮布局（垂直，按钮占满宽度）
        self.button_layout = QVBoxLayout()
        self.confirm_btn = QPushButton("确认选择")
        self.cancel_btn = QPushButton("取消")
        self.button_layout.addWidget(self.confirm_btn)
        self.button_layout.addWidget(self.cancel_btn)
        self.layout.addLayout(self.button_layout)

        # 信号绑定
        self.confirm_btn.clicked.connect(self.on_confirm)
        self.cancel_btn.clicked.connect(self.reject)

        # 加载可用串口（按COM号降序）
        self.load_available_ports()

    def load_available_ports(self):
        self.port_combo.clear()
        available_ports = list(list_ports.comports())
        if not available_ports:
            QMessageBox.warning(self, "警告", "未检测到可用串口！\n请检查设备连接后重试")
            self.confirm_btn.setEnabled(False)
            return

        # 按COM号降序排序（原有逻辑不变）
        com_ports = []
        other_ports = []
        for port in available_ports:
            if "COM" in port.device:
                com_ports.append(port)
            else:
                other_ports.append(port)

        def extract_com_number(port):
            return int(port.device.replace("COM", ""))
        com_ports_sorted = sorted(com_ports, key=extract_com_number, reverse=True)
        sorted_ports = com_ports_sorted + other_ports

        # 添加到下拉框
        for port in sorted_ports:
            port_info = f"{port.device} - {port.description}"
            self.port_combo.addItem(port_info, port.device)
        self.port_combo.setCurrentIndex(0)

    def on_confirm(self):
        self.selected_port = self.port_combo.currentData()
        self.save_data = self.save_data_checkbox.isChecked()
        self.accept()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.worker = None
        self.selected_port = None
        self.save_data = SAVE_DATA_DEFAULT

        self.select_serial_port()
        if not self.selected_port:
            sys.exit(0)

        self.init_main_ui()

    def select_serial_port(self):
        dialog = SerialSelectionDialog()
        if dialog.exec_() == QDialog.Accepted:
            self.selected_port = dialog.selected_port
            self.save_data = dialog.save_data
        else:
            self.selected_port = None

    def init_main_ui(self):
        self.setWindowTitle("10x10传感器 - 图像与波形双视图")
        self.resize(1500, 800)
        # 添加以下行来使窗口最大化显示
        self.showMaximized()

        self.central_widget = QSplitter()
        self.setCentralWidget(self.central_widget)

        # 左侧：图像显示 + 参数控制面板
        self.image_container = QWidget()
        self.image_container.setStyleSheet("background-color: black; border: none;")
        self.image_container_layout = QVBoxLayout(self.image_container)
        self.image_container_layout.setContentsMargins(0, 0, 0, 0)
        
        # 1. 图像显示布局
        self.image_layout = pg.GraphicsLayoutWidget()
        # 初始化时传入插值密度和高斯核默认参数
        self.image_visualizer = MatrixVisualizer(
            self.image_layout, 
            interplotation=False, 
            rotation_angle=0, 
            flip_horizontal=True, 
            flip_vertical=False,
            zoom_factor=7,  # 默认插值密度
            gaussian_sigma=0.5  # 默认高斯核
        )
        self.image_container_layout.addWidget(self.image_layout)

        # 2. 图像参数控制面板（插值和高斯参数控件）
        self.image_control_panel = QWidget()
        self.image_control_panel.setStyleSheet("background-color: black; border: none;")
        self.image_control_layout = QHBoxLayout(self.image_control_panel)
        self.image_control_layout.setContentsMargins(10, 5, 10, 5)
        self.image_control_layout.setSpacing(15)

        # （1）插值显示复选框
        self.interp_checkbox = QCheckBox("插值显示")
        self.interp_checkbox.setStyleSheet("color: white;")
        self.interp_checkbox.setChecked(self.image_visualizer.interplotation)
        self.interp_checkbox.stateChanged.connect(
            lambda state: self.image_visualizer.set_interpolation(state == QtCore.Qt.Checked)
        )
        self.image_control_layout.addWidget(self.interp_checkbox)

        # （2）插值密度调节
        self.zoom_label = QLabel("插值密度:")
        self.zoom_label.setStyleSheet("color: white;")
        self.zoom_spin = QSpinBox()
        self.zoom_spin.setRange(1, 15)  # 范围1-15（原默认7）
        self.zoom_spin.setValue(self.image_visualizer.zoom_factor)
        self.zoom_spin.setStyleSheet("""
            QSpinBox { color: white; background-color: #222222; border: 1px solid #555555; padding: 2px; }
        """)
        self.zoom_spin.valueChanged.connect(self.image_visualizer.set_zoom_factor)
        self.image_control_layout.addWidget(self.zoom_label)
        self.image_control_layout.addWidget(self.zoom_spin)

        # （3）高斯核参数调节
        self.sigma_label = QLabel("高斯核:")
        self.sigma_label.setStyleSheet("color: white;")
        self.sigma_spin = QDoubleSpinBox()
        self.sigma_spin.setRange(0.0, 10.0)  # 范围0.0-10.0
        self.sigma_spin.setSingleStep(0.5)
        self.sigma_spin.setValue(self.image_visualizer.gaussian_sigma)
        self.sigma_spin.setStyleSheet("""
            QDoubleSpinBox { color: white; background-color: #222222; border: 1px solid #555555; padding: 2px; }
        """)
        self.sigma_spin.valueChanged.connect(self.image_visualizer.set_gaussian_sigma)
        self.image_control_layout.addWidget(self.sigma_label)
        self.image_control_layout.addWidget(self.sigma_spin)

        # （4）旋转角度下拉框
        self.rotation_label = QLabel("旋转角度：")
        self.rotation_label.setStyleSheet("color: white;")
        self.rotation_combo = QComboBox()
        self.rotation_combo.addItems(["0度", "90度", "180度", "270度"])
        self.rotation_combo.setStyleSheet("""
            QComboBox { color: white; background-color: #222222; border: 1px solid #555555; }
            QComboBox QAbstractItemView { background-color: #222222; color: white; }
        """)
        rotation_map = {0:0, 90:1, 180:2, 270:3}
        self.rotation_combo.setCurrentIndex(rotation_map[self.image_visualizer.rotation_angle])
        self.rotation_combo.currentIndexChanged.connect(
            lambda idx: self.image_visualizer.set_rotation_angle([0,90,180,270][idx])
        )
        self.image_control_layout.addWidget(self.rotation_label)
        self.image_control_layout.addWidget(self.rotation_combo)

        # （5）垂直翻转复选框
        self.flip_h_checkbox = QCheckBox("垂直翻转")
        self.flip_h_checkbox.setStyleSheet("color: white;")
        self.flip_h_checkbox.setChecked(self.image_visualizer.flip_horizontal)
        self.flip_h_checkbox.stateChanged.connect(
            lambda state: self.image_visualizer.set_flip_horizontal(state == QtCore.Qt.Checked)
        )
        self.image_control_layout.addWidget(self.flip_h_checkbox)

        # （6）水平翻转复选框
        self.flip_v_checkbox = QCheckBox("水平翻转")
        self.flip_v_checkbox.setStyleSheet("color: white;")
        self.flip_v_checkbox.setChecked(self.image_visualizer.flip_vertical)
        self.flip_v_checkbox.stateChanged.connect(
            lambda state: self.image_visualizer.set_flip_vertical(state == QtCore.Qt.Checked)
        )
        self.image_control_layout.addWidget(self.flip_v_checkbox)

        self.image_control_layout.addSpacerItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        self.image_container_layout.addWidget(self.image_control_panel)

        self.central_widget.addWidget(self.image_container)

        # 右侧：波形显示
        self.waveform_widget = QWidget()
        self.waveform_visualizer = RoutineWaveformVisualizer(self.waveform_widget)
        self.waveform_visualizer.set_reset_callback(self.reset_initial_value)
        self.central_widget.addWidget(self.waveform_widget)

        # 串口线程
        self.worker = SerialWorker(
            port=self.selected_port,
            save_data=self.save_data,
            normalization_low=0,
            normalization_high=3000
        )
        self.worker.data_ready.connect(self.image_visualizer.receive_data)
        self.worker.waveform_ready.connect(self.waveform_visualizer.update_plot)
        self.worker.error_signal.connect(self.show_serial_error)
        self.worker.start()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.image_visualizer.update_fps)
        self.timer.start(200)

    def reset_initial_value(self):
        if self.worker:
            self.worker.reset_initialization()

    def show_serial_error(self, error_msg):
        QMessageBox.critical(self, "串口错误", error_msg)
        self.close()

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