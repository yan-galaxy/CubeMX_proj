import sys
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)  # 忽略弃用警告
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QSplitter, QDialog, 
                             QVBoxLayout, QHBoxLayout, QComboBox, QPushButton, QMessageBox, QWidget, QLabel, QCheckBox, QSpacerItem, QSizePolicy, QDoubleSpinBox, QSpinBox, QFileDialog)
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import serial
from serial.tools import list_ports
from queue import Queue
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt
from scipy.ndimage import zoom
from scipy import signal, ndimage
import os
import threading
from datetime import datetime
import csv
# 不准改我的注释！！！不准删！！！

# 数据保存选项的宏定义（同步）
SAVE_DATA_DEFAULT = False  # 默认保存数据 True False
# 归一化默认最高值
NORMALIZATION_HIGH_VALUE = 1500
# 插值密度默认值 (2-15)
DEFAULT_ZOOM_FACTOR = 8
# 高斯核默认值
DEFAULT_GAUSSIAN_SIGMA = 0.0
# 旋转角度默认值 (0, 90, 180, 270对应索引)
DEFAULT_ROTATION_INDEX = 0  # 0度
# 翻转默认值
DEFAULT_FLIP_HORIZONTAL = True
DEFAULT_FLIP_VERTICAL = False

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
        """初始化滤波器状态（修复：返回初始化后的状态值）"""
        self.zi_low = signal.lfilter_zi(self.b_low, self.a_low) * initial_value
        return self.zi_low  # 关键修复：返回zi_low，用于DataReplayWorker初始化
    
    def apply_low_pass(self, data_point):
        filtered_point, self.zi_low = signal.lfilter(self.b_low, self.a_low, 
                                                   [data_point], zi=self.zi_low)
        return filtered_point[0]


class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 用于图像更新
    waveform_ready = pyqtSignal(list)  # 用于波形更新
    error_signal = pyqtSignal(str)  # 串口错误信号（跨平台错误提示）
    replay_finished = pyqtSignal()  # 回放完成信号

    def __init__(self, port=None, save_data=True, normalization_low=0, normalization_high=700, csv_path=None):
        super().__init__()
        self.port = port
        self.csv_path = csv_path  # CSV文件路径（回放用）
        self.baudrate = 460800
        self.ser = None
        self.running = False
        self.buffer = bytearray()
        self.FRAME_HEADER = b'\x55\xAA\xBB\xCC'
        self.FRAME_TAIL = b'\xAA\x55\x66\x77'
        self.matrix_init = None  # 初始化矩阵（用于归零）
        self.dead_value = 50  # 死点值
        self.normalization_low = normalization_low
        self.normalization_high = normalization_high

        # 1. 校准用最近帧缓冲区（排除分组，保留校准核心）
        self.recent_frames = []  # 存储最近帧用于校准
        self.max_recent_frames = 50  # 最多存储50帧
        self.init_frames = []  # 初始帧收集（用于首次初始化）
        self.init_frame_count = 0
        self.max_init_frames = 20  # 收集x帧用于初始化

        # 2. 数据保存配置（支持选择是否保存、CSV存储）
        self.save_data = save_data
        self.save_dir = "Routine_acq_raw_data"  # 10x10专属保存目录
        self.data_queue = Queue()
        self.writer_thread = None
        self.is_saving = False

        # 滤波器功能
        self.filter_handlers = {i: FilterHandler(fs=100.0, low_cutoff=40.0, order=8) for i in range(100)}
        self.filters_initialized = False

        # 初始化保存线程（仅当需要保存时）
        if self.save_data:
            self.init_raw_data_saving()

        self.is_stopped = False  # 防止stop重复执行的标记

    # 添加设置 normalization_high 的方法
    def set_normalization_high(self, value):
        self.normalization_high = value

    # 3. 数据保存初始化
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

    # 6. 停止逻辑（修改：加标记+按需打印）
    def stop(self):
        if self.is_stopped:  # 已停止则直接返回，避免重复执行
            return
        self.is_stopped = True  # 标记为已停止

        self.running = False
        # 停止保存线程（仅当save_data=True时处理）
        if self.save_data:
            self.is_saving = False
            if self.writer_thread and self.writer_thread.is_alive():
                self.writer_thread.join()
            # 仅当处理了保存线程时，才打印“保存完毕”
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


class DataReplayWorker(SerialWorker):
    def __init__(self, csv_path, save_data=False, normalization_low=0, normalization_high=1500):
        super().__init__(port=None, save_data=save_data, 
                        normalization_low=normalization_low, 
                        normalization_high=normalization_high, 
                        csv_path=csv_path)
        self.frame_index = 0  # 当前播放帧索引
        self.total_frames = 0  # 总帧数
        self.raw_data = None  # 存储CSV原始数据
        self.play_timer = None  # 定时器（在run中初始化）
        self.is_stopped = False  # 停止标记

    def run(self):
        try:
            # 1. 读取CSV数据（批量读取，提升效率）
            try:
                self.raw_data = np.loadtxt(self.csv_path, delimiter=',', skiprows=1)
                if self.raw_data.ndim != 2 or self.raw_data.shape[1] != 100:
                    raise ValueError(f"CSV格式错误，应为N行100列，实际为{self.raw_data.shape}")
                self.total_frames = self.raw_data.shape[0]
                print(f"成功加载CSV文件，共{self.total_frames}帧数据（预计播放{self.total_frames/100:.1f}秒）")
            except Exception as e:
                self.error_signal.emit(f"CSV文件读取失败: {str(e)}")
                return

            # 2. 初始化零位校准（复用前30帧）
            if self.matrix_init is None:
                init_count = min(self.max_init_frames, self.total_frames)
                init_data = self.raw_data[:init_count]
                self.matrix_init = np.mean(init_data, axis=0) + self.dead_value
                print(f"回放初始化完成，使用前{init_count}帧计算初始值")

            # 3. 预计算滤波器系数和初始状态（关键修复：正确获取zi值）
            if not self.filters_initialized:
                # 提取所有滤波器的系数
                self.b_lows = []
                self.a_lows = []
                self.zi_lows = []
                first_frame = self.raw_data[0]  # 用第一帧初始化滤波器状态
                for i in range(100):
                    fh = self.filter_handlers[i]
                    self.b_lows.append(fh.b_low)
                    self.a_lows.append(fh.a_low)
                    # 修复：调用initialize_states获取返回的zi值，而非使用无返回值的方法
                    zi = fh.initialize_states(first_frame[i])
                    self.zi_lows.append(zi)
                # 转换为numpy数组便于后续处理
                self.b_lows = np.array(self.b_lows)
                self.a_lows = np.array(self.a_lows)
                self.zi_lows = np.array(self.zi_lows)
                self.filters_initialized = True

            # 4. 在子线程内创建并启动定时器（确保线程亲和性）
            self.play_timer = QTimer()
            # self.play_timer.setParent(self)  # 关键：将定时器父对象设为当前线程对象
            self.play_timer.setInterval(10)  # 10ms = 100FPS
            self.play_timer.timeout.connect(self.process_frame)
            self.play_timer.start()

            # 启动子线程事件循环（处理定时器事件）
            self.exec_()  # 此处会阻塞，直到调用quit()

            # 5. 清理工作（在子线程内完成）
            if self.play_timer and self.play_timer.isActive():
                self.play_timer.stop()
            self.play_timer = None  # 直接置空，避免deleteLater跨线程问题
            # self.stop()
            self.replay_finished.emit()
            print("数据回放完成")

        except Exception as e:
            self.error_signal.emit(f"回放异常: {str(e)}")

    def process_frame(self):
        """向量化解码和滤波，提升效率"""
        if self.is_stopped or self.frame_index >= self.total_frames:
            self.quit()  # 退出事件循环（关键：手动关闭时也能触发）
            return

        # 1. 按10帧一组读取（与实时采集逻辑一致）
        remaining = self.total_frames - self.frame_index
        take_count = min(10, remaining)
        current_frames = self.raw_data[self.frame_index : self.frame_index + take_count]
        self.frame_index += take_count

        # 2. 10帧取平均（批量计算）
        average = np.mean(current_frames, axis=0)  # 形状(100,)

        # 3. 校准缓冲区更新
        self.recent_frames.append(average.copy())
        if len(self.recent_frames) > self.max_recent_frames:
            self.recent_frames.pop(0)

        # 4. 滤波处理（修复：确保zi参数有效）
        filtered_results = np.zeros(100)
        for i in range(100):
            # 确保b、a、zi都是有效数组
            b = self.b_lows[i]
            a = self.a_lows[i]
            zi = self.zi_lows[i]
            # 调用lfilter，此时zi有效，返回2个值
            filtered, self.zi_lows[i] = signal.lfilter(b, a, [average[i]], zi=zi)
            filtered_results[i] = filtered[0]

        # 5. 批量归一化处理
        zeroed = filtered_results - self.matrix_init
        clipped = np.clip(zeroed, self.normalization_low, self.normalization_high)
        normalized = (clipped - self.normalization_low) / (self.normalization_high - self.normalization_low)

        # 6. 发射信号（使用queued连接确保线程安全）
        self.waveform_ready.emit(filtered_results.tolist())
        self.data_ready.emit(normalized.tolist())

    def stop(self):
        """安全停止回放（关键修复：主动停止定时器和事件循环）"""
        self.is_stopped = True
        # 1. 主动停止定时器（避免定时器继续触发process_frame）
        if self.play_timer and self.play_timer.isActive():
            self.play_timer.stop()
        # 2. 主动退出事件循环（确保exec_()阻塞解除）
        self.quit()
        # 3. 调用父类停止逻辑（处理保存线程等）
        # super().stop()


# 9. 波形可视化（支持10行选择+校准按钮+界面样式，排除分组）
class RoutineWaveformVisualizer:
    def __init__(self, parent_widget):
        self.parent_widget = parent_widget
        self.selected_row = 0  # 修改默认选择为平均值（索引0）
        self.show_all_average = True  # 默认显示所有点的平均值
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
        self.plot.setTitle("所有点位平均值")  # 修改默认标题
        self.plot.setLabels(left='数值', bottom='帧编号')
        self.plot.showGrid(x=True, y=True)
        self.plot.setYRange(0, 4095)  # 适配16位数据  4095  700
        self.main_layout.addWidget(self.plot_widget)

        # 控件布局（水平：行选择+校准按钮）
        self.control_layout = QHBoxLayout()
        # 行选择下拉框（10行：第1行~第10行）
        self.row_selector = QComboBox()
        row_names = ["平均值"] + [f"第{i+1}列" for i in range(10)]
        self.row_selector.addItems(row_names)
        self.row_selector.setCurrentIndex(self.selected_row)  # 设置默认选择为平均值
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
            'c',        # 青色（鲜明）
            'y',        # 黄色（明亮）
            'm',        # 品红（艳丽）
            'r',        # 红色（醒目）
            'lime',     # 亮绿（比普通绿色更鲜艳）
            'royalblue',# 宝蓝（比普通蓝色更亮）
            'orange',   # 橙色（明快）
            'hotpink',  # 亮粉色（替换紫色，鲜艳且偏暖调，区别度高）
            'turquoise',# 青绿色（替换绯红，鲜艳且偏冷调，与现有蓝绿系颜色区分明显）
            'aqua'      # 水蓝（比青色更透亮）
        ]
        for i in range(10):
            pen = pg.mkPen(color=colors[i], width=2) 
            curve = self.plot.plot(pen=pen, name=f'点{i+1}')
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
        # 检查是否选择的是平均值选项（索引0）
        if self.selected_row == 0:
            self.show_all_average = True
            self.plot.setTitle("所有点位平均值")
        else:
            self.show_all_average = False
            self.plot.setTitle(f"第{self.selected_row}列波形（10个点）")
        # 重置当前行的波形数据显示
        self.show_data = [[0] * 300 for _ in range(10)]

    # 更新波形（显示选中行的10个点）
    def update_plot(self, new_row):
        if len(new_row) == 100:
            if self.show_all_average:
                # 计算所有100个点的平均值
                avg_value = sum(new_row) / len(new_row) #-3400
                # # 计算所有100个点的最大值
                # max_value = max(new_row)
                
                # 只更新第一条曲线的数据，其余曲线清空（不显示）
                self.show_data[0] = self.show_data[0][1:] + [avg_value]
                x_data = np.arange(len(self.show_data[0]))
                y_data = np.array(self.show_data[0])
                self.curves[0].setData(x_data, y_data)
                
                # 清空其余9条曲线的数据，使其不显示
                for i in range(1, 10):
                    self.curves[i].setData([], [])
            else:
                # 提取选中行的10个点数据（行优先：第selected_row行的索引为selected_row*10 ~ (selected_row+1)*10 -1）
                row_start_idx = ( self.selected_row - 1 ) * 10
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
        self.layout.addItem(self.plot, 0, 0, 1, 2)

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
        self.setWindowTitle("选择串口或数据回放")
        self.setFixedSize(500, 350)  # 增加高度容纳新按钮
        self.selected_port = None
        self.save_data = SAVE_DATA_DEFAULT
        self.is_replay = False  # 标记是否选择回放
        self.selected_csv = None  # 选中的CSV文件路径

        self.setStyleSheet("""
            /* 窗口：圆角+轻微阴影，背景用柔和浅灰 */
            QDialog {
                background-color: #f5f5f5;
                border-radius: 12px; /* 大圆角 */
            }

            /* 下拉框：圆角+细边框，hover时边框变色 */
            QComboBox {
                background-color: white;
                border: 2px solid #dddddd;
                border-radius: 8px; /* 圆角 */
                padding: 6px 10px;
                margin: 8px 0;
                min-width: 250px;
                color: black; /* 文本颜色 */
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
                selection-background-color: #88c9ff; /* 选中项背景色 */
                selection-color: black; /* 选中项文字颜色 */
            }
            
            /* 下拉列表项样式 */
            QComboBox QAbstractItemView::item {
                color: black; /* 默认文字颜色 */
                padding: 4px;
            }
            QComboBox QAbstractItemView::item:hover {
                background-color: #e6f0ff; /* 悬停背景色 */
                color: black; /* 悬停文字颜色 */
            }

            /* 复选框：圆角勾选框，颜色柔和 */
            QCheckBox {
                margin: 8px 0;
                spacing: 8px; /* 文字与框间距 */
                color: black; /* 文本颜色 */
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
            /* 回放按钮特殊样式（调暗，降低亮度） */
            QPushButton#ReplayBtn {
                background-color: #66cc99; /* 较暗的绿色，原#88ffb4的柔和版 */
            }
            QPushButton#ReplayBtn:hover {
                background-color: #55b388; /*  hover时稍深，保持协调 */
            }
        """)

        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(25, 20, 25, 20)  # 内边距，让控件不贴边
        self.setLayout(self.layout)

        # 串口选择下拉框
        self.port_combo = QComboBox()
        self.port_combo.setPlaceholderText("请选择可用串口（实时采集用）")
        self.layout.addWidget(self.port_combo)

        # 保存数据复选框
        self.save_data_checkbox = QCheckBox("保存数据到CSV（仅实时采集）")
        self.save_data_checkbox.setChecked(SAVE_DATA_DEFAULT)
        self.layout.addWidget(self.save_data_checkbox)

        # 按钮布局（垂直，按钮占满宽度）
        self.button_layout = QVBoxLayout()
        self.confirm_btn = QPushButton("确认选择（实时采集）")
        self.cancel_btn = QPushButton("取消")
        # 数据回放按钮（带间距）
        self.replay_spacer = QSpacerItem(20, 10, QSizePolicy.Minimum, QSizePolicy.Fixed)
        self.replay_btn = QPushButton("数据回放（选择CSV文件）")
        self.replay_btn.setObjectName("ReplayBtn")  # 用于样式区分

        # 按钮添加顺序：确认→取消→间距→回放
        self.button_layout.addWidget(self.confirm_btn)
        self.button_layout.addWidget(self.cancel_btn)
        self.button_layout.addItem(self.replay_spacer)
        self.button_layout.addWidget(self.replay_btn)
        
        self.layout.addLayout(self.button_layout)

        # 信号绑定
        self.confirm_btn.clicked.connect(self.on_confirm)
        self.cancel_btn.clicked.connect(self.reject)
        self.replay_btn.clicked.connect(self.on_replay)  # 回放按钮绑定

        # 加载可用串口（即使无串口也显示对话框）
        self.load_available_ports()

    def load_available_ports(self):
        self.port_combo.clear()
        available_ports = list(list_ports.comports())
        
        if not available_ports:
            self.port_combo.addItem("未检测到可用串口", None)
            self.confirm_btn.setEnabled(False)  # 无串口时禁用实时采集
        else:
            # 按照系统类型进行不同的排序
            if sys.platform.startswith('linux') or sys.platform.startswith('darwin'):
                # Linux 或 macOS 系统
                ttyacm_ports = []       # ttyACM 设备 (Arduino等)
                ttyusb_ports = []       # ttyUSB 设备 (USB转串口等)
                other_ports = []        # 其他设备
                
                for port in available_ports:
                    if "ttyACM" in port.device:
                        ttyacm_ports.append(port)
                    elif "ttyUSB" in port.device:
                        ttyusb_ports.append(port)
                    else:
                        other_ports.append(port)
                
                # 对 ttyACM 和 ttyUSB 设备按数字顺序排序
                def extract_port_number(port):
                    import re
                    match = re.search(r'\d+$', port.device)
                    return int(match.group()) if match else 0
                    
                ttyacm_ports_sorted = sorted(ttyacm_ports, key=extract_port_number)
                ttyusb_ports_sorted = sorted(ttyusb_ports, key=extract_port_number)
                sorted_ports = ttyacm_ports_sorted + ttyusb_ports_sorted + other_ports
            else:
                # Windows 系统按原来的方式排序
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
            self.confirm_btn.setEnabled(True)

    def on_confirm(self):
        """实时采集确认"""
        self.selected_port = self.port_combo.currentData()
        self.save_data = self.save_data_checkbox.isChecked()
        self.is_replay = False
        self.accept()

    def on_replay(self):
        """数据回放：打开文件选择对话框"""
        csv_path, _ = QFileDialog.getOpenFileName(
            self, "选择CSV数据文件", "", "CSV Files (*.csv);;All Files (*)"
        )
        if csv_path and os.path.exists(csv_path):
            self.selected_csv = csv_path
            self.is_replay = True
            self.accept()
        else:
            QMessageBox.warning(self, "警告", "请选择有效的CSV文件")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.worker = None
        self.selected_port = None
        self.save_data = SAVE_DATA_DEFAULT
        self.is_replay_mode = False  # 标记是否为回放模式

        # 先显示选择对话框（无论有无串口都会显示）
        if not self.select_serial_or_replay():
            sys.exit(0)

        self.init_main_ui()

    def select_serial_or_replay(self):
        """选择实时采集或数据回放"""
        dialog = SerialSelectionDialog()
        if dialog.exec_() == QDialog.Accepted:
            self.is_replay_mode = dialog.is_replay
            if self.is_replay_mode:
                self.selected_csv = dialog.selected_csv  # 回放模式：保存CSV路径
                return True
            else:
                self.selected_port = dialog.selected_port  # 实时模式：保存串口
                self.save_data = dialog.save_data
                return self.selected_port is not None
        else:
            return False

    def init_main_ui(self):
        self.setWindowTitle("10x10传感器 - 图像与波形双视图")
        self.resize(1500, 800)
        self.showMaximized()  # 窗口最大化

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
            rotation_angle=[0, 90, 180, 270][DEFAULT_ROTATION_INDEX], 
            flip_horizontal=DEFAULT_FLIP_HORIZONTAL, 
            flip_vertical=DEFAULT_FLIP_VERTICAL,
            zoom_factor=DEFAULT_ZOOM_FACTOR,  # 默认插值密度
            gaussian_sigma=DEFAULT_GAUSSIAN_SIGMA  # 默认高斯核
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
        self.interp_checkbox.setStyleSheet("""
            color: white;
            background-color: #222222;
            border: 1px solid #555555;
            padding: 5px;
        """)
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
            QSpinBox { 
                color: white; 
                background-color: #222222; 
                border: 1px solid #555555; 
                padding: 2px; 
            }
            QSpinBox::up-button {
                background-color: #444444;
                border: 1px solid #666666;
                width: 16px;
            }
            QSpinBox::down-button {
                background-color: #444444;
                border: 1px solid #666666;
                width: 16px;
            }
            QSpinBox::up-button:hover, QSpinBox::down-button:hover {
                background-color: #555555;
            }
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
            QDoubleSpinBox { 
                color: white; 
                background-color: #222222; 
                border: 1px solid #555555; 
                padding: 2px; 
            }
            QDoubleSpinBox::up-button {
                background-color: #444444;
                border: 1px solid #666666;
                width: 16px;
            }
            QDoubleSpinBox::down-button {
                background-color: #444444;
                border: 1px solid #666666;
                width: 16px;
            }
            QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {
                background-color: #555555;
            }
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
            QComboBox { 
                color: white; 
                background-color: #222222; 
                border: 1px solid #555555;
                padding: 2px 5px;
            }
            QComboBox::drop-down {
                background-color: #444444;
                border: 1px solid #666666;
                width: 20px;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 4px solid transparent;
                border-right: 4px solid transparent;
                border-top: 4px solid white;
                width: 0;
                height: 0;
                margin: 6px;
            }
            QComboBox QAbstractItemView { 
                background-color: #222222; 
                color: white; 
                border: 1px solid #555555;
                selection-background-color: #444444;
            }
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
        self.flip_h_checkbox.setStyleSheet("""
            color: white;
            background-color: #222222;
            border: 1px solid #555555;
            padding: 5px;
        """)
        self.flip_h_checkbox.setChecked(self.image_visualizer.flip_horizontal)
        self.flip_h_checkbox.stateChanged.connect(
            lambda state: self.image_visualizer.set_flip_horizontal(state == QtCore.Qt.Checked)
        )
        self.image_control_layout.addWidget(self.flip_h_checkbox)

        # （6）水平翻转复选框
        self.flip_v_checkbox = QCheckBox("水平翻转")
        self.flip_v_checkbox.setStyleSheet("""
            color: white;
            background-color: #222222;
            border: 1px solid #555555;
            padding: 5px;
        """)
        self.flip_v_checkbox.setChecked(self.image_visualizer.flip_vertical)
        self.flip_v_checkbox.stateChanged.connect(
            lambda state: self.image_visualizer.set_flip_vertical(state == QtCore.Qt.Checked)
        )
        self.image_control_layout.addWidget(self.flip_v_checkbox)

        # （7）Normalization High 参数输入框
        self.norm_high_label = QLabel("最大值:")
        self.norm_high_label.setStyleSheet("color: white;")
        self.norm_high_input = QSpinBox()
        self.norm_high_input.setRange(1, 5000)  # 设置合理范围
        self.norm_high_input.setSingleStep(100)
        self.norm_high_input.setValue(NORMALIZATION_HIGH_VALUE)  # 默认值
        self.norm_high_input.setStyleSheet("""
            QSpinBox { 
                color: white; 
                background-color: #222222; 
                border: 1px solid #555555; 
                padding: 2px; 
            }
            QSpinBox::up-button {
                background-color: #444444;
                border: 1px solid #666666;
                width: 16px;
            }
            QSpinBox::down-button {
                background-color: #444444;
                border: 1px solid #666666;
                width: 16px;
            }
            QSpinBox::up-button:hover, QSpinBox::down-button:hover {
                background-color: #555555;
            }
        """)
        self.norm_high_input.valueChanged.connect(self.set_normalization_high)
        self.image_control_layout.addWidget(self.norm_high_label)
        self.image_control_layout.addWidget(self.norm_high_input)

        self.image_control_layout.addSpacerItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        self.image_container_layout.addWidget(self.image_control_panel)

        self.central_widget.addWidget(self.image_container)

        # 右侧：波形显示
        self.waveform_widget = QWidget()
        self.waveform_visualizer = RoutineWaveformVisualizer(self.waveform_widget)
        self.waveform_visualizer.set_reset_callback(self.reset_initial_value)
        self.central_widget.addWidget(self.waveform_widget)

        # 根据模式创建Worker（实时采集/数据回放）
        if self.is_replay_mode:
            # 回放模式：使用DataReplayWorker（继承SerialWorker）
            self.worker = DataReplayWorker(
                csv_path=self.selected_csv,
                save_data=False,  # 回放不保存数据
                normalization_low=0,
                normalization_high=self.norm_high_input.value()  # 使用界面设置的值
            )
            # 回放完成后自动关闭程序
            self.worker.replay_finished.connect(self.close)
            # 修复：仅绑定一次带QueuedConnection的信号
            self.worker.data_ready.connect(self.image_visualizer.receive_data, Qt.QueuedConnection)
            self.worker.waveform_ready.connect(self.waveform_visualizer.update_plot, Qt.QueuedConnection)
        else:
            # 实时模式：原有SerialWorker
            self.worker = SerialWorker(
                port=self.selected_port,
                save_data=self.save_data,
                normalization_low=0,
                normalization_high=self.norm_high_input.value()  # 使用界面设置的值
            )
            # 实时模式绑定默认信号
            self.worker.data_ready.connect(self.image_visualizer.receive_data)
            self.worker.waveform_ready.connect(self.waveform_visualizer.update_plot)

        # 错误信号绑定（通用）
        self.worker.error_signal.connect(self.show_serial_error)
        self.worker.start()

        # FPS更新定时器（复用原有逻辑）
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.image_visualizer.update_fps)
        self.timer.start(200)

    def set_normalization_high(self, value):
        """设置 normalization_high 值"""
        if self.worker:
            self.worker.set_normalization_high(value)

    def reset_initial_value(self):
        if self.worker:
            self.worker.reset_initialization()

    def show_serial_error(self, error_msg):
        QMessageBox.critical(self, "错误", error_msg)
        self.close()

    def closeEvent(self, event):
        self.hide()  # 先隐藏界面，提升用户体验
        if self.worker:
            # 1. 停止worker（触发定时器停止和事件循环退出）
            self.worker.stop()
            # 2. 等待线程完全终止（最多等待2秒，避免无限阻塞）
            if not self.worker.wait(2000):
                # 若超时，强制终止线程（极端情况兜底）
                self.worker.terminate()
                print("线程超时，已强制终止")
        event.accept()  # 确认关闭事件


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec_())