import sys
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
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
import struct
import time
from collections import deque

# 配置参数
SAVE_DATA_DEFAULT = False
NORMALIZATION_HIGH_VALUE = 1500
DEFAULT_ZOOM_FACTOR = 8
DEFAULT_GAUSSIAN_SIGMA = 0.0
DEFAULT_ROTATION_INDEX = 1
DEFAULT_FLIP_HORIZONTAL = True
DEFAULT_FLIP_VERTICAL = True
FRAMES_PER_PACKET = 100  # 5MIC 每包帧数

class FilterHandler:
    def __init__(self, fs, low_cutoff, order=4):
        self.fs = fs
        self.nyquist = 0.5 * fs
        normal_low_cutoff = low_cutoff / self.nyquist
        self.b_low, self.a_low = signal.bessel(order, normal_low_cutoff, 
                                              btype='low', analog=False)
        self.zi_low = None
    
    def initialize_states(self, initial_value):
        self.zi_low = signal.lfilter_zi(self.b_low, self.a_low) * initial_value
        return self.zi_low
    
    def apply_low_pass(self, data_point):
        filtered_point, self.zi_low = signal.lfilter(self.b_low, self.a_low, 
                                                   [data_point], zi=self.zi_low)
        return filtered_point[0]
    
    # 批量处理整帧数据（替代逐点处理）
    def apply_low_pass_batch(self, data):
        if self.zi_low is None:
            # 初始化状态（用第一帧数据的第一个值）
            self.zi_low = signal.lfilter_zi(self.b_low, self.a_low) * data[0]
        # 批量滤波，返回滤波后的数据和新状态
        filtered_data, self.zi_low = signal.lfilter(self.b_low, self.a_low, data, zi=self.zi_low)
        return filtered_data

class CombinedSerialWorker(QThread):
    # 常规数据信号
    routine_data_ready = pyqtSignal(list)
    routine_waveform_ready = pyqtSignal(list)
    # 5MIC 数据信号
    mic_data_ready = pyqtSignal(list)
    mic_waveform_ready = pyqtSignal(list)
    # 通用信号
    error_signal = pyqtSignal(str)
    replay_finished = pyqtSignal()
    fps_ready = pyqtSignal(float)

    def __init__(self, port=None, save_data=True, normalization_low=0, normalization_high=700, csv_path=None, save_mic_data=False):
        super().__init__()
        self.port = port
        self.csv_path = csv_path
        self.baudrate = 460800
        self.ser = None
        self.running = False
        self.buffer = bytearray()
        self.FRAME_HEADER = b'\x55\xAA\xBB\xCC'
        self.FRAME_TAIL = b'\xAA\x55\x66\x77'
        
        # 常规数据相关
        self.routine_matrix_init = None
        self.dead_value = 0
        self.normalization_low = normalization_low
        self.normalization_high = normalization_high
        self.recent_frames = []
        self.max_recent_frames = 50
        # self.recent_frames = deque(maxlen=self.max_recent_frames)
        self.init_frames = []
        self.init_frame_count = 0
        self.max_init_frames = 20
        # self.init_frames = deque(maxlen=self.max_init_frames)
        
        # 5MIC 数据相关
        self.mic_matrix_init = []
        self.mic_matrix_flag = 0
        self.mic_normalization_low = 0
        self.mic_normalization_high = 3.5
        
        # 保存相关
        self.save_data = save_data
        self.save_mic_data = save_mic_data
        self.routine_save_dir = "Routine_acq_raw_data_6x6"
        self.mic_save_dir = "5MIC_raw_data"
        self.routine_data_queue = Queue()
        self.mic_data_queue = Queue()
        self.routine_writer_thread = None
        self.mic_writer_thread = None
        self.is_saving_routine = False
        self.is_saving_mic = False
        
        # 滤波器
        self.filter_handlers = {i: FilterHandler(fs=100.0, low_cutoff=30.0, order=8) for i in range(36)}
        self.filters_initialized = False
        
        # 帧率计算
        self.frame_count = 0
        self.last_time = time.time()
        self.fps_update_interval = 1.0
        
        # 回放相关
        self.is_replay = csv_path is not None
        self.frame_index = 0
        self.total_frames = 0
        self.raw_data = None
        self.play_timer = None
        self.is_stopped = False

        if self.save_data:
            self.init_routine_data_saving()
        if self.save_mic_data:
            self.init_mic_data_saving()

    def init_routine_data_saving(self):
        if not os.path.exists(self.routine_save_dir):
            os.makedirs(self.routine_save_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.routine_save_dir, f"raw_data_6x6_{timestamp}.csv")
        self.is_saving_routine = True
        self.routine_writer_thread = threading.Thread(target=self.write_routine_data_to_file, daemon=True)
        self.routine_writer_thread.start()

    def init_mic_data_saving(self):
        if not os.path.exists(self.mic_save_dir):
            os.makedirs(self.mic_save_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.mic_csv_path = os.path.join(self.mic_save_dir, f"mic_{timestamp}.csv")
        self.is_saving_mic = True
        with open(self.mic_csv_path, 'w', encoding='utf-8') as f:
            header = ','.join([f'MIC{i+1}' for i in range(5)])
            f.write(header + '\n')
        self.mic_writer_thread = threading.Thread(target=self.write_mic_data_to_csv, daemon=True)
        self.mic_writer_thread.start()

    def write_routine_data_to_file(self):
        with open(self.csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            headers = [f"point{i}" for i in range(36)]
            writer.writerow(headers)
            while self.is_saving_routine:
                if not self.routine_data_queue.empty():
                    frames = self.routine_data_queue.get()
                    flattened_data = frames.reshape(10, 36)
                    writer.writerows(flattened_data.tolist())
                else:
                    self.msleep(100)

    def write_mic_data_to_csv(self):
        while self.is_saving_mic:
            try:
                parsed_data = self.mic_data_queue.get(block=True, timeout=None)
                points_per_channel = FRAMES_PER_PACKET
                reshaped_data = np.zeros((points_per_channel, 5), dtype=np.uint16)
                for frame_idx in range(points_per_channel):
                    frame_data = parsed_data[frame_idx*5 : (frame_idx+1)*5]
                    reshaped_data[frame_idx, :] = frame_data
                with open(self.mic_csv_path, 'a', encoding='utf-8') as f:
                    np.savetxt(f, reshaped_data, delimiter=',', fmt='%d')
                self.mic_data_queue.task_done()
            except Exception as e:
                if self.is_saving_mic:
                    print(f"5MIC 写入异常: {e}")
                break

    def reset_initialization(self):
        if len(self.recent_frames) > 0:
            self.routine_matrix_init = np.mean(self.recent_frames, axis=0) + self.dead_value
            print("常规数据已校准零位")

    def stop(self):
        if self.is_stopped:
            return
        self.is_stopped = True
        self.running = False
        
        # 停止常规数据保存
        if self.save_data:
            self.is_saving_routine = False
            if self.routine_writer_thread and self.routine_writer_thread.is_alive():
                self.routine_writer_thread.join()
            print('常规数据保存完毕')
        
        # 停止 5MIC 数据保存
        if self.save_mic_data:
            self.is_saving_mic = False
            if self.mic_data_queue.empty():
                self.mic_data_queue.put(np.array([]))
            if self.mic_writer_thread and self.mic_writer_thread.is_alive():
                self.mic_writer_thread.join(timeout=2)
            print(f"5MIC 数据保存完毕，文件路径：{self.mic_csv_path}")
        
        # 停止回放
        if self.is_replay and self.play_timer and self.play_timer.isActive():
            self.play_timer.stop()
        self.quit()

    def run(self):
        if self.is_replay:
            self.run_replay()
        else:
            self.run_realtime()

    def run_realtime(self):
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
                            
                            # 处理 900 字节数据包
                            if len(parsed) == 900:  # 900 个 uint16
                                # 解析常规数据（前 400 个 uint16）
                                routine_parsed = parsed[:400]
                                self.process_routine_data(routine_parsed)
                                
                                # 解析 5MIC 数据（后 500 个 uint16）
                                mic_parsed = parsed[400:]
                                self.process_mic_data(mic_parsed)
                                
                                # 计算帧率
                                self.update_fps()
                            
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

    def run_replay(self):
        try:
            self.raw_data = np.loadtxt(self.csv_path, delimiter=',', skiprows=1)
            if self.raw_data.ndim != 2 or self.raw_data.shape[1] != 36:
                raise ValueError(f"CSV格式错误，应为N行36列，实际为{self.raw_data.shape}")
            self.total_frames = self.raw_data.shape[0]
            print(f"成功加载CSV文件，共{self.total_frames}帧数据")

            if self.routine_matrix_init is None:
                init_count = min(self.max_init_frames, self.total_frames)
                init_data = self.raw_data[:init_count]
                self.routine_matrix_init = np.mean(init_data, axis=0) + self.dead_value
                print(f"回放初始化完成，使用前{init_count}帧计算初始值")

            if not self.filters_initialized:
                self.b_lows = []
                self.a_lows = []
                self.zi_lows = []
                first_frame = self.raw_data[0]
                for i in range(36):
                    fh = self.filter_handlers[i]
                    self.b_lows.append(fh.b_low)
                    self.a_lows.append(fh.a_low)
                    zi = fh.initialize_states(first_frame[i])
                    self.zi_lows.append(zi)
                self.b_lows = np.array(self.b_lows)
                self.a_lows = np.array(self.a_lows)
                self.zi_lows = np.array(self.zi_lows)
                self.filters_initialized = True

            self.play_timer = QTimer()
            self.play_timer.setInterval(10)
            self.play_timer.timeout.connect(self.process_replay_frame)
            self.play_timer.start()
            self.exec_()

            if self.play_timer and self.play_timer.isActive():
                self.play_timer.stop()
            self.replay_finished.emit()
            print("数据回放完成")
        except Exception as e:
            self.error_signal.emit(f"回放异常: {str(e)}")

    def process_replay_frame(self):
        if self.is_stopped or self.frame_index >= self.total_frames:
            self.quit()
            return

        remaining = self.total_frames - self.frame_index
        take_count = min(10, remaining)
        current_frames = self.raw_data[self.frame_index : self.frame_index + take_count]
        self.frame_index += take_count

        average = np.mean(current_frames, axis=0)
        self.recent_frames.append(average.copy())
        if len(self.recent_frames) > self.max_recent_frames:
            self.recent_frames.pop(0)

        filtered_results = np.zeros(36)
        for i in range(36):
            b = self.b_lows[i]
            a = self.a_lows[i]
            zi = self.zi_lows[i]
            filtered, self.zi_lows[i] = signal.lfilter(b, a, [average[i]], zi=zi)
            filtered_results[i] = filtered[0]

        zeroed = filtered_results - self.routine_matrix_init
        clipped = np.clip(zeroed, self.normalization_low, self.normalization_high)
        normalized = (clipped - self.normalization_low) / (self.normalization_high - self.normalization_low)

        self.routine_waveform_ready.emit(filtered_results.tolist())
        self.routine_data_ready.emit(normalized.tolist())
        self.update_fps()

    def process_routine_data(self, parsed):
        if len(parsed) == 400:
            frames = parsed.reshape(10, 40)
            valid_frames = frames[:, :36]
            average = np.mean(valid_frames, axis=0)
            
            if self.save_data:
                self.routine_data_queue.put(frames.copy())
            
            self.recent_frames.append(average.copy())
            if len(self.recent_frames) > self.max_recent_frames:
                self.recent_frames.pop(0)
            
            if self.routine_matrix_init is None and self.init_frame_count < self.max_init_frames:
                self.init_frames.append(average.copy())
                self.init_frame_count += 1
                if self.init_frame_count == self.max_init_frames:
                    self.routine_matrix_init = np.mean(self.init_frames, axis=0) + self.dead_value
                    print(f"常规数据初始化完成，使用{self.max_init_frames}帧计算初始值")
            
            if not self.filters_initialized and self.routine_matrix_init is not None:
                for i in range(36):
                    self.filter_handlers[i].initialize_states(average[i])
                self.filters_initialized = True
            
            if self.routine_matrix_init is not None and self.filters_initialized:
                filtered_results = [self.filter_handlers[i].apply_low_pass(average[i]) for i in range(36)]
                zeroed_results = np.array(filtered_results) - self.routine_matrix_init
                clipped_result = np.clip(zeroed_results, self.normalization_low, self.normalization_high)
                normalized_result = (clipped_result - self.normalization_low) / (self.normalization_high - self.normalization_low)
                
                self.routine_waveform_ready.emit(filtered_results)
                self.routine_data_ready.emit(normalized_result.tolist())

    def process_mic_data(self, parsed):
        expected_length = FRAMES_PER_PACKET * 5
        if len(parsed) == expected_length:
            if self.save_mic_data:
                self.mic_data_queue.put(parsed.copy())
            
            if self.mic_matrix_flag == 0:
                self.mic_matrix_init = parsed.copy()
                self.mic_matrix_flag = 1
            else:
                result = parsed
                clipped_result = np.clip(result, self.mic_normalization_low, self.mic_normalization_high)
                normalized_result = (clipped_result - self.mic_normalization_low) / (self.mic_normalization_high - self.mic_normalization_low)
                
                self.mic_waveform_ready.emit(parsed.tolist())
                self.mic_data_ready.emit(normalized_result.tolist())

    def update_fps(self):
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_time >= self.fps_update_interval:
            fps = self.frame_count / (current_time - self.last_time)
            self.fps_ready.emit(fps)
            self.frame_count = 0
            self.last_time = current_time

class CombinedWaveformVisualizer:
    def __init__(self, parent_widget):
        self.parent_widget = parent_widget
        self.display_mode = "mic"  # "routine" 或 "mic"
        self.routine_selected_row = 0
        self.routine_show_all_average = True
        self.reset_callback = None

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

        self.main_layout = QVBoxLayout()
        self.parent_widget.setLayout(self.main_layout)

        # 波形显示切换控件
        self.control_layout = QHBoxLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["压阻单元波形", "5麦克风波形"])
        self.mode_combo.setCurrentIndex(1)  # 默认选择5麦克风波形
        self.mode_combo.currentTextChanged.connect(self.change_display_mode)
        self.control_layout.addWidget(QLabel("波形显示:", styleSheet="color:white;"))
        self.control_layout.addWidget(self.mode_combo)
        
        # 压阻单元波形控制
        self.row_selector = QComboBox()
        row_names = ["平均值"] + [f"第{i+1}行" for i in range(4)]
        self.row_selector.addItems(row_names)
        self.row_selector.setCurrentIndex(self.routine_selected_row)
        self.row_selector.currentIndexChanged.connect(self.change_routine_row)
        self.control_layout.addWidget(self.row_selector)
        
        # 校准按钮
        self.reset_button = QPushButton("校准零位")
        self.reset_button.clicked.connect(self.on_reset_clicked)
        self.control_layout.addWidget(self.reset_button)
        
        self.control_layout.addStretch()
        self.main_layout.addLayout(self.control_layout)

        # 绘图区域
        self.plot_widget = pg.PlotWidget()
        self.plot = self.plot_widget.getPlotItem()
        self.plot.setTitle("压阻单元波形 - 平均值")
        self.plot.setLabels(left='数值', bottom='帧编号')
        self.plot.showGrid(x=True, y=True)
        self.plot.setYRange(0, 4095)
        self.main_layout.addWidget(self.plot_widget)

        # 压阻单元波形曲线
        self.routine_curves = []
        routine_colors = ['c', 'y', 'm', 'r', 'lime', 'royalblue', 'orange', 'hotpink', 'turquoise']
        for i in range(9):
            pen = pg.mkPen(color=routine_colors[i], width=2)
            curve = self.plot.plot(pen=pen, name=f'点{i+1}')
            self.routine_curves.append(curve)
        
        # 5MIC 波形曲线
        self.mic_curves = []
        mic_colors = ['r', 'g', 'b', 'y', 'm']
        for i in range(5):
            pen = pg.mkPen(color=mic_colors[i], width=2)
            curve = self.plot.plot(pen=pen, name=f'MIC{i+1}')
            self.mic_curves.append(curve)
        
        # 数据缓冲区
        self.routine_show_data = [[0] * 300 for _ in range(9)]
        self.mic_show_data = [[0] * (FRAMES_PER_PACKET * 10) for _ in range(5)]



        # # 初始隐藏 5MIC 曲线
        # self.hide_mic_curves()

        # 初始隐藏压阻单元波形曲线，显示5MIC波形曲线
        self.hide_routine_curves()
        self.show_mic_curves()
        self.row_selector.setVisible(False)

    def set_reset_callback(self, callback):
        self.reset_callback = callback

    def on_reset_clicked(self):
        if self.reset_callback:
            self.reset_callback()

    def change_display_mode(self, mode):
        if mode == "压阻单元波形":
            self.display_mode = "routine"
            self.plot.setTitle("压阻单元波形 - 平均值")
            self.show_routine_curves()
            self.hide_mic_curves()
            self.row_selector.setVisible(True)
        else:
            self.display_mode = "mic"
            self.plot.setTitle("5麦克风波形")
            self.show_mic_curves()
            self.hide_routine_curves()
            self.row_selector.setVisible(False)

    def change_routine_row(self, index):
        self.routine_selected_row = index
        if self.routine_selected_row == 0:
            self.routine_show_all_average = True
            self.plot.setTitle("压阻单元波形 - 平均值")
        else:
            self.routine_show_all_average = False
            self.plot.setTitle(f"压阻单元波形 - 第{self.routine_selected_row}行波形（9个点）")
        self.routine_show_data = [[0] * 300 for _ in range(9)]

    def show_routine_curves(self):
        for curve in self.routine_curves:
            curve.setVisible(True)

    def hide_routine_curves(self):
        for curve in self.routine_curves:
            curve.setVisible(False)

    def show_mic_curves(self):
        for curve in self.mic_curves:
            curve.setVisible(True)

    def hide_mic_curves(self):
        for curve in self.mic_curves:
            curve.setVisible(False)

    def update_routine_plot(self, new_row):
        if self.display_mode != "routine":
            return
        if len(new_row) == 36:
            if self.routine_show_all_average:
                avg_value = sum(new_row) / len(new_row)
                self.routine_show_data[0] = self.routine_show_data[0][1:] + [avg_value]
                x_data = np.arange(len(self.routine_show_data[0]))
                y_data = np.array(self.routine_show_data[0])
                self.routine_curves[0].setData(x_data, y_data)
                
                for i in range(1, 9):
                    self.routine_curves[i].setData([], [])
            else:
                row_start_idx = (self.routine_selected_row - 1) * 9
                row_data = new_row[row_start_idx : row_start_idx + 9]

                for i in range(9):
                    self.routine_show_data[i] = self.routine_show_data[i][1:] + [row_data[i]]
                    x_data = np.arange(len(self.routine_show_data[i]))
                    y_data = np.array(self.routine_show_data[i])
                    self.routine_curves[i].setData(x_data, y_data)

    def update_mic_plot(self, new_row):
        if self.display_mode != "mic":
            return
        expected_length = FRAMES_PER_PACKET * 5
        if len(new_row) == expected_length:
            for mic in range(5):
                mic_data = new_row[mic::5]
                self.mic_show_data[mic] = self.mic_show_data[mic][len(mic_data):] + mic_data
                x_data = np.arange(len(self.mic_show_data[mic]))
                y_data = np.array(self.mic_show_data[mic])
                self.mic_curves[mic].setData(x_data, y_data)

class MatrixVisualizer:
    def __init__(self, layout, interplotation=False, rotation_angle=0, flip_horizontal=False, flip_vertical=False,
                 zoom_factor=7, gaussian_sigma=0.5):
        self.layout = layout
        self.interplotation = interplotation
        self.rotation_angle = rotation_angle
        self.flip_horizontal = flip_horizontal
        self.flip_vertical = flip_vertical
        self.zoom_factor = zoom_factor
        self.gaussian_sigma = gaussian_sigma

        self.current_data = np.zeros((6, 6))

        self.image_item = pg.ImageItem()
        self.plot = pg.PlotItem(title="6x6传感器数据矩阵")
        self.plot.addItem(self.image_item)
        self.plot.setLabels(left='Y轴', bottom='X轴')
        self.layout.addItem(self.plot, 0, 0, 1, 2)

        self.color_map = pg.colormap.get('viridis')
        self.image_item.setColorMap(self.color_map)

        self.fps_label = pg.LabelItem(justify='left')
        self.runtime_label = pg.LabelItem(justify='right')
        self.layout.addItem(self.fps_label, 1, 0)
        self.layout.addItem(self.runtime_label, 1, 1)

        self.frame_count = 0
        self.start_time = QtCore.QTime.currentTime()
        self.runtime_start_time = None
        self.runtime_elapsed = 0

    def set_zoom_factor(self, factor):
        self.zoom_factor = factor
        if self.interplotation:
            self.refresh_image()

    def set_gaussian_sigma(self, sigma):
        self.gaussian_sigma = sigma
        if self.interplotation:
            self.refresh_image()

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

    def refresh_image(self):
        data = self.current_data.copy()

        if self.rotation_angle == 90:
            data = np.rot90(data, 1)
        elif self.rotation_angle == 180:
            data = np.rot90(data, 2)
        elif self.rotation_angle == 270:
            data = np.rot90(data, 3)

        if self.flip_horizontal:
            data = np.fliplr(data)
        if self.flip_vertical:
            data = np.flipud(data)

        if self.interplotation:
            data = zoom(data, (self.zoom_factor, self.zoom_factor), order=3)
            data = ndimage.gaussian_filter(data, sigma=self.gaussian_sigma)

        self.image_item.setImage(data, levels=(0.0, 1.0))

    def receive_data(self, new_row):
        if len(new_row) == 36:
            if self.runtime_start_time is None:
                self.runtime_start_time = QtCore.QTime.currentTime()
                self.runtime_elapsed = 0

            mapping = [
                [0,  1,  4,  5,  8,  9 ],
                [3,  2,  7,  6,  11, 10],
                [12, 13, 16, 17, 20, 21],
                [15, 14, 19, 18, 23, 22],
                [24, 25, 28, 29, 32, 33],
                [27, 26, 31, 30, 35, 34]
            ]
            
            self.current_data = np.zeros((6, 6))
            for i in range(6):
                for j in range(6):
                    self.current_data[i][j] = new_row[mapping[i][j]]
            
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

class SerialSelectionDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("选择串口或数据回放（6x6矩阵 + 5MIC）")
        self.setFixedSize(500, 400)
        self.selected_port = None
        self.save_data = SAVE_DATA_DEFAULT
        self.save_mic_data = False
        self.is_replay = False
        self.selected_csv = None

        self.setStyleSheet("""
            QDialog {
                background-color: #f5f5f5;
                border-radius: 12px;
            }
            QComboBox {
                background-color: white;
                border: 2px solid #dddddd;
                border-radius: 8px;
                padding: 6px 10px;
                margin: 8px 0;
                min-width: 250px;
                color: black;
            }
            QComboBox:hover {
                border-color: #88c9ff;
            }
            QComboBox QAbstractItemView {
                background-color: white;
                border-radius: 8px;
                border: 2px solid #dddddd;
                padding: 4px;
                selection-background-color: #88c9ff;
                selection-color: black;
            }
            QCheckBox {
                margin: 8px 0;
                spacing: 8px;
                color: black;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
                border-radius: 6px;
                border: 2px solid #dddddd;
                background-color: white;
            }
            QCheckBox::indicator:checked {
                background-color: #88c9ff;
                border-color: #88c9ff;
                image: url(:/qt-project.org/styles/commonstyle/images/check.png);
            }
            QPushButton {
                background-color: #88c9ff;
                border: none;
                border-radius: 8px;
                padding: 8px 0;
                margin: 4px 0;
                color: white;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #66b3ff;
            }
            QPushButton:disabled {
                background-color: #cccccc;
            }
            QPushButton#ReplayBtn {
                background-color: #66cc99;
            }
            QPushButton#ReplayBtn:hover {
                background-color: #55b388;
            }
        """)

        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(25, 20, 25, 20)
        self.setLayout(self.layout)

        self.port_combo = QComboBox()
        self.port_combo.setPlaceholderText("请选择可用串口（实时采集用）")
        self.layout.addWidget(self.port_combo)

        self.save_data_checkbox = QCheckBox("保存常规数据到CSV")
        self.save_data_checkbox.setChecked(SAVE_DATA_DEFAULT)
        self.layout.addWidget(self.save_data_checkbox)

        self.save_mic_data_checkbox = QCheckBox("保存5MIC数据到CSV")
        self.save_mic_data_checkbox.setChecked(False)
        self.layout.addWidget(self.save_mic_data_checkbox)

        self.button_layout = QVBoxLayout()
        self.confirm_btn = QPushButton("确认选择（实时采集）")
        self.cancel_btn = QPushButton("取消")
        self.replay_spacer = QSpacerItem(20, 10, QSizePolicy.Minimum, QSizePolicy.Fixed)
        self.replay_btn = QPushButton("数据回放（选择CSV文件）")
        self.replay_btn.setObjectName("ReplayBtn")

        self.button_layout.addWidget(self.confirm_btn)
        self.button_layout.addWidget(self.cancel_btn)
        self.button_layout.addItem(self.replay_spacer)
        self.button_layout.addWidget(self.replay_btn)
        
        self.layout.addLayout(self.button_layout)

        self.confirm_btn.clicked.connect(self.on_confirm)
        self.cancel_btn.clicked.connect(self.reject)
        self.replay_btn.clicked.connect(self.on_replay)

        self.load_available_ports()

    def load_available_ports(self):
        self.port_combo.clear()
        available_ports = list(list_ports.comports())
        
        if not available_ports:
            self.port_combo.addItem("未检测到可用串口", None)
            self.confirm_btn.setEnabled(False)
        else:
            if sys.platform.startswith('linux') or sys.platform.startswith('darwin'):
                ttyacm_ports = []
                ttyusb_ports = []
                other_ports = []
                
                for port in available_ports:
                    if "ttyACM" in port.device:
                        ttyacm_ports.append(port)
                    elif "ttyUSB" in port.device:
                        ttyusb_ports.append(port)
                    else:
                        other_ports.append(port)
                
                def extract_port_number(port):
                    import re
                    match = re.search(r'\d+$', port.device)
                    return int(match.group()) if match else 0
                    
                ttyacm_ports_sorted = sorted(ttyacm_ports, key=extract_port_number)
                ttyusb_ports_sorted = sorted(ttyusb_ports, key=extract_port_number)
                sorted_ports = ttyacm_ports_sorted + ttyusb_ports_sorted + other_ports
            else:
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

            for port in sorted_ports:
                port_info = f"{port.device} - {port.description}"
                self.port_combo.addItem(port_info, port.device)
            self.port_combo.setCurrentIndex(0)
            self.confirm_btn.setEnabled(True)

    def on_confirm(self):
        self.selected_port = self.port_combo.currentData()
        self.save_data = self.save_data_checkbox.isChecked()
        self.save_mic_data = self.save_mic_data_checkbox.isChecked()
        self.is_replay = False
        self.accept()

    def on_replay(self):
        csv_path, _ = QFileDialog.getOpenFileName(
            self, "选择CSV数据文件（6x6矩阵）", "", "CSV Files (*.csv);;All Files (*)"
        )
        if csv_path and os.path.exists(csv_path):
            self.selected_csv = csv_path
            self.is_replay = True
            self.accept()
        else:
            QMessageBox.warning(self, "警告", "请选择有效的CSV文件（需为36列数据）")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.worker = None
        self.selected_port = None
        self.save_data = SAVE_DATA_DEFAULT
        self.save_mic_data = False
        self.is_replay_mode = False

        if not self.select_serial_or_replay():
            sys.exit(0)

        self.init_main_ui()

    def select_serial_or_replay(self):
        dialog = SerialSelectionDialog()
        if dialog.exec_() == QDialog.Accepted:
            self.is_replay_mode = dialog.is_replay
            if self.is_replay_mode:
                self.selected_csv = dialog.selected_csv
                return True
            else:
                self.selected_port = dialog.selected_port
                self.save_data = dialog.save_data
                self.save_mic_data = dialog.save_mic_data
                return self.selected_port is not None
        else:
            return False

    def init_main_ui(self):
        self.setWindowTitle("6x6传感器 + 5MIC 双视图可视化")
        self.resize(1500, 800)
        self.showMaximized()

        self.central_widget = QSplitter()
        self.setCentralWidget(self.central_widget)

        # 左侧：图像显示 + 参数控制面板
        self.image_container = QWidget()
        self.image_container.setStyleSheet("background-color: black; border: none;")
        self.image_container_layout = QVBoxLayout(self.image_container)
        self.image_container_layout.setContentsMargins(0, 0, 0, 0)
        
        self.image_layout = pg.GraphicsLayoutWidget()
        self.image_visualizer = MatrixVisualizer(
            self.image_layout, 
            interplotation=False, 
            rotation_angle=[0, 90, 180, 270][DEFAULT_ROTATION_INDEX], 
            flip_horizontal=DEFAULT_FLIP_HORIZONTAL, 
            flip_vertical=DEFAULT_FLIP_VERTICAL,
            zoom_factor=DEFAULT_ZOOM_FACTOR,
            gaussian_sigma=DEFAULT_GAUSSIAN_SIGMA
        )
        self.image_container_layout.addWidget(self.image_layout)

        self.image_control_panel = QWidget()
        self.image_control_panel.setStyleSheet("background-color: black; border: none;")
        self.image_control_layout = QHBoxLayout(self.image_control_panel)
        self.image_control_layout.setContentsMargins(10, 5, 10, 5)
        self.image_control_layout.setSpacing(15)

        self.interp_checkbox = QCheckBox("插值显示")
        self.interp_checkbox.setStyleSheet("color: white; background-color: #222222; border: 1px solid #555555; padding: 5px;")
        self.interp_checkbox.setChecked(self.image_visualizer.interplotation)
        self.interp_checkbox.stateChanged.connect(
            lambda state: self.image_visualizer.set_interpolation(state == QtCore.Qt.Checked)
        )
        self.image_control_layout.addWidget(self.interp_checkbox)

        self.zoom_label = QLabel("插值密度:")
        self.zoom_label.setStyleSheet("color: white;")
        self.zoom_spin = QSpinBox()
        self.zoom_spin.setRange(1, 15)
        self.zoom_spin.setValue(self.image_visualizer.zoom_factor)
        self.zoom_spin.setStyleSheet("QSpinBox { color: white; background-color: #222222; border: 1px solid #555555; padding: 2px; }")
        self.zoom_spin.valueChanged.connect(self.image_visualizer.set_zoom_factor)
        self.image_control_layout.addWidget(self.zoom_label)
        self.image_control_layout.addWidget(self.zoom_spin)

        self.sigma_label = QLabel("高斯核:")
        self.sigma_label.setStyleSheet("color: white;")
        self.sigma_spin = QDoubleSpinBox()
        self.sigma_spin.setRange(0.0, 10.0)
        self.sigma_spin.setSingleStep(0.5)
        self.sigma_spin.setValue(self.image_visualizer.gaussian_sigma)
        self.sigma_spin.setStyleSheet("QDoubleSpinBox { color: white; background-color: #222222; border: 1px solid #555555; padding: 2px; }")
        self.sigma_spin.valueChanged.connect(self.image_visualizer.set_gaussian_sigma)
        self.image_control_layout.addWidget(self.sigma_label)
        self.image_control_layout.addWidget(self.sigma_spin)

        self.rotation_label = QLabel("旋转角度：")
        self.rotation_label.setStyleSheet("color: white;")
        self.rotation_combo = QComboBox()
        self.rotation_combo.addItems(["0度", "90度", "180度", "270度"])
        self.rotation_combo.setStyleSheet("QComboBox { color: white; background-color: #222222; border: 1px solid #555555; padding: 2px 5px; }")
        rotation_map = {0:0, 90:1, 180:2, 270:3}
        self.rotation_combo.setCurrentIndex(rotation_map[self.image_visualizer.rotation_angle])
        self.rotation_combo.currentIndexChanged.connect(
            lambda idx: self.image_visualizer.set_rotation_angle([0,90,180,270][idx])
        )
        self.image_control_layout.addWidget(self.rotation_label)
        self.image_control_layout.addWidget(self.rotation_combo)

        self.flip_h_checkbox = QCheckBox("垂直翻转")
        self.flip_h_checkbox.setStyleSheet("color: white; background-color: #222222; border: 1px solid #555555; padding: 5px;")
        self.flip_h_checkbox.setChecked(self.image_visualizer.flip_horizontal)
        self.flip_h_checkbox.stateChanged.connect(
            lambda state: self.image_visualizer.set_flip_horizontal(state == QtCore.Qt.Checked)
        )
        self.image_control_layout.addWidget(self.flip_h_checkbox)

        self.flip_v_checkbox = QCheckBox("水平翻转")
        self.flip_v_checkbox.setStyleSheet("color: white; background-color: #222222; border: 1px solid #555555; padding: 5px;")
        self.flip_v_checkbox.setChecked(self.image_visualizer.flip_vertical)
        self.flip_v_checkbox.stateChanged.connect(
            lambda state: self.image_visualizer.set_flip_vertical(state == QtCore.Qt.Checked)
        )
        self.image_control_layout.addWidget(self.flip_v_checkbox)

        self.norm_high_label = QLabel("最大值:")
        self.norm_high_label.setStyleSheet("color: white;")
        self.norm_high_input = QSpinBox()
        self.norm_high_input.setRange(1, 5000)
        self.norm_high_input.setSingleStep(100)
        self.norm_high_input.setValue(NORMALIZATION_HIGH_VALUE)
        self.norm_high_input.setStyleSheet("QSpinBox { color: white; background-color: #222222; border: 1px solid #555555; padding: 2px; }")
        self.norm_high_input.valueChanged.connect(self.set_normalization_high)
        self.image_control_layout.addWidget(self.norm_high_label)
        self.image_control_layout.addWidget(self.norm_high_input)

        self.image_control_layout.addSpacerItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        self.image_container_layout.addWidget(self.image_control_panel)

        self.central_widget.addWidget(self.image_container)

        # 右侧：波形显示
        self.waveform_widget = QWidget()
        self.waveform_visualizer = CombinedWaveformVisualizer(self.waveform_widget)
        self.waveform_visualizer.set_reset_callback(self.reset_initial_value)
        self.central_widget.addWidget(self.waveform_widget)

        # 创建并启动工作线程
        if self.is_replay_mode:
            self.worker = CombinedSerialWorker(
                csv_path=self.selected_csv,
                save_data=False,
                save_mic_data=False,
                normalization_high=self.norm_high_input.value()
            )
            self.worker.replay_finished.connect(self.close)
            self.worker.routine_data_ready.connect(self.image_visualizer.receive_data, Qt.QueuedConnection)
            self.worker.routine_waveform_ready.connect(self.waveform_visualizer.update_routine_plot, Qt.QueuedConnection)
        else:
            self.worker = CombinedSerialWorker(
                port=self.selected_port,
                save_data=self.save_data,
                save_mic_data=self.save_mic_data,
                normalization_high=self.norm_high_input.value()
            )
            self.worker.routine_data_ready.connect(self.image_visualizer.receive_data)
            self.worker.routine_waveform_ready.connect(self.waveform_visualizer.update_routine_plot)
            self.worker.mic_waveform_ready.connect(self.waveform_visualizer.update_mic_plot)
        
        self.worker.error_signal.connect(self.show_serial_error)
        self.worker.fps_ready.connect(self.update_fps_display)
        self.worker.start()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.image_visualizer.update_fps)
        self.timer.start(200)

    def set_normalization_high(self, value):
        if self.worker:
            self.worker.normalization_high = value

    def reset_initial_value(self):
        if self.worker:
            self.worker.reset_initialization()

    def update_fps_display(self, fps):
        self.image_visualizer.fps_label.setText(f"FPS: {fps:.1f}")

    def show_serial_error(self, error_msg):
        QMessageBox.critical(self, "错误", error_msg)
        self.close()

    def closeEvent(self, event):
        self.hide()
        if self.worker:
            self.worker.stop()
            if not self.worker.wait(2000):
                self.worker.terminate()
                print("线程超时，已强制终止")
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec_())