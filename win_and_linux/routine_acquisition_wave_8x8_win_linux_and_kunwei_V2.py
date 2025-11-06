import sys
import numpy as np
from scipy import signal  # 新增：导入信号处理库
from PyQt5.QtWidgets import (QApplication, QMainWindow, QSplitter, QDialog, 
                             QVBoxLayout, QHBoxLayout, QComboBox, QPushButton, QMessageBox, QWidget, QLabel, QCheckBox,
                             QSpacerItem, QSizePolicy, QDoubleSpinBox, QSpinBox, QFileDialog)
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import serial
from serial.tools import list_ports  # 跨平台串口检测
from queue import Queue
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from scipy.ndimage import zoom, gaussian_filter
import os
import threading
from datetime import datetime
import csv
import struct
import time
# 不准改我的注释！！！不准删！！！

# 数据保存选项的宏定义
SAVE_DATA_DEFAULT = False  # 默认保存数据

# 滤波参数默认值（新增）
DEFAULT_FS = 100.0  # 默认采样率100Hz
DEFAULT_LOW_CUTOFF = 40.0  # 默认低通截止频率10Hz
DEFAULT_FILTER_ORDER = 2  # 默认滤波器阶数



# 新增：复用代码1的默认参数宏定义
NORMALIZATION_HIGH_VALUE = 700  # 代码2原有默认值
DEFAULT_ZOOM_FACTOR = 8
DEFAULT_GAUSSIAN_SIGMA = 0.0
DEFAULT_ROTATION_INDEX = 1  # 代码2原有默认90度
DEFAULT_FLIP_HORIZONTAL = False  # 代码2原有默认
DEFAULT_FLIP_VERTICAL = False    # 代码2原有默认
# 新增：滤波处理类（来自代码1）
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
        return self.zi_low  # 关键修复：返回zi_low，用于初始化
    
    def apply_low_pass(self, data_point):
        filtered_point, self.zi_low = signal.lfilter(self.b_low, self.a_low, 
                                                   [data_point], zi=self.zi_low)
        return filtered_point[0]





class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 用于图像更新
    waveform_ready = pyqtSignal(list)  # 用于波形更新
    error_signal = pyqtSignal(str)  # 串口错误信号（跨平台错误提示）

    def __init__(self, port, save_data=True, normalization_low=0, normalization_high=700,
                 fs=DEFAULT_FS, low_cutoff=DEFAULT_LOW_CUTOFF, filter_order=DEFAULT_FILTER_ORDER):
        super().__init__()
        self.port = port  # 外部传入选择的串口
        self.baudrate = 460800
        self.ser = None
        self.running = False
        self.buffer = bytearray()
        self.FRAME_HEADER = b'\x55\xAA\xBB\xCC'
        self.FRAME_TAIL = b'\xAA\x55\x66\x77'
        
        # 添加初始化帧相关变量
        self.init_frames = []  # 用于存储初始化帧
        self.init_frame_count = 0  # 初始化帧计数器
        self.max_init_frames = 20  # 最大初始化帧数
        self.matrix_init = None  # 初始化矩阵
        self.dead_value = 0.0  # 死点值
        # 用于存储最近帧数据的缓冲区
        self.recent_frames = []  # 存储最近的帧数据用于校准
        self.max_recent_frames = 50  # 最多存储50帧

        self.normalization_low = normalization_low
        self.normalization_high = normalization_high

        # 新增：滤波相关初始化
        self.fs = fs
        self.low_cutoff = low_cutoff
        self.filter_order = filter_order
        self.filters = [FilterHandler(fs, low_cutoff, filter_order) for _ in range(64)]  # 64个点各一个滤波器
        self.filters_initialized = False  # 滤波器是否已初始化

        self.save_dir = "Routine_acq_raw_data_8x8"
        self.data_queue = Queue()
        self.writer_thread = None
        self.is_saving = False
        self.data_buffer = []
        self.save_data = save_data
        if self.save_data:
            self.init_raw_data_saving()

    # 新增：更新滤波参数
    def update_filter_params(self, fs, low_cutoff, order):
        self.fs = fs
        self.low_cutoff = low_cutoff
        self.filter_order = order
        # 重新初始化所有滤波器
        self.filters = [FilterHandler(fs, low_cutoff, order) for _ in range(64)]
        self.filters_initialized = False  # 需要重新初始化状态

    def init_raw_data_saving(self):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.save_dir, f"self_data_8x8_{timestamp}.csv")
        self.is_saving = True
        self.writer_thread = threading.Thread(target=self.write_raw_data_to_file, daemon=True)
        self.writer_thread.start()
        
    def write_raw_data_to_file(self):
        # 按组顺序保存数据到CSV的逻辑
        with open(self.csv_path, 'w', newline='', buffering=65536) as csvfile:
            writer = csv.writer(csvfile)
            
            # 创建按组排列的列标题
            headers = ['Timestamp(ns)']  # 添加时间戳列标题
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
                    # 获取ns级时间戳
                    ns_timestamp = time.perf_counter_ns()
                    # 将数据重塑为(10, 64)
                    flattened_data = mapped_frames.reshape(10, 64)
                    
                    # 使用向量化操作重新排列数据
                    grouped_data = flattened_data[:, index_mapping]
                    
                    # 为每行数据添加时间戳
                    timestamped_data = []
                    for i in range(grouped_data.shape[0]):
                        timestamped_row = [ns_timestamp + i] + grouped_data[i].tolist()
                        timestamped_data.append(timestamped_row)
                    
                    # 写入CSV
                    writer.writerows(timestamped_data)
                else:
                    self.msleep(5)

    def stop(self):
        self.running = False
        if self.save_data:
            self.is_saving = False
            if self.writer_thread and self.writer_thread.is_alive():
                self.writer_thread.join()
        print('保存完毕')

    def reset_initialization(self):
        """重置初始化相关变量，以便重新计算初始值"""
        if len(self.recent_frames) > 0:
            self.matrix_init = np.mean(self.recent_frames, axis=0)
            print("已校准零位")
            # 重置滤波器状态（使用新的初始值）
            self.filters_initialized = False

    def run(self):
        try:
            # 跨平台串口连接
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            while self.running:
                try:
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
                                    
                                    # 更新的映射表
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
                                    src_coords = [(x, y) for row in mapping for x, y in row]
                                    src_x_coords = [coord[0] for coord in src_coords]
                                    src_y_coords = [coord[1] for coord in src_coords]
                                    
                                    # 对所有10帧数据进行映射处理
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

                                    # 新增：应用低通滤波
                                    if not self.filters_initialized:
                                        # 初始化滤波器状态
                                        for i in range(64):
                                            self.filters[i].initialize_states(average[i])
                                        self.filters_initialized = True
                                        filtered_average = average.copy()
                                    else:
                                        # 对每个点应用滤波
                                        filtered_average = np.array([
                                            self.filters[i].apply_low_pass(average[i]) 
                                            for i in range(64)
                                        ])

                                    # 将滤波后的数据添加到recent_frames缓冲区用于校准
                                    self.recent_frames.append(filtered_average.copy())
                                    # 如果recent_frames超过最大长度，则移除最旧的帧
                                    if len(self.recent_frames) > self.max_recent_frames:
                                        self.recent_frames.pop(0)

                                    # 收集前max_init_frames帧用于初始化
                                    if self.init_frame_count < self.max_init_frames:
                                        self.init_frames.append(filtered_average.copy())
                                        self.init_frame_count += 1
                                        
                                        # 当收集到足够的帧时，计算平均值作为matrix_init
                                        if self.init_frame_count == self.max_init_frames:
                                            self.matrix_init = np.mean(self.init_frames, axis=0)
                                            print(f"初始化完成，使用{self.max_init_frames}帧计算初始值")
                                            
                                    # 如果已经完成初始化，进行数据处理
                                    elif self.matrix_init is not None:
                                        # 使用matrix_init进行归零处理
                                        zeroed_results = np.array(filtered_average) - self.matrix_init
                                        deaded_results = zeroed_results - self.dead_value
                                        clipped_result = np.clip(deaded_results, self.normalization_low, self.normalization_high)
                                        normalized_result = (clipped_result - self.normalization_low) / (self.normalization_high - self.normalization_low)
                                        
                                        self.waveform_ready.emit(filtered_average.tolist())  # 发送滤波后的波形数据
                                        self.data_ready.emit(normalized_result.tolist())  # 发送归一化后的图像数据

                                self.buffer = self.buffer[end_index + len(self.FRAME_TAIL):]
                                start_index = self.buffer.find(self.FRAME_HEADER)
                            else:
                                break
                except Exception as e:
                    print(f"读取传感器数据时出错: {str(e)}")
                    self.msleep(100)
        except Exception as e:
            error_msg = f"串口通信异常: {str(e)}\nUbuntu用户请检查是否加入dialout组"
            self.error_signal.emit(error_msg)
        finally:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                    print('串口关闭')
                except OSError as e:
                    print(f"关闭串口时出错: {str(e)}")
            self.running = False


class DataReplayWorker(SerialWorker):
    replay_finished = pyqtSignal()  # 回放完成信号

    def __init__(self, csv_path, save_data=False, normalization_low=0, normalization_high=700,
                 fs=DEFAULT_FS, low_cutoff=DEFAULT_LOW_CUTOFF, filter_order=DEFAULT_FILTER_ORDER):
        super().__init__(port=None, save_data=save_data,
                        normalization_low=normalization_low,
                        normalization_high=normalization_high,
                        fs=fs, low_cutoff=low_cutoff, filter_order=filter_order)
        self.csv_path = csv_path
        self.frame_index = 0
        self.total_frames = 0
        self.raw_data = None
        self.play_timer = None
        self.is_stopped = False

        # 复用代码2的group_mapping
        self.group_mapping = []
        for i in range(16):
            row = 3 - i // 4
            col = 3 - i % 4
            idx1 = (row * 2 + 1) * 8 + (col + 1) * 2 - 1
            idx2 = (row * 2 + 1) * 8 + (col + 1) * 2 - 2
            idx3 = (row * 2 + 0) * 8 + (col + 1) * 2 - 1
            idx4 = (row * 2 + 0) * 8 + (col + 1) * 2 - 2
            self.group_mapping.append([idx1, idx2, idx3, idx4])
        self.flat_mapping = [idx for group in self.group_mapping for idx in group]

    # 重写：更新滤波参数（与父类保持一致）
    def update_filter_params(self, fs, low_cutoff, order):
        super().update_filter_params(fs, low_cutoff, order)

    def run(self):
        try:
            # 读取CSV（跳过时间戳列，按组排列数据）
            self.raw_data = np.loadtxt(self.csv_path, delimiter=',', skiprows=1)
            if self.raw_data.ndim != 2 or self.raw_data.shape[1] != 65:  # 时间戳+64数据
                raise ValueError(f"CSV格式错误，应为N行65列，实际为{self.raw_data.shape}")
            self.raw_data = self.raw_data[:, 1:]  # 去掉时间戳列
            self.total_frames = self.raw_data.shape[0]
            print(f"成功加载CSV文件，共{self.total_frames}帧数据")

            # 初始化零位校准
            if self.matrix_init is None:
                init_count = min(self.max_init_frames, self.total_frames)
                init_data = self.raw_data[:init_count]
                # 恢复为8x8顺序后计算初始值
                init_data_8x8 = self.restore_8x8_order(init_data)
                self.matrix_init = np.mean(init_data_8x8, axis=0) + self.dead_value
                print(f"回放初始化完成，使用前{init_count}帧计算初始值")

            # 启动定时器播放
            self.play_timer = QTimer()
            self.play_timer.setInterval(10)
            self.play_timer.timeout.connect(self.process_frame)
            self.play_timer.start()
            self.exec_()

            # 清理
            if self.play_timer and self.play_timer.isActive():
                self.play_timer.stop()
            self.replay_finished.emit()
            print("数据回放完成")

        except Exception as e:
            self.error_signal.emit(f"回放异常: {str(e)}")

    def restore_8x8_order(self, grouped_data):
        """将按组排列的数据恢复为8x8索引顺序"""
        inv_mapping = np.zeros(64, dtype=int)
        for idx_8x8, idx_grouped in enumerate(self.flat_mapping):
            inv_mapping[idx_grouped] = idx_8x8
        return grouped_data[:, inv_mapping]

    def process_frame(self):
        if self.is_stopped or self.frame_index >= self.total_frames:
            self.quit()
            return

        # 按10帧一组读取
        remaining = self.total_frames - self.frame_index
        take_count = min(10, remaining)
        current_frames_grouped = self.raw_data[self.frame_index:self.frame_index + take_count]
        self.frame_index += take_count

        # 恢复为8x8顺序
        current_frames_8x8 = self.restore_8x8_order(current_frames_grouped)
        average = np.mean(current_frames_8x8, axis=0)

        # 新增：应用滤波
        if not self.filters_initialized:
            # 初始化滤波器状态
            for i in range(64):
                self.filters[i].initialize_states(average[i])
            self.filters_initialized = True
            filtered_average = average.copy()
        else:
            # 对每个点应用滤波
            filtered_average = np.array([
                self.filters[i].apply_low_pass(average[i]) 
                for i in range(64)
            ])

        # 校准缓冲区更新
        self.recent_frames.append(filtered_average.copy())
        if len(self.recent_frames) > self.max_recent_frames:
            self.recent_frames.pop(0)

        # 数据处理
        zeroed_results = np.array(filtered_average) - self.matrix_init
        deaded_results = zeroed_results - self.dead_value
        clipped_result = np.clip(deaded_results, self.normalization_low, self.normalization_high)
        normalized_result = (clipped_result - self.normalization_low) / (self.normalization_high - self.normalization_low)

        # 发送信号
        self.waveform_ready.emit(filtered_average.tolist())
        self.data_ready.emit(normalized_result.tolist())

    def stop(self):
        self.is_stopped = True
        if self.play_timer and self.play_timer.isActive():
            self.play_timer.stop()
        self.quit()


class KunweiSerialWorker(QThread):
    kunwei_data_ready = pyqtSignal(list)  # 发送坤维传感器数据
    error_signal = pyqtSignal(str)  # 串口错误信号

    def __init__(self, port, save_data=True,
                 fs=DEFAULT_FS, low_cutoff=DEFAULT_LOW_CUTOFF, filter_order=DEFAULT_FILTER_ORDER):
        super().__init__()
        self.port = port
        self.baudrate = 460800
        self.ser = None
        self.running = False
        self.buffer = b''  # 数据缓冲区
        # 帧头和帧尾定义
        self.FRAME_HEADER = b'\x48\xAA'
        self.FRAME_TAIL = b'\x0D\x0A'
        # 启动前需发送的十六进制数据（48 AA 0D 0A）
        self.INIT_SEND_DATA = b'\x48\xAA\x0D\x0A'
        
        # 新增：坤维传感器滤波（6个通道）
        self.fs = fs
        self.low_cutoff = low_cutoff
        self.filter_order = filter_order
        self.kunwei_filters = [FilterHandler(fs, low_cutoff, filter_order) for _ in range(6)]  # 6个通道各一个滤波器
        self.kunwei_filters_initialized = False  # 滤波器是否已初始化
        
        # CSV数据保存相关
        self.save_data = save_data
        self.csv_filename = f"Routine_acq_raw_data_8x8/kunwei_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = None
        self.csv_writer = None
    
    # 新增：更新坤维传感器滤波参数
    def update_filter_params(self, fs, low_cutoff, order):
        self.fs = fs
        self.low_cutoff = low_cutoff
        self.filter_order = order
        self.kunwei_filters = [FilterHandler(fs, low_cutoff, order) for _ in range(6)]
        self.kunwei_filters_initialized = False

    def stop(self):
        """停止数据读取"""
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print('坤维传感器串口已关闭')
            except OSError as e:
                print(f"关闭坤维传感器串口时出错: {str(e)}")

    def parse_frame(self, frame):
        """解析单帧数据为浮点数"""
        try:
            # 提取有效载荷（去掉帧头和帧尾）
            payload = frame[len(self.FRAME_HEADER):-len(self.FRAME_TAIL)]
            
            # 检查有效载荷长度是否符合要求（6个浮点数，每个4字节，共24字节）
            if len(payload) != 24:
                return None, f"有效载荷长度错误，应为24字节，实际为{len(payload)}字节"
            
            components = []
            for i in range(6):  # 6个分量
                # 提取4字节
                start = i * 4
                comp_bytes = payload[start:start+4]
                # 调整字节顺序（低字节在前 → 高字节在前）
                reversed_bytes = comp_bytes[::-1]
                # 解析为浮点数
                comp_float = struct.unpack('>f', reversed_bytes)[0]
                components.append(comp_float)
                
            return components, "解析成功"
            
        except Exception as e:
            return None, f"解析错误: {str(e)}"
    
    def init_kunwei_data_saving(self):
        """初始化坤维传感器数据保存"""
        if self.save_data:
            self.csv_file = open(self.csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # 表头顺序：时间戳(ns)、Fx、Fy、Fz、Mx、My、Mz
            self.csv_writer.writerow(['Timestamp(ns)', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])
            print(f"坤维传感器数据将保存到当前目录下的 {self.csv_filename} 文件")
            
    def run(self):
        try:
            # 打开串口
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"坤维传感器串口 {self.port} 已打开，波特率: {self.baudrate}，等待数据...")
            # 启动接收前发送指定十六进制数据（48 AA 0D 0A）
            self.ser.write(self.INIT_SEND_DATA)
            print(f"已发送启动初始化数据: {self.INIT_SEND_DATA.hex().upper()}（十六进制）")
            
            # 初始化CSV文件（如果需要保存）
            self.init_kunwei_data_saving()
            
            self.running = True
            while self.running:
                try:
                    if self.ser.in_waiting > 0:
                        # 读取所有可用数据
                        data = self.ser.read(self.ser.in_waiting)
                        self.buffer += data
                        
                        # 在缓冲区中查找帧头
                        start_index = self.buffer.find(self.FRAME_HEADER)
                        while start_index != -1:
                            # 查找帧尾
                            end_index = self.buffer.find(self.FRAME_TAIL, start_index + len(self.FRAME_HEADER))
                            
                            if end_index != -1:
                                # 提取完整帧
                                frame = self.buffer[start_index:end_index + len(self.FRAME_TAIL)]
                                
                                # 解析帧数据
                                components, msg = self.parse_frame(frame)
                                if components:
                                    # 新增：应用滤波
                                    if not self.kunwei_filters_initialized:
                                        # 初始化滤波器状态
                                        for i in range(6):
                                            self.kunwei_filters[i].initialize_states(components[i])
                                        self.kunwei_filters_initialized = True
                                        filtered_components = components.copy()
                                    else:
                                        # 对每个通道应用滤波
                                        filtered_components = [
                                            self.kunwei_filters[i].apply_low_pass(components[i])
                                            for i in range(6)
                                        ]
                                    
                                    # 获取ns级时间戳
                                    ns_timestamp = time.perf_counter_ns()
                                    # 写入CSV文件
                                    if self.save_data and self.csv_writer:
                                        self.csv_writer.writerow([ns_timestamp] + components)  # 保存原始数据
                                    
                                    # 发送滤波后的数据到主线程
                                    self.kunwei_data_ready.emit(filtered_components)
                                else:
                                    pass
                                
                                # 移除已处理的帧
                                self.buffer = self.buffer[end_index + len(self.FRAME_TAIL):]
                                start_index = self.buffer.find(self.FRAME_HEADER)
                            else:
                                # 未找到帧尾，保留剩余缓冲区内容
                                self.buffer = self.buffer[start_index:]
                                break
                except Exception as e:
                    print(f"读取坤维传感器数据时出错: {str(e)}")
        except Exception as e:
            error_msg = f"坤维传感器串口通信异常: {str(e)}\nUbuntu用户请检查是否加入dialout组"
            self.error_signal.emit(error_msg)
        finally:
            # 关闭CSV文件
            if self.csv_file:
                try:
                    self.csv_file.close()
                    print(f"坤维传感器CSV文件 {self.csv_filename} 已成功关闭，数据已保存")
                except Exception as e:
                    print(f"关闭坤维传感器CSV文件时出错: {str(e)}")
            self.running = False


# ---------------------- 坤维传感器波形可视化类 ----------------------
class KunweiWaveformVisualizer:
    def __init__(self, parent_widget):
        self.parent_widget = parent_widget
        # 保持黑色背景+黑色边框的样式一致性
        self.parent_widget.setStyleSheet("""
            QWidget {
                background-color: black;
                border: 2px solid black;
            }
        """)
        
        # 垂直布局：仅包含绘图区
        self.main_layout = QVBoxLayout()
        self.parent_widget.setLayout(self.main_layout)
        
        # 1. 坤维波形绘图区配置
        self.plot_widget = pg.PlotWidget()
        self.plot = self.plot_widget.getPlotItem()
        # 根据默认模式设置标题
        self.plot.setTitle("坤维传感器 - Fx/Fy/Fz/Mx/My/Mz (滤波后)")  # 新增：标注滤波后
        self.plot.setLabels(left='力/力矩值', bottom='帧编号')
        self.plot.showGrid(x=True, y=True)
        self.plot.setYRange(-20, 20)  # 可根据实际硬件调整
        self.main_layout.addWidget(self.plot_widget)

        # 2. 为6个通道创建曲线
        self.curves = []
        colors = ['r', 'g', 'c', 'y', 'b', 'm']  # 6通道颜色区分
        channel_names = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
        for i in range(6):
            curve = self.plot.plot(pen=colors[i], name=channel_names[i])
            self.curves.append(curve)
        
        # 批量更新配置
        self.frame_buffer = []  # 临时存储原始帧
        self.update_batch_size = 10  # 每10帧处理一次
        self.use_average = True  # True=10帧→1平均值；False=10帧真实数据

        # 3. 数据缓冲区配置
        self.buffer_time_100ms = 40  # 总显示帧数（固定）
        if self.use_average:  # 10帧→1平均值
            self.buffer_length = self.buffer_time_100ms*10  # 总显示帧数（固定）
        else:
            self.buffer_length = self.buffer_time_100ms*100  # 总显示帧数（固定）
        self.show_data = [[0.0] * self.buffer_length for _ in range(6)]  # 6通道数据

    def update_plot(self, new_data):
        """更新逻辑：通过use_average变量控制两种模式"""
        if len(new_data) != 6:
            return  # 数据格式错误直接返回
        
        # 1. 积累原始帧数据
        self.frame_buffer.append(new_data)
        
        # 2. 当积累满10帧时处理
        if len(self.frame_buffer) == self.update_batch_size:
            frame_array = np.array(self.frame_buffer, dtype=np.float32)  # 形状：(10,6)
            
            # 3. 根据模式处理数据
            if self.use_average:
                # 模式1：10帧→1个平均值，更新1帧（无阶梯）
                avg_values = np.mean(frame_array, axis=0)  # 形状：(6,)
                # 每个通道移除1帧旧数据，添加1帧平均值
                for i in range(6):
                    self.show_data[i] = self.show_data[i][1:] + [avg_values[i]]
            else:
                # 模式2：直接使用10帧真实数据，更新10帧（保留细节）
                batch_data = frame_array.T  # 转置为(6,10)
                # 每个通道移除10帧旧数据，添加10帧新数据
                for i in range(6):
                    self.show_data[i] = self.show_data[i][self.update_batch_size:] + batch_data[i].tolist()
            
            # 4. 刷新所有通道曲线
            for i in range(6):
                x_data = np.arange(len(self.show_data[i]))
                y_data = np.array(self.show_data[i])
                self.curves[i].setData(x_data, y_data)
            
            # 5. 清空缓冲区，准备下一批数据
            self.frame_buffer = []


# ---------------------- 传感器点位波形可视化类 ----------------------
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
        self.plot.setTitle(f"组 {self.selected_group+1} (Row {(self.selected_group//4)+1} Col {(self.selected_group%4)+1}) (滤波后)")  # 新增：标注滤波后
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
        self.plot.setTitle(f"组 {self.selected_group+1} (Row {row} Col {col}) (滤波后)")  # 新增：标注滤波后

    def update_plot(self, new_row):
        if len(new_row) == 64:
            # 定义4x4组的映射关系，每组4个点
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
            self.plot.setTitle(f"组 {self.selected_group+1} (Row {row} Col {col}) (滤波后)")


class MatrixVisualizer:
    def __init__(self, layout, interplotation=False, rotation_angle=0, flip_horizontal=False, flip_vertical=False,
                 zoom_factor=DEFAULT_ZOOM_FACTOR, gaussian_sigma=DEFAULT_GAUSSIAN_SIGMA):
        self.layout = layout
        self.interplotation = interplotation
        self.rotation_angle = rotation_angle
        self.flip_horizontal = flip_horizontal
        self.flip_vertical = flip_vertical
        self.zoom_factor = zoom_factor  # 插值密度
        self.gaussian_sigma = gaussian_sigma  # 高斯核
        self.current_data = np.zeros((8, 8))

        self.image_item = pg.ImageItem()
        self.plot = pg.PlotItem(title="8x8传感器数据矩阵 (滤波后)")  # 新增：标注滤波后
        self.plot.addItem(self.image_item)
        self.plot.setLabels(left='Y轴', bottom='X轴')
        self.layout.addItem(self.plot, 0, 0, 1, 2)

        self.color_map = pg.colormap.get('viridis')
        self.image_item.setColorMap(self.color_map)

        # FPS+运行时间标签
        self.fps_label = pg.LabelItem(justify='left')
        self.runtime_label = pg.LabelItem(justify='right')
        self.layout.addItem(self.fps_label, 1, 0)
        self.layout.addItem(self.runtime_label, 1, 1)

        self.frame_count = 0
        self.start_time = QtCore.QTime.currentTime()
        self.runtime_start_time = None
        self.runtime_elapsed = 0

    # 新增参数setter方法
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
        """实时刷新图像（应用所有参数）"""
        data = self.current_data.copy()

        # 旋转
        if self.rotation_angle == 90:
            data = np.rot90(data, 1)
        elif self.rotation_angle == 180:
            data = np.rot90(data, 2)
        elif self.rotation_angle == 270:
            data = np.rot90(data, 3)

        # 翻转
        if self.flip_horizontal:
            data = np.fliplr(data)
        if self.flip_vertical:
            data = np.flipud(data)

        # 插值+高斯滤波
        if self.interplotation:
            data = zoom(data, (self.zoom_factor, self.zoom_factor), order=3)
            data = gaussian_filter(data, sigma=self.gaussian_sigma)

        self.image_item.setImage(data, levels=(0.0, 1.0))

    def receive_data(self, new_row):
        if len(new_row) == 64:
            if self.runtime_start_time is None:
                self.runtime_start_time = QtCore.QTime.currentTime()
                self.runtime_elapsed = 0

            self.current_data = np.array(new_row).reshape(8, 8)
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
    """跨平台串口选择对话框（融合实时采集/数据回放）"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("选择串口或数据回放")
        self.setFixedSize(500, 450)
        self.selected_sensor_port = None
        self.selected_kunwei_port = None
        self.save_data = SAVE_DATA_DEFAULT
        self.is_replay = False
        self.selected_csv = None

        # 复用代码1的美观样式
        self.setStyleSheet("""
            QDialog { background-color: #f5f5f5; border-radius: 12px; }
            QLabel { color: black; font-size: 14px; margin: 8px 0 4px 0; }
            QComboBox {
                background-color: white; border: 2px solid #dddddd; border-radius: 8px;
                padding: 6px 10px; margin: 4px 0 12px 0; min-width: 250px; color: black;
            }
            QComboBox:hover { border-color: #88c9ff; }
            QComboBox QAbstractItemView {
                background-color: white; border-radius: 8px; border: 2px solid #dddddd;
                padding: 4px; selection-background-color: #88c9ff; selection-color: black;
            }
            QCheckBox { margin: 8px 0; spacing: 8px; color: black; }
            QCheckBox::indicator {
                width: 18px; height: 18px; border-radius: 6px;
                border: 2px solid #dddddd; background-color: white;
            }
            QCheckBox::indicator:checked {
                background-color: #88c9ff; border-color: #88c9ff;
                image: url(:/qt-project.org/styles/commonstyle/images/check.png);
            }
            QPushButton {
                background-color: #88c9ff; border: none; border-radius: 8px;
                padding: 8px 0; margin: 4px 0; color: white; font-weight: bold;
            }
            QPushButton:hover { background-color: #66b3ff; }
            QPushButton:disabled { background-color: #cccccc; }
            QPushButton#ReplayBtn { background-color: #66cc99; }
            QPushButton#ReplayBtn:hover { background-color: #55b388; }
        """)

        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(25, 20, 25, 20)
        self.setLayout(self.layout)

        # 传感器串口选择（保留代码2双串口需求）
        self.sensor_label = QLabel("选择传感器串口（实时采集用）:")
        self.sensor_port_combo = QComboBox()
        self.sensor_port_combo.setPlaceholderText("请选择传感器串口")
        self.layout.addWidget(self.sensor_label)
        self.layout.addWidget(self.sensor_port_combo)

        # 坤维传感器串口选择
        self.kunwei_label = QLabel("选择坤维传感器串口（实时采集用）:")
        self.kunwei_port_combo = QComboBox()
        self.kunwei_port_combo.setPlaceholderText("请选择坤维传感器串口")
        self.layout.addWidget(self.kunwei_label)
        self.layout.addWidget(self.kunwei_port_combo)

        # 保存数据复选框
        self.save_data_checkbox = QCheckBox("保存数据到CSV（仅实时采集）")
        self.save_data_checkbox.setChecked(SAVE_DATA_DEFAULT)
        self.layout.addWidget(self.save_data_checkbox)

        # 按钮布局
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

        # 信号绑定
        self.confirm_btn.clicked.connect(self.on_confirm)
        self.cancel_btn.clicked.connect(self.reject)
        self.replay_btn.clicked.connect(self.on_replay)

        # 加载可用串口
        self.load_available_ports()

    def load_available_ports(self):
        """复用代码2的跨平台串口加载逻辑，添加None选项"""
        self.sensor_port_combo.clear()
        self.kunwei_port_combo.clear()
        available_ports = list(list_ports.comports())

        # 先添加"不使用"选项
        self.sensor_port_combo.addItem("None", None)
        self.kunwei_port_combo.addItem("None", None)

        if not available_ports:
            QMessageBox.warning(self, "警告", "未检测到可用串口！")
            self.confirm_btn.setEnabled(False)
            return

        # 按系统排序
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

        # 添加到下拉框
        for port in sorted_ports:
            port_info = f"{port.device} - {port.description}"
            self.sensor_port_combo.addItem(port_info, port.device)
            self.kunwei_port_combo.addItem(port_info, port.device)

        # 默认选择第一个可用串口
        self.sensor_port_combo.setCurrentIndex(1 if self.sensor_port_combo.count() > 1 else 0)
        self.kunwei_port_combo.setCurrentIndex(1 if self.kunwei_port_combo.count() > 1 else 0)
        self.confirm_btn.setEnabled(True)

    def on_confirm(self):
        """实时采集确认，检查串口冲突"""
        self.selected_sensor_port = self.sensor_port_combo.currentData()
        self.selected_kunwei_port = self.kunwei_port_combo.currentData()
        self.save_data = self.save_data_checkbox.isChecked()
        self.is_replay = False

        # 检查串口冲突
        if self.selected_sensor_port and self.selected_kunwei_port and self.selected_sensor_port == self.selected_kunwei_port:
            QMessageBox.warning(self, "警告", "不能选择相同的串口用于两个设备！")
            return
        self.accept()

    def on_replay(self):
        """数据回放选择CSV文件"""
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
        self.kunwei_worker = None
        self.selected_sensor_port = None
        self.selected_kunwei_port = None
        self.save_data = SAVE_DATA_DEFAULT
        self.is_replay_mode = False
        self.selected_csv = None

        # 先显示选择对话框
        if not self.select_serial_or_replay():
            sys.exit(0)

        self.init_main_ui()

    def select_serial_or_replay(self):
        """选择实时采集或数据回放"""
        dialog = SerialSelectionDialog()
        if dialog.exec_() == QDialog.Accepted:
            self.is_replay_mode = dialog.is_replay
            if self.is_replay_mode:
                self.selected_csv = dialog.selected_csv
                return True
            else:
                self.selected_sensor_port = dialog.selected_sensor_port
                self.selected_kunwei_port = dialog.selected_kunwei_port
                self.save_data = dialog.save_data
                return True
        else:
            return False

    def init_main_ui(self):
        self.setWindowTitle("8x8+坤维传感器 - 图像与波形双视图（带滤波）")
        self.resize(1500, 800)
        self.showMaximized()

        self.central_widget = QSplitter()
        self.setCentralWidget(self.central_widget)

        # 左侧：图像显示 + 参数控制面板
        self.image_container = QWidget()
        self.image_container.setStyleSheet("background-color: black; border: none;")
        self.image_container_layout = QVBoxLayout(self.image_container)
        self.image_container_layout.setContentsMargins(0, 0, 0, 0)
        
        # 图像显示布局
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

        # 图像参数控制面板
        self.image_control_panel = QWidget()
        self.image_control_panel.setStyleSheet("background-color: black; border: none;")
        self.image_control_layout = QHBoxLayout(self.image_control_panel)
        self.image_control_layout.setContentsMargins(10, 5, 10, 5)
        self.image_control_layout.setSpacing(15)

        # 插值显示复选框
        self.interp_checkbox = QCheckBox("插值显示")
        self.interp_checkbox.setStyleSheet("color: white; background-color: #222222; border: 1px solid #555555; padding: 5px;")
        self.interp_checkbox.setChecked(self.image_visualizer.interplotation)
        self.interp_checkbox.stateChanged.connect(
            lambda state: self.image_visualizer.set_interpolation(state == QtCore.Qt.Checked)
        )
        self.image_control_layout.addWidget(self.interp_checkbox)

        # 插值密度调节
        self.zoom_label = QLabel("插值密度:")
        self.zoom_label.setStyleSheet("color: white;")
        self.zoom_spin = QSpinBox()
        self.zoom_spin.setRange(1, 15)
        self.zoom_spin.setValue(self.image_visualizer.zoom_factor)
        self.zoom_spin.setStyleSheet("""
            QSpinBox { color: white; background-color: #222222; border: 1px solid #555555; padding: 2px; }
            QSpinBox::up-button { background-color: #444444; border: 1px solid #666666; width: 16px; }
            QSpinBox::down-button { background-color: #444444; border: 1px solid #666666; width: 16px; }
        """)
        self.zoom_spin.valueChanged.connect(self.image_visualizer.set_zoom_factor)
        self.image_control_layout.addWidget(self.zoom_label)
        self.image_control_layout.addWidget(self.zoom_spin)

        # 高斯核参数调节
        self.sigma_label = QLabel("高斯核:")
        self.sigma_label.setStyleSheet("color: white;")
        self.sigma_spin = QDoubleSpinBox()
        self.sigma_spin.setRange(0.0, 10.0)
        self.sigma_spin.setSingleStep(0.5)
        self.sigma_spin.setValue(self.image_visualizer.gaussian_sigma)
        self.sigma_spin.setStyleSheet("""
            QDoubleSpinBox { color: white; background-color: #222222; border: 1px solid #555555; padding: 2px; }
            QDoubleSpinBox::up-button { background-color: #444444; border: 1px solid #666666; width: 16px; }
            QDoubleSpinBox::down-button { background-color: #444444; border: 1px solid #666666; width: 16px; }
        """)
        self.sigma_spin.valueChanged.connect(self.image_visualizer.set_gaussian_sigma)
        self.image_control_layout.addWidget(self.sigma_label)
        self.image_control_layout.addWidget(self.sigma_spin)

        # 旋转角度下拉框
        self.rotation_label = QLabel("旋转角度：")
        self.rotation_label.setStyleSheet("color: white;")
        self.rotation_combo = QComboBox()
        self.rotation_combo.addItems(["0度", "90度", "180度", "270度"])
        self.rotation_combo.setStyleSheet("""
            QComboBox { color: white; background-color: #222222; border: 1px solid #555555; padding: 2px 5px; }
            QComboBox::drop-down { background-color: #444444; border: 1px solid #666666; width: 20px; }
            QComboBox::down-arrow { border-left: 4px solid transparent; border-right: 4px solid transparent; border-top: 4px solid white; width: 0; height: 0; margin: 6px; }
            QComboBox QAbstractItemView { background-color: #222222; color: white; border: 1px solid #555555; }
        """)
        rotation_map = {0:0, 90:1, 180:2, 270:3}
        self.rotation_combo.setCurrentIndex(rotation_map[self.image_visualizer.rotation_angle])
        self.rotation_combo.currentIndexChanged.connect(
            lambda idx: self.image_visualizer.set_rotation_angle([0,90,180,270][idx])
        )
        self.image_control_layout.addWidget(self.rotation_label)
        self.image_control_layout.addWidget(self.rotation_combo)

        # 翻转复选框
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

        self.image_control_layout.addSpacerItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        self.image_container_layout.addWidget(self.image_control_panel)

        # 新增：滤波参数控制面板
        self.filter_control_panel = QWidget()
        self.filter_control_panel.setStyleSheet("background-color: black; border: none;")
        self.filter_control_layout = QHBoxLayout(self.filter_control_panel)
        self.filter_control_layout.setContentsMargins(10, 5, 10, 5)
        self.filter_control_layout.setSpacing(15)

        # 采样率调节
        self.fs_label = QLabel("采样率(Hz):")
        self.fs_label.setStyleSheet("color: white;")
        self.fs_spin = QDoubleSpinBox()
        self.fs_spin.setRange(10.0, 1000.0)
        self.fs_spin.setSingleStep(10.0)
        self.fs_spin.setValue(DEFAULT_FS)
        self.fs_spin.setStyleSheet("""
            QDoubleSpinBox { color: white; background-color: #222222; border: 1px solid #555555; padding: 2px; }
            QDoubleSpinBox::up-button { background-color: #444444; border: 1px solid #666666; width: 16px; }
            QDoubleSpinBox::down-button { background-color: #444444; border: 1px solid #666666; width: 16px; }
        """)
        self.fs_spin.valueChanged.connect(self.update_filter_parameters)
        self.filter_control_layout.addWidget(self.fs_label)
        self.filter_control_layout.addWidget(self.fs_spin)

        # 截止频率调节
        self.cutoff_label = QLabel("截止频率(Hz):")
        self.cutoff_label.setStyleSheet("color: white;")
        self.cutoff_spin = QDoubleSpinBox()
        self.cutoff_spin.setRange(0.1, 500.0)
        self.cutoff_spin.setSingleStep(1.0)
        self.cutoff_spin.setValue(DEFAULT_LOW_CUTOFF)
        self.cutoff_spin.setStyleSheet("""
            QDoubleSpinBox { color: white; background-color: #222222; border: 1px solid #555555; padding: 2px; }
            QDoubleSpinBox::up-button { background-color: #444444; border: 1px solid #666666; width: 16px; }
            QDoubleSpinBox::down-button { background-color: #444444; border: 1px solid #666666; width: 16px; }
        """)
        self.cutoff_spin.valueChanged.connect(self.update_filter_parameters)
        self.filter_control_layout.addWidget(self.cutoff_label)
        self.filter_control_layout.addWidget(self.cutoff_spin)

        # 滤波器阶数调节
        self.order_label = QLabel("滤波器阶数:")
        self.order_label.setStyleSheet("color: white;")
        self.order_spin = QSpinBox()
        self.order_spin.setRange(1, 8)
        self.order_spin.setValue(DEFAULT_FILTER_ORDER)
        self.order_spin.setStyleSheet("""
            QSpinBox { color: white; background-color: #222222; border: 1px solid #555555; padding: 2px; }
            QSpinBox::up-button { background-color: #444444; border: 1px solid #666666; width: 16px; }
            QSpinBox::down-button { background-color: #444444; border: 1px solid #666666; width: 16px; }
        """)
        self.order_spin.valueChanged.connect(self.update_filter_parameters)
        self.filter_control_layout.addWidget(self.order_label)
        self.filter_control_layout.addWidget(self.order_spin)

        # 最大值调节
        self.norm_high_label = QLabel("最大值:")
        self.norm_high_label.setStyleSheet("color: white;")
        self.norm_high_input = QSpinBox()
        self.norm_high_input.setRange(1, 5000)
        self.norm_high_input.setSingleStep(100)
        self.norm_high_input.setValue(NORMALIZATION_HIGH_VALUE)
        self.norm_high_input.setStyleSheet("""
            QSpinBox { color: white; background-color: #222222; border: 1px solid #555555; padding: 2px; }
            QSpinBox::up-button { background-color: #444444; border: 1px solid #666666; width: 16px; }
            QSpinBox::down-button { background-color: #444444; border: 1px solid #666666; width: 16px; }
        """)
        self.norm_high_input.valueChanged.connect(self.set_normalization_high)
        self.filter_control_layout.addWidget(self.norm_high_label)
        self.filter_control_layout.addWidget(self.norm_high_input)

        self.filter_control_layout.addSpacerItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        self.image_container_layout.addWidget(self.filter_control_panel)

        # 右侧：波形显示
        self.right_container = QWidget()
        self.right_container.setStyleSheet("background-color: black; border: 2px solid black;")
        self.right_layout = QVBoxLayout(self.right_container)
        self.right_layout.setContentsMargins(0, 0, 0, 0)
        self.right_layout.setSpacing(2)

        # 坤维波形（仅实时采集且选择坤维串口时显示）
        self.kunwei_waveform_widget = QWidget()
        self.kunwei_waveform_visualizer = KunweiWaveformVisualizer(self.kunwei_waveform_widget)

        # 8x8点位波形
        self.original_waveform_widget = QWidget()
        self.waveform_visualizer = RoutineWaveformVisualizer(self.original_waveform_widget)
        self.waveform_visualizer.set_reset_callback(self.reset_initial_value)

        # 布局逻辑：回放模式只显示8x8波形，实时模式按选择显示
        if self.is_replay_mode or self.selected_sensor_port:
            self.central_widget.addWidget(self.image_container)
            self.right_layout.addWidget(self.original_waveform_widget, stretch=1)
        if not self.is_replay_mode and self.selected_kunwei_port:
            self.right_layout.addWidget(self.kunwei_waveform_widget, stretch=1)
        self.central_widget.addWidget(self.right_container)

        # 线程初始化
        if self.is_replay_mode:
            # 回放模式：只启动DataReplayWorker
            self.worker = DataReplayWorker(
                csv_path=self.selected_csv,
                save_data=False,
                normalization_low=0,
                normalization_high=self.norm_high_input.value(),
                fs=self.fs_spin.value(),
                low_cutoff=self.cutoff_spin.value(),
                filter_order=self.order_spin.value()
            )
            self.worker.replay_finished.connect(self.close)
            self.worker.data_ready.connect(self.image_visualizer.receive_data, Qt.QueuedConnection)
            self.worker.waveform_ready.connect(self.waveform_visualizer.update_plot, Qt.QueuedConnection)
            self.worker.error_signal.connect(self.show_serial_error)
            self.worker.start()
        else:
            # 实时模式：按选择启动对应线程
            if self.selected_sensor_port:
                self.worker = SerialWorker(
                    port=self.selected_sensor_port,
                    save_data=self.save_data,
                    normalization_low=0,
                    normalization_high=self.norm_high_input.value(),
                    fs=self.fs_spin.value(),
                    low_cutoff=self.cutoff_spin.value(),
                    filter_order=self.order_spin.value()
                )
                self.worker.data_ready.connect(self.image_visualizer.receive_data)
                self.worker.waveform_ready.connect(self.waveform_visualizer.update_plot)
                self.worker.error_signal.connect(self.show_serial_error)
                self.worker.start()
            if self.selected_kunwei_port:
                self.kunwei_worker = KunweiSerialWorker(
                    port=self.selected_kunwei_port,
                    save_data=self.save_data,
                    fs=self.fs_spin.value(),
                    low_cutoff=self.cutoff_spin.value(),
                    filter_order=self.order_spin.value()
                )
                self.kunwei_worker.kunwei_data_ready.connect(self.handle_kunwei_data)
                self.kunwei_worker.kunwei_data_ready.connect(self.kunwei_waveform_visualizer.update_plot)
                self.kunwei_worker.error_signal.connect(self.show_kunwei_serial_error)
                self.kunwei_worker.start()

        # FPS定时器
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.image_visualizer.update_fps)
        self.timer.start(200)

    # 新增：更新滤波参数
    def update_filter_parameters(self):
        fs = self.fs_spin.value()
        cutoff = self.cutoff_spin.value()
        order = self.order_spin.value()

        # 确保截止频率不超过采样率的一半（奈奎斯特频率）
        if cutoff >= 0.5 * fs:
            cutoff = 0.45 * fs  # 设置为采样率的45%
            self.cutoff_spin.setValue(cutoff)
            QMessageBox.warning(self, "参数警告", "截止频率不能超过采样率的一半！已自动调整")

        # 更新传感器线程滤波参数
        if self.worker:
            self.worker.update_filter_params(fs, cutoff, order)
        
        # 更新坤维传感器线程滤波参数
        if self.kunwei_worker:
            self.kunwei_worker.update_filter_params(fs, cutoff, order)

    def set_normalization_high(self, value):
        """实时更新最大值参数"""
        if self.worker:
            self.worker.normalization_high = value

    def reset_initial_value(self):
        if self.worker and not self.is_replay_mode:
            self.worker.reset_initialization()

    def handle_kunwei_data(self, data):
        pass

    def show_serial_error(self, error_msg):
        QMessageBox.critical(self, "传感器串口错误", error_msg)
        if self.worker:
            self.worker.stop()
            self.worker.wait()
            self.worker = None
        if self.is_replay_mode:
            self.close()

    def show_kunwei_serial_error(self, error_msg):
        QMessageBox.critical(self, "坤维传感器串口错误", error_msg)
        if self.kunwei_worker:
            self.kunwei_worker.stop()
            self.kunwei_worker.wait()
            self.kunwei_worker = None

    def closeEvent(self, event):
        self.hide()
        if self.worker:
            self.worker.stop()
            self.worker.wait()
        if self.kunwei_worker:
            self.kunwei_worker.stop()
            self.kunwei_worker.wait()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec_())