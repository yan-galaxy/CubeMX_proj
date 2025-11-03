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
import struct
import time

# 帧数配置宏定义 - 可以修改这个值来改变每次接收的帧数 (100、150、200等  必须为10的倍数)
FRAMES_PER_PACKET = 100
# 数据保存选项的宏定义，默认不保存数据
SAVE_DATA_DEFAULT = False

# 不准改我的注释！！！不准删！！！

class SerialSelectionDialog(QDialog):
    """跨平台串口选择对话框（Windows/Ubuntu通用）"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("选择串口")
        self.setFixedSize(400, 350)  # 调整窗口大小以容纳新控件
        self.selected_7mic_port = None  # 存储用户选择的7MIC串口
        self.selected_kunwei_port = None   # 存储用户选择的坤维串口
        self.save_data = SAVE_DATA_DEFAULT  # 存储用户是否选择保存数据

        # 1. 布局初始化
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        # 2. 7MIC串口选择下拉框
        mic_label = QLabel("选择7MIC串口:")
        self.mic_port_combo = QComboBox()
        self.mic_port_combo.setPlaceholderText("请选择7MIC串口")
        
        # 3. 坤维传感器串口选择下拉框
        kunwei_label = QLabel("选择坤维传感器串口:")
        self.kunwei_port_combo = QComboBox()
        self.kunwei_port_combo.setPlaceholderText("请选择坤维传感器串口")
        
        # 4. 添加控件到布局
        self.layout.addWidget(mic_label)
        self.layout.addWidget(self.mic_port_combo)
        self.layout.addWidget(kunwei_label)
        self.layout.addWidget(self.kunwei_port_combo)

        # 5. 添加保存数据选项
        self.save_data_checkbox = QCheckBox("保存数据到文件")
        self.save_data_checkbox.setChecked(SAVE_DATA_DEFAULT)
        self.layout.addWidget(self.save_data_checkbox)

        # 6. 加载可用串口（跨平台）
        self.load_available_ports()

        # 7. 按钮布局（确认/取消）
        self.button_layout = QHBoxLayout()
        self.confirm_btn = QPushButton("确认选择")
        self.cancel_btn = QPushButton("取消")
        self.button_layout.addWidget(self.confirm_btn)
        self.button_layout.addWidget(self.cancel_btn)
        self.layout.addLayout(self.button_layout)

        # 8. 按钮信号连接
        self.confirm_btn.clicked.connect(self.on_confirm)
        self.cancel_btn.clicked.connect(self.reject)

    def load_available_ports(self):
        """加载当前系统所有可用串口（Windows: COMx / Ubuntu: /dev/ttyUSBx）"""
        self.mic_port_combo.clear()
        self.kunwei_port_combo.clear()
        available_ports = list(list_ports.comports())  # 跨平台串口检测
        
        if not available_ports:
            # 无可用串口时提示
            QMessageBox.warning(self, "警告", "未检测到可用串口！\n请检查设备连接后重试")
            self.confirm_btn.setEnabled(False)  # 禁用确认按钮
            return
        
        # 添加"不使用"选项
        self.mic_port_combo.addItem("None", None)
        self.kunwei_port_combo.addItem("None", None)
        
        # 添加可用串口到下拉框（显示端口名+描述，方便用户识别）
        for port in available_ports:
            port_info = f"{port.device} - {port.description}"  # 例：COM3 - USB Serial Port
            self.mic_port_combo.addItem(port_info, port.device)  # 存储真实端口名（如COM3）作为用户数据
            self.kunwei_port_combo.addItem(port_info, port.device)

        # 默认选择第一个串口
        if self.mic_port_combo.count() > 0:
            self.mic_port_combo.setCurrentIndex(1 if self.mic_port_combo.count() >= 1 else 0)
        if self.kunwei_port_combo.count() > 1:
            self.kunwei_port_combo.setCurrentIndex(1)
        elif self.kunwei_port_combo.count() > 0:
            self.kunwei_port_combo.setCurrentIndex(0)

    def on_confirm(self):
        """确认选择，保存串口并关闭对话框"""
        self.selected_7mic_port = self.mic_port_combo.currentData()  # 获取真实端口名
        self.selected_kunwei_port = self.kunwei_port_combo.currentData()   # 获取真实端口名
        self.save_data = self.save_data_checkbox.isChecked()  # 获取是否保存数据的选项
        
        # 检查是否选择了相同的串口
        if self.selected_7mic_port and self.selected_kunwei_port and self.selected_7mic_port == self.selected_kunwei_port:
            QMessageBox.warning(self, "警告", "不能选择相同的串口用于两个设备！")
            return
            
        self.accept()  # 关闭对话框并返回QDialog.Accepted

class MICSerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 用于图像更新
    waveform_ready = pyqtSignal(list)  # 用于波形更新
    fps_ready = pyqtSignal(float)  # 用于帧率更新
    error_signal = pyqtSignal(str)  # 串口错误信号（跨平台错误提示）

    def __init__(self, port, save_data=False, normalization_low=0, normalization_high=3.5):
        super().__init__()
        self.port = port  # 外部传入选择的串口（不再硬编码）
        self.baudrate = 460800
        self.ser = None
        self.running = False
        self.buffer = bytearray()
        self.FRAME_HEADER = b'\x55\xAA\xBB\xCC'
        self.FRAME_TAIL = b'\xAA\x55\x66\x77'
        self.matrix_init = []
        self.matrix_flag = 0

        self.normalization_low = normalization_low
        self.normalization_high = normalization_high

        self.save_dir = "7MIC_raw_data"
        self.raw_data_queue = Queue()
        self.writer_thread = None
        self.is_saving = False
        self.data_buffer = []
        self.csv_file_path = ""  # 存储CSV文件路径，未保存时为空
        self.save_data = save_data  # 接收外部传入的保存选项
        
        # 若选择保存，初始化数据保存相关资源
        if self.save_data:
            self.init_raw_data_saving()
        
        # 帧率计算相关变量
        self.frame_count = 0
        self.last_time = time.time()
        self.fps_update_interval = 1.0  # 每秒更新一次FPS

    def init_raw_data_saving(self):
        """仅在选择保存数据时，初始化CSV文件和写入线程"""
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        # 生成唯一的CSV文件名（含时间戳）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file_path = os.path.join(self.save_dir, f"mic_{timestamp}.csv")
        self.is_saving = True
        
        # 新增：创建CSV文件并写入表头（7个麦克风通道）
        with open(self.csv_file_path, 'w', encoding='utf-8') as f:
            header = ','.join([f'MIC{i+1}' for i in range(7)])  # 表头：MIC1,MIC2,...MIC7
            f.write(header + '\n')
        
        # 启动实时写入线程（守护线程，主程序退出时自动结束）
        self.writer_thread = threading.Thread(target=self.write_raw_data_to_csv, daemon=True)
        self.writer_thread.start()

    def write_raw_data_to_csv(self):
        """仅在选择保存数据时，执行CSV写入逻辑"""
        while self.is_saving:
            try:
                # 阻塞式获取数据，队列空时线程自动挂起（不占CPU）
                parsed_data = self.raw_data_queue.get(block=True, timeout=None)
                
                # 数据格式转换、写入CSV的逻辑
                points_per_channel = FRAMES_PER_PACKET
                reshaped_data = np.zeros((points_per_channel, 7), dtype=np.uint16)
                for channel in range(7):
                    start_idx = channel * points_per_channel
                    end_idx = (channel + 1) * points_per_channel
                    reshaped_data[:, channel] = parsed_data[start_idx:end_idx]
                
                # 实时追加写入CSV
                with open(self.csv_file_path, 'a', encoding='utf-8') as f:
                    np.savetxt(f, reshaped_data, delimiter=',', fmt='%d')
                    
                self.raw_data_queue.task_done()  # 标记任务完成，避免内存泄漏
                
            except Exception as e:
                # 仅在非主动停止时打印异常
                if self.is_saving:
                    print(f"实时写入CSV异常: {e}")
                break

    def stop(self):
        self.running = False
        self.is_saving = False  # 停止线程循环
        
        # 若开启了保存，唤醒阻塞的写入线程并释放资源
        if self.save_data and self.raw_data_queue.empty():
            self.raw_data_queue.put(np.array([]))  # 放入空数组触发线程唤醒
        
        # 等待写入线程安全结束（超时2秒，防止卡死）
        if self.save_data and self.writer_thread and self.writer_thread.is_alive():
            self.writer_thread.join(timeout=2)
        
        # 仅在开启保存时，打印保存路径
        if self.save_data:
            print(f"实时CSV保存已停止，文件路径：{self.csv_file_path}")

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
                                
                                # 检查数据长度是否符合预期
                                expected_length = FRAMES_PER_PACKET * 7
                                if len(parsed) == expected_length:
                                    # 仅在开启保存时，将数据放入队列
                                    if self.save_data:
                                        self.raw_data_queue.put(parsed.copy())
                                    # 收集数据到缓冲区（用于手动保存）
                                    self.data_buffer.append(parsed.tolist())

                                    if self.matrix_flag == 0:
                                        self.matrix_init = parsed.copy()
                                        self.matrix_flag = 1
                                    else:
                                        result = parsed
                                        clipped_result = np.clip(result, self.normalization_low, self.normalization_high)
                                        normalized_result = (clipped_result - self.normalization_low) / (self.normalization_high - self.normalization_low)
                                        
                                        self.waveform_ready.emit(parsed.tolist())
                                        self.data_ready.emit(normalized_result.tolist())
                                    
                                    # 帧率计算
                                    self.frame_count += 1
                                    current_time = time.time()
                                    if current_time - self.last_time >= self.fps_update_interval:
                                        fps = self.frame_count / (current_time - self.last_time)
                                        self.fps_ready.emit(fps)
                                        self.frame_count = 0
                                        self.last_time = current_time

                                self.buffer = self.buffer[end_index + len(self.FRAME_TAIL):]
                                start_index = self.buffer.find(self.FRAME_HEADER)
                            else:
                                break
                except Exception as e:
                    # 捕获读取数据时的异常，但不终止线程
                    print(f"读取7MIC数据时出错: {str(e)}")
                    self.msleep(100)  # 等待一段时间再继续
        except Exception as e:
            # 发送错误信号到主线程，弹出可视化提示
            error_msg = f"7MIC串口通信异常: {str(e)}\nUbuntu用户请检查是否加入dialout组"
            self.error_signal.emit(error_msg)
        finally:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                    print('7MIC串口关闭')
                except OSError as e:
                    print(f"关闭7MIC串口时出错: {str(e)}")
            self.running = False

class KunweiSerialWorker(QThread):
    kunwei_data_ready = pyqtSignal(list)  # 发送坤维传感器数据
    error_signal = pyqtSignal(str)  # 串口错误信号

    def __init__(self, port, save_data=True):
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
        
        # CSV数据保存相关
        self.save_data = save_data
        self.csv_filename = f"7MIC_raw_data/kunwei_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = None
        self.csv_writer = None
        
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
                                    # 获取ns级时间戳（使用perf_counter_ns，系统级高精度时间）
                                    ns_timestamp = time.perf_counter_ns()
                                    # 写入CSV文件 - 拼接"时间戳+6个分量"，时间戳作为首列
                                    if self.save_data and self.csv_writer:
                                        self.csv_writer.writerow([ns_timestamp] + components)
                                    
                                    # 发送数据到主线程（用于波形显示）
                                    self.kunwei_data_ready.emit(components)
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
                    # 捕获读取数据时的异常，但不终止线程
                    print(f"读取坤维传感器数据时出错: {str(e)}")
                    # self.msleep(100)  # 等待一段时间再继续
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
        self.plot.setTitle("坤维传感器 - Fx/Fy/Fz/Mx/My/Mz")
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
        
        # 批量更新配置（核心模式控制变量）
        self.frame_buffer = []  # 临时存储原始帧
        self.update_batch_size = 10  # 每10帧处理一次
        self.use_average = True  # 代码内控制模式：True=10帧→1平均值；False=10帧真实数据

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

# ---------------------- 7MIC波形可视化类 ----------------------
class MultiChannelWaveformVisualizer:
    def __init__(self, widget):
        self.layout = widget.addLayout()
        self.plots = []
        self.curves = []
        self.show_data = []
        
        # 定义每个麦克风的位置 (行, 列)
        positions = [
            (0, 0),  # MIC 1
            (1, 0),  # MIC 2
            (2, 0),  # MIC 3
            (1, 1),  # MIC 4
            (2, 2),  # MIC 5
            (1, 2),  # MIC 6
            (0, 2),  # MIC 7
        ]
    
        # 为7个麦克风创建独立的波形显示
        for i in range(7):
            plot = pg.PlotItem(title=f"MIC {i+1}")
            plot.setLabels(left='数值', bottom='帧编号')
            plot.showGrid(x=True, y=True)
            plot.setYRange(0, 4095)
            
            curve = plot.plot(pen=pg.intColor(i))  # 使用不同颜色区分
            # 显示点数根据帧数动态调整
            show_data = [0] * FRAMES_PER_PACKET * 10  # 显示FRAMES_PER_PACKET*10个点
            
            self.plots.append(plot)
            self.curves.append(curve)
            self.show_data.append(show_data)
            
            # 使用预定义的位置
            row, col = positions[i]
            self.layout.addItem(plot, row, col)

    def update_plot(self, new_row):
        # 检查数据长度是否符合预期
        expected_length = FRAMES_PER_PACKET * 7
        if len(new_row) == expected_length:
            # 每个麦克风的数据点数根据帧数动态计算
            points_per_channel = FRAMES_PER_PACKET
            for mic in range(7):
                mic_data = new_row[mic*points_per_channel:(mic+1)*points_per_channel]
                
                # 更新显示数据
                self.show_data[mic] = self.show_data[mic][len(mic_data):] + mic_data
                
                # 更新图形
                x_data = np.arange(len(self.show_data[mic]))
                y_data = np.array(self.show_data[mic])
                self.curves[mic].setData(x_data, y_data)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.mic_worker = None  # 7MIC串口线程
        self.kunwei_worker = None  # 坤维传感器串口线程
        self.fps_label = None  # 帧率显示标签
        self.save_data = SAVE_DATA_DEFAULT  # 是否保存数据

        # 1. 先显示串口选择对话框（必须先选串口，再初始化主界面）
        self.select_serial_port()
        # 用户取消选择或无可用串口，直接退出
        if self.selected_7mic_port is None and self.selected_kunwei_port is None:
            sys.exit(0)

        # 2. 初始化主界面（选择串口成功后才执行）
        self.init_main_ui()

    def select_serial_port(self):
        """显示串口选择对话框，获取用户选择的端口"""
        dialog = SerialSelectionDialog()
        if dialog.exec_() == QDialog.Accepted:
            self.selected_7mic_port = dialog.selected_7mic_port
            self.selected_kunwei_port = dialog.selected_kunwei_port
            self.save_data = dialog.save_data
        else:
            self.selected_7mic_port = None
            self.selected_kunwei_port = None

    def init_main_ui(self):
        """初始化主界面（图像+波形显示）"""
        self.setWindowTitle("7MIC与坤维传感器波形显示")
        self.resize(1500, 800)
        # 添加以下行来使窗口最大化显示
        self.showMaximized()

        self.central_widget = QSplitter()
        self.setCentralWidget(self.central_widget)

        # ---------------------- 左侧：7MIC波形显示 ----------------------
        # 左侧总容器（统一黑色背景和边框）
        self.left_container = QWidget()
        self.left_container.setStyleSheet("""
            QWidget {
                background-color: black;
                border: 2px solid black;
            }
        """)
        self.left_layout = QVBoxLayout(self.left_container)
        self.left_layout.setContentsMargins(0, 0, 0, 0)  # 消除内边距
        self.left_layout.setSpacing(2)  # 组件间距2px

        # 1. 左侧：7MIC波形显示
        self.mic_waveform_widget = pg.GraphicsLayoutWidget()
        self.mic_waveform_visualizer = MultiChannelWaveformVisualizer(self.mic_waveform_widget)
        
        # 2. 在GraphicsLayoutWidget中添加FPS显示
        self.fps_label = pg.LabelItem(justify='right')
        self.fps_label.setText("FPS: --", color='green')
        self.mic_waveform_widget.addItem(self.fps_label, row=3, col=0, colspan=3)
        
        self.left_layout.addWidget(self.mic_waveform_widget)
        
        # ---------------------- 右侧：坤维传感器波形显示 ----------------------
        self.right_container = QWidget()
        self.right_container.setStyleSheet("""
            QWidget {
                background-color: black;
                border: 2px solid black;
            }
        """)
        self.right_layout = QVBoxLayout(self.right_container)
        self.right_layout.setContentsMargins(0, 0, 0, 0)  # 消除内边距
        self.right_layout.setSpacing(2)  # 组件间距2px

        # 右侧：坤维传感器波形
        self.kunwei_waveform_widget = QWidget()
        self.kunwei_waveform_visualizer = KunweiWaveformVisualizer(self.kunwei_waveform_widget)
        
        # 根据选择的传感器决定显示哪些组件
        if self.selected_7mic_port:
            self.left_layout.addWidget(self.mic_waveform_widget, stretch=1)
            self.central_widget.addWidget(self.left_container)
            
        if self.selected_kunwei_port:
            self.right_layout.addWidget(self.kunwei_waveform_widget, stretch=1)
            self.central_widget.addWidget(self.right_container)
            
        # ---------------------- 线程初始化与信号连接 ----------------------
        # 1. 7MIC传感器线程
        if self.selected_7mic_port:
            self.mic_worker = MICSerialWorker(port=self.selected_7mic_port, save_data=self.save_data)
            self.mic_worker.waveform_ready.connect(self.mic_waveform_visualizer.update_plot)
            self.mic_worker.fps_ready.connect(self.update_fps_display)
            self.mic_worker.error_signal.connect(self.show_mic_serial_error)
            self.mic_worker.start()
        
        # 2. 坤维传感器线程（波形信号连接）
        if self.selected_kunwei_port:
            self.kunwei_worker = KunweiSerialWorker(port=self.selected_kunwei_port, save_data=self.save_data)
            self.kunwei_worker.kunwei_data_ready.connect(self.kunwei_waveform_visualizer.update_plot)  # 波形更新
            self.kunwei_worker.error_signal.connect(self.show_kunwei_serial_error)
            self.kunwei_worker.start()

    def update_fps_display(self, fps):
        """更新帧率显示"""
        self.fps_label.setText(f"FPS: {fps:.1f}", color='green')
        
    def show_mic_serial_error(self, error_msg):
        """显示7MIC串口错误提示（可视化弹窗）"""
        QMessageBox.critical(self, "7MIC串口错误", error_msg)
        # 不再关闭整个程序，只停止对应的worker
        if self.mic_worker:
            self.mic_worker.stop()
            self.mic_worker.wait()
            self.mic_worker = None

    def show_kunwei_serial_error(self, error_msg):
        """显示坤维传感器串口错误提示（可视化弹窗）"""
        QMessageBox.critical(self, "坤维传感器串口错误", error_msg)
        # 不再关闭整个程序，只停止对应的worker
        if self.kunwei_worker:
            self.kunwei_worker.stop()
            self.kunwei_worker.wait()
            self.kunwei_worker = None

    def closeEvent(self, event):
        """窗口关闭时释放资源"""
        self.hide()
        if self.mic_worker:
            self.mic_worker.stop()
            self.mic_worker.wait()
        if self.kunwei_worker:
            self.kunwei_worker.stop()
            self.kunwei_worker.wait()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec_())