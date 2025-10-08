import sys
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QSplitter, QDialog, 
                             QVBoxLayout, QHBoxLayout, QComboBox, QPushButton, QMessageBox, QWidget, QLabel)
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
import time

# 帧数配置宏定义 - 可以修改这个值来改变每次接收的帧数 (100、150、200等)
FRAMES_PER_PACKET = 20

# 不准改我的注释！！！不准删！！！
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

class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 用于图像更新
    waveform_ready = pyqtSignal(list)  # 用于波形更新
    fps_ready = pyqtSignal(float)  # 用于帧率更新

    def __init__(self, port, normalization_low=0, normalization_high=3.5):
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
        # 注释掉初始化npz文件保存的代码
        # self.init_raw_data_saving()
        
        # 帧率计算相关变量
        self.frame_count = 0
        self.last_time = time.time()
        self.fps_update_interval = 1.0  # 每秒更新一次FPS

    # 注释掉初始化npz文件保存的方法
    # def init_raw_data_saving(self):
    #     if not os.path.exists(self.save_dir):
    #         os.makedirs(self.save_dir)
    #     timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    #     self.npz_path = os.path.join(self.save_dir, f"mic_{timestamp}.npz")
    #     self.is_saving = True
    #     self.writer_thread = threading.Thread(target=self.write_raw_data_to_file, daemon=True)
    #     self.writer_thread.start()

    # 注释掉写入npz文件的方法
    # def write_raw_data_to_file(self):
    #     while self.is_saving:
    #         if not self.raw_data_queue.empty():
    #             data = self.raw_data_queue.get()
    #             self.data_buffer.append(data.tolist())
    #         else:
    #             self.msleep(100)

    def stop(self):
        self.running = False
        # 注释掉npz文件保存相关代码
        # self.is_saving = False
        # if self.writer_thread and self.writer_thread.is_alive():
        #     self.writer_thread.join()
        # print('正在保存缓冲区数据到npz文件')
        # if len(self.data_buffer) > 0:
        #     metadata = {
        #         'description': '7MIC采集 10KHz',
        #         'format_version': '1.0',
        #         'normalization_range': [self.normalization_low, self.normalization_high],
        #         'timestamp': datetime.now().isoformat()
        #     }
        #     np.savez_compressed(self.npz_path, data=np.array(self.data_buffer), **metadata)
        # print('保存完毕')
        
        # 新增：直接保存为CSV文件
        if len(self.data_buffer) > 0:
            self.save_to_csv()

    def save_to_csv(self):
        """将收集的数据保存为CSV格式"""
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_file_path = os.path.join(self.save_dir, f"mic_{timestamp}.csv")
        
        # 获取数据
        raw_data = np.array(self.data_buffer)
        
        # 确保数据是二维数组 (帧数, FRAMES_PER_PACKET*7)
        if raw_data.ndim == 1:
            # 如果是一维数组，重塑为(1, FRAMES_PER_PACKET*7)
            raw_data = raw_data.reshape(1, -1)
        elif raw_data.ndim > 2:
            # 如果是更高维的数组，压平除最后一维外的所有维度
            raw_data = raw_data.reshape(-1, raw_data.shape[-1])
        
        # 将数据重塑为 (帧数*FRAMES_PER_PACKET, 7) 的形式
        # 每一列代表一个麦克风通道
        num_frames = raw_data.shape[0]
        num_points_per_channel = FRAMES_PER_PACKET  # 每帧10个点
        num_channels = 7
        
        # 创建一个新的数组来存储重新排列的数据
        reshaped_data = np.zeros((num_frames * num_points_per_channel, num_channels), dtype=np.uint16)
        
        for frame_idx in range(num_frames):
            frame_data = raw_data[frame_idx]
            for channel in range(num_channels):
                start_idx = channel * num_points_per_channel
                end_idx = (channel + 1) * num_points_per_channel
                reshaped_data[frame_idx*num_points_per_channel:(frame_idx+1)*num_points_per_channel, channel] = \
                    frame_data[start_idx:end_idx]
        
        # 创建列名（7列）
        column_names = [f'MIC{i+1}' for i in range(7)]
        
        # 保存为CSV文件
        header = ','.join(column_names)
        
        # 写入CSV文件
        with open(csv_file_path, 'w') as f:
            f.write(header + '\n')
            np.savetxt(f, reshaped_data, delimiter=',', fmt='%d')  # 使用整数格式保存原始数据
        
        print(f"数据已保存为CSV格式: {csv_file_path}")
        print(f"数据形状: {reshaped_data.shape} (数据点数, 通道数)")
        print(f"列数: {len(column_names)}")
        print(f"帧数: {num_frames}")

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
                            
                            # 使用宏定义的帧数来检查数据长度
                            expected_length = FRAMES_PER_PACKET * 7  # 帧数 * 通道数 * 每通道点数
                            if len(parsed) == expected_length:
                                # parsed = parsed[:100]
                                # parsed = parsed[100:200]
                                # parsed = parsed[200:300]
                                # parsed = parsed[300:400]
                                # parsed = parsed[400:500]
                                # parsed = parsed[500:600]
                                # parsed = parsed[600:]
                                self.raw_data_queue.put(parsed.copy())
                                # 收集数据到缓冲区用于保存为CSV
                                self.data_buffer.append(parsed.tolist())

                                # print("接收到数据帧:",parsed)
                                # parsed = parsed.flatten()

                                if self.matrix_flag == 0:
                                    self.matrix_init = parsed.copy()
                                    self.matrix_flag = 1
                                else:
                                    # result = parsed - self.matrix_init
                                    result = parsed
                                    clipped_result = np.clip(result, self.normalization_low, self.normalization_high)
                                    normalized_result = (clipped_result - self.normalization_low) / (self.normalization_high - self.normalization_low)
                                    
                                    # parsed[42]=4095 # 中心左上 42   中心左下43   中心右上34   中心右下35
                                    self.waveform_ready.emit(parsed.tolist())
                                    # parsed = parsed/4096.0
                                    # self.data_ready.emit(parsed.tolist())
                                    self.data_ready.emit(normalized_result.tolist())# normalized_result.tolist()    average.tolist()
                                
                                # 帧率计算
                                self.frame_count += 1
                                current_time = time.time()
                                if current_time - self.last_time >= self.fps_update_interval:
                                    fps = self.frame_count / (current_time - self.last_time)
                                    self.fps_ready.emit(fps)
                                    self.frame_count = 0
                                    self.last_time = current_time
                                
                                # # 打印结果（已保留三位小数）
                                # with np.printoptions(precision=3, suppress=True):
                                #     print('avg_matrix:\n', average)
                                #     # 计算均值和标准差
                                #     avg_mean = np.mean(average)
                                #     avg_std = np.std(average)
                                #     print(f'均值: {avg_mean:.3f}, 标准差: {avg_std:.3f}')

                            self.buffer = self.buffer[end_index + len(self.FRAME_TAIL):]
                            start_index = self.buffer.find(self.FRAME_HEADER)
                        else:
                            break
        except Exception as e:
            print(f"串口通信异常: {e}")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print('串口关闭')

class MultiChannelWaveformVisualizer:
    def __init__(self, layout):
        self.layout = layout
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
            show_data = [0] * FRAMES_PER_PACKET * 10  # 显示FRAMES_PER_PACKET*100个点
            
            self.plots.append(plot)
            self.curves.append(curve)
            self.show_data.append(show_data)
            
            # 使用预定义的位置
            row, col = positions[i]
            self.layout.addItem(plot, row, col)

    def update_plot(self, new_row):
        # 使用宏定义的帧数来检查数据长度
        expected_length = FRAMES_PER_PACKET * 7  # 帧数 * 通道数 * 每通道点数
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
        self.worker = None  # 串口线程（需在选择串口后初始化）
        self.fps_label = None  # 帧率显示标签

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
        self.setWindowTitle(f"7MIC波形显示 (每次{FRAMES_PER_PACKET}个数据包)")
        self.resize(1500, 800)

        self.central_widget = QSplitter()
        self.setCentralWidget(self.central_widget)

        # 创建波形显示组件（7个麦克风）
        self.waveform_layout = pg.GraphicsLayoutWidget()
        self.waveform_visualizer = MultiChannelWaveformVisualizer(self.waveform_layout)
        self.central_widget.addWidget(self.waveform_layout)
        
        # 在GraphicsLayoutWidget中添加FPS显示
        self.fps_label = pg.LabelItem(justify='right')
        self.fps_label.setText("FPS: --", color='green')
        self.waveform_layout.addItem(self.fps_label, row=3, col=0, colspan=3)

        # 初始化串口线程（传入用户选择的端口）
        self.worker = SerialWorker(port=self.selected_port)
        self.worker.waveform_ready.connect(self.waveform_visualizer.update_plot)
        self.worker.fps_ready.connect(self.update_fps_display)
        self.worker.start()
        
    def update_fps_display(self, fps):
        """更新帧率显示"""
        self.fps_label.setText(f"FPS: {fps:.1f}", color='green')

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