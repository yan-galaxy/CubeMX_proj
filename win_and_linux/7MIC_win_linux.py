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
        self.setFixedSize(300, 180)  # 扩大窗口高度，容纳新增的勾选框
        self.selected_port = None  # 存储用户选择的串口
        self.save_data = SAVE_DATA_DEFAULT  # 存储用户是否选择保存数据

        # 1. 布局初始化
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        # 2. 串口选择下拉框
        self.port_combo = QComboBox()
        self.port_combo.setPlaceholderText("请选择可用串口")
        self.layout.addWidget(self.port_combo)

        # 3. 新增：保存数据勾选框
        self.save_data_checkbox = QCheckBox("保存数据到CSV文件")
        self.save_data_checkbox.setChecked(SAVE_DATA_DEFAULT)  # 按默认值设置勾选状态
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
        """确认选择，保存串口和保存选项并关闭对话框"""
        self.selected_port = self.port_combo.currentData()  # 获取真实端口名（如COM3 / /dev/ttyUSB0）
        self.save_data = self.save_data_checkbox.isChecked()  # 获取用户是否勾选保存数据
        self.accept()  # 关闭对话框并返回QDialog.Accepted

class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 用于图像更新
    waveform_ready = pyqtSignal(list)  # 用于波形更新
    fps_ready = pyqtSignal(float)  # 用于帧率更新

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

    def save_to_csv(self):
        """将收集的数据保存为CSV格式（仅在开启保存时有效）"""
        if not self.save_data:
            print("未开启数据保存功能，跳过手动保存")
            return
            
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_file_path = os.path.join(self.save_dir, f"mic_manual_{timestamp}.csv")
        
        # 获取数据
        raw_data = np.array(self.data_buffer)
        
        # 确保数据是二维数组 (帧数, FRAMES_PER_PACKET*7)
        if raw_data.ndim == 1:
            raw_data = raw_data.reshape(1, -1)
        elif raw_data.ndim > 2:
            raw_data = raw_data.reshape(-1, raw_data.shape[-1])
        
        # 数据重塑为 (帧数*FRAMES_PER_PACKET, 7)
        num_frames = raw_data.shape[0]
        num_points_per_channel = FRAMES_PER_PACKET
        num_channels = 7
        reshaped_data = np.zeros((num_frames * num_points_per_channel, num_channels), dtype=np.uint16)
        
        for frame_idx in range(num_frames):
            frame_data = raw_data[frame_idx]
            for channel in range(num_channels):
                start_idx = channel * num_points_per_channel
                end_idx = (channel + 1) * num_points_per_channel
                reshaped_data[frame_idx*num_points_per_channel:(frame_idx+1)*num_points_per_channel, channel] = \
                    frame_data[start_idx:end_idx]
        
        # 创建列名并写入CSV
        column_names = [f'MIC{i+1}' for i in range(7)]
        header = ','.join(column_names)
        
        with open(csv_file_path, 'w') as f:
            f.write(header + '\n')
            np.savetxt(f, reshaped_data, delimiter=',', fmt='%d')
        
        print(f"手动保存数据完成，文件路径: {csv_file_path}")

    def run(self):
        try:
            # 跨平台串口连接
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
        self.worker = None  # 串口线程（需在选择串口后初始化）
        self.fps_label = None  # 帧率显示标签
        self.save_data = SAVE_DATA_DEFAULT  # 存储是否保存数据的选项

        # 1. 先显示串口选择对话框（获取串口和保存选项）
        self.select_serial_port()
        if not self.selected_port:
            # 用户取消选择或无可用串口，直接退出
            sys.exit(0)

        # 2. 初始化主界面（选择串口成功后才执行）
        self.init_main_ui()

    def select_serial_port(self):
        """显示串口选择对话框，获取用户选择的端口和保存选项"""
        dialog = SerialSelectionDialog()
        if dialog.exec_() == QDialog.Accepted:
            self.selected_port = dialog.selected_port
            self.save_data = dialog.save_data  # 接收对话框的保存选项
        else:
            self.selected_port = None

    def init_main_ui(self):
        """初始化主界面（图像+波形显示）"""
        self.setWindowTitle(f"7MIC波形显示")
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

        # 初始化串口线程（传入用户选择的端口和保存选项）
        self.worker = SerialWorker(port=self.selected_port, save_data=self.save_data)
        # 无需再手动调用init_raw_data_saving()，Worker内部已根据save_data自动处理
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