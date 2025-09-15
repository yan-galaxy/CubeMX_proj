# routine_acquisition_npz_wave.py (修复版)
import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QSplitter
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import serial
from queue import Queue
from PyQt5.QtCore import QThread, pyqtSignal
from scipy.ndimage import zoom
from scipy import signal  # 导入信号处理模块
import os
import threading
from datetime import datetime

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

    def __init__(self, port='COM13', normalization_low=0, normalization_high=0.7):
        super().__init__()
        self.port = port
        self.baudrate = 460800
        self.ser = None
        self.running = False
        self.buffer = bytearray()
        self.FRAME_HEADER = b'\x55\xAA\xBB\xCC'
        self.FRAME_TAIL = b'\xAA\x55\x66\x77'
        
        # 初始化滤波器（64个通道）
        self.filter_handlers = {
            i: FilterHandler(fs=66.7, high_cutoff=0.2, low_cutoff=10.0, order=1)
            for i in range(64)
        }
        self.filters_initialized = False  # 滤波器初始化标志

        self.normalization_low = normalization_low
        self.normalization_high = normalization_high

        self.save_dir = "BMP_raw_data_8x8"
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
        self.csv_path = os.path.join(self.save_dir, f"raw_data_8x8_{timestamp}.csv")
        
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

                                # 初始化滤波器（使用第一个数据点）
                                if not self.filters_initialized:
                                    for i in range(64):
                                        self.filter_handlers[i].initialize_states(parsed[i])
                                    self.filters_initialized = True
                                
                                # 对每个通道进行滤波处理
                                filtered_results = []
                                waveform_data = []
                                for i in range(64):
                                    # 高通滤波
                                    hp_val = self.filter_handlers[i].apply_high_pass(parsed[i])
                                    
                                    # 不处理
                                    abs_val = hp_val
                                    # # 取绝对值
                                    # abs_val = np.abs(hp_val)
                                    # # 只保留正数部分，负数置为0
                                    # abs_val = np.maximum(hp_val, 0)

                                    # 低通滤波
                                    lp_val = self.filter_handlers[i].apply_low_pass(abs_val)
                                    filtered_results.append(lp_val)
                                    waveform_data.append(lp_val)  # 波形数据使用低通滤波结果
                                
                                # 处理显示数据（裁剪和归一化）
                                clipped_result = np.clip(filtered_results, self.normalization_low, self.normalization_high)
                                normalized_result = (clipped_result - self.normalization_low) / (self.normalization_high - self.normalization_low)
                                
                                # 发送数据到界面显示
                                self.waveform_ready.emit(waveform_data)  # 发送滤波后的波形数据
                                self.data_ready.emit(normalized_result.tolist())  # 发送归一化后的图像数据
                            
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

class RoutineWaveformVisualizer:
    def __init__(self, layout):
        self.layout = layout
        self.plot = pg.PlotItem(title="某通道波形")
        self.plot.setLabels(left='数值', bottom='帧编号')
        self.plot.showGrid(x=True, y=True)
        self.plot.setYRange(0, 0.2)
        self.layout.addItem(self.plot, 0, 0)

        self.curve = self.plot.plot(pen='y')
        self.show_data = [0] * 300

    def update_plot(self, new_row):
        if len(new_row) == 64:
            center_sum = new_row[3]  # 使用第4个通道（索引3）的滤波后数据
            new_data = [center_sum]
            self.show_data = self.show_data[len(new_data):] + new_data

            x_data = np.arange(len(self.show_data))
            y_data = np.array(self.show_data)
            self.curve.setData(x_data, y_data)
            self.plot.setTitle(f"单通道波形 - 第 {len(self.show_data)} 帧")

class MatrixVisualizer:
    def __init__(self, layout, interplotation=False, rotation_angle=0, flip_horizontal=False, flip_vertical=False):
        self.layout = layout
        self.interplotation = interplotation
        self.rotation_angle = rotation_angle
        self.flip_horizontal = flip_horizontal
        self.flip_vertical = flip_vertical

        self.image_item = pg.ImageItem()
        self.plot = pg.PlotItem(title="8x8传感器数据矩阵（滤波后）")
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
        self.setWindowTitle("图像与波形双视图（带滤波）")
        self.resize(1500, 800)

        self.central_widget = QSplitter()
        self.setCentralWidget(self.central_widget)

        # 创建图像显示组件
        self.image_layout = pg.GraphicsLayoutWidget()
        self.image_visualizer = MatrixVisualizer(self.image_layout, interplotation=False, rotation_angle=0, flip_horizontal=False, flip_vertical=True)
        self.central_widget.addWidget(self.image_layout)

        # 创建波形显示组件
        self.waveform_layout = pg.GraphicsLayoutWidget()
        self.waveform_visualizer = RoutineWaveformVisualizer(self.waveform_layout)
        self.central_widget.addWidget(self.waveform_layout)

        # 初始化串口线程
        self.worker = SerialWorker()
        self.worker.data_ready.connect(self.image_visualizer.receive_data)
        self.worker.waveform_ready.connect(self.waveform_visualizer.update_plot)
        self.worker.start()

        # 设置定时器更新FPS
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.image_visualizer.update_fps)
        self.timer.start(200)

    def closeEvent(self, event):
        self.hide()
        self.worker.stop()
        self.worker.wait()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec_())