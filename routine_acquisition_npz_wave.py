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
from scipy import signal  # 添加信号处理模块
import os
import threading
from datetime import datetime

class FilterHandler:
    """滤波器处理类，封装低通滤波的相关方法"""
    
    def __init__(self, fs, low_cutoff, order=4):
        """
        初始化滤波器参数
        
        参数:
        fs: 采样频率
        low_cutoff: 低通滤波器截止频率
        order: 滤波器阶数，默认为4
        """
        self.fs = fs
        self.nyquist = 0.5 * fs
        
        # 设计低通滤波器
        normal_low_cutoff = low_cutoff / self.nyquist
        self.b_low, self.a_low = signal.butter(order, normal_low_cutoff, 
                                              btype='low', analog=False)
        
        # 初始化滤波器状态变量
        self.zi_low = None
    
    def initialize_states(self, initial_value):
        """初始化滤波器状态，使用第一个数据点作为初值"""
        self.zi_low = signal.lfilter_zi(self.b_low, self.a_low) * initial_value
    
    def apply_low_pass(self, data_point):
        """对单个数据点应用低通滤波"""
        filtered_point, self.zi_low = signal.lfilter(self.b_low, self.a_low, 
                                                   [data_point], zi=self.zi_low)
        return filtered_point[0]



class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 用于图像更新
    waveform_ready = pyqtSignal(list)  # 用于波形更新

    def __init__(self, port='COM9', normalization_low=0, normalization_high=2500):#2500
        super().__init__()
        self.port = port
        self.baudrate = 460800
        self.ser = None
        self.running = False
        self.buffer = bytearray()
        self.FRAME_HEADER = b'\x55\xAA\xBB\xCC'
        self.FRAME_TAIL = b'\xAA\x55\x66\x77'
        self.matrix_init = []
        self.matrix_flag = 0
        self.init_frames = []  # 用于存储初始化帧
        self.init_frame_count = 0  # 初始化帧计数器
        self.max_init_frames = 100  # 最大初始化帧数
        self.dead_value = 0  # 可视化死点值

        # 初始化滤波器（100个通道，对应10x10矩阵）
        self.filter_handlers = {
            i: FilterHandler(fs=100.0, low_cutoff=5.0, order=1)
            for i in range(100)
        }
        self.filters_initialized = False  # 滤波器初始化标志


        self.normalization_low = normalization_low
        self.normalization_high = normalization_high

        self.save_dir = "Routine_acq_raw_data"
        self.raw_data_queue = Queue()
        self.writer_thread = None
        self.is_saving = False
        self.data_buffer = []
        self.init_raw_data_saving()

    def init_raw_data_saving(self):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.npz_path = os.path.join(self.save_dir, f"raw_data_{timestamp}.npz")
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
                'description': 'INCRMA原始传感器数据',
                'format_version': '1.0',
                'normalization_range': [self.normalization_low, self.normalization_high],
                'timestamp': datetime.now().isoformat()
            }
            np.savez_compressed(self.npz_path, data=np.array(self.data_buffer), **metadata)
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
                                self.raw_data_queue.put(parsed.copy())
                                frames = parsed.reshape(10, 100)
                                average = np.mean(frames, axis=0)

                                # 收集前100帧用于初始化
                                if self.init_frame_count < self.max_init_frames:
                                    self.init_frames.append(average.copy())
                                    self.init_frame_count += 1
                                    
                                    # 当收集到足够的帧时，计算平均值作为matrix_init
                                    if self.init_frame_count == self.max_init_frames:
                                        self.matrix_init = np.mean(self.init_frames, axis=0) + self.dead_value
                                else:
                                    # 初始化滤波器（使用第一个数据点）
                                    if not self.filters_initialized:
                                        for i in range(100):
                                            self.filter_handlers[i].initialize_states(average[i])
                                        self.filters_initialized = True
                                    
                                    # 对每个通道进行低通滤波处理
                                    filtered_results = []
                                    for i in range(100):
                                        lp_val = self.filter_handlers[i].apply_low_pass(average[i])
                                        filtered_results.append(lp_val)
                                        
                                    # result = average - self.matrix_init
                                    result = filtered_results - self.matrix_init
                                    
                                    
                                    clipped_result = np.clip(result, self.normalization_low, self.normalization_high)
                                    normalized_result = (clipped_result - self.normalization_low) / (self.normalization_high - self.normalization_low)
                                    
                                    # average[55]=4095 # 中心左上 44   中心左下45   中心右上54   中心右下55
    
                                    # self.waveform_ready.emit(average.tolist())
                                    self.waveform_ready.emit(filtered_results)
                                    # average = average/4096.0
                                    self.data_ready.emit(normalized_result.tolist())# normalized_result.tolist()    average.tolist()
                                    

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
        self.plot = pg.PlotItem(title="中心点波形")
        self.plot.setLabels(left='数值', bottom='帧编号')
        self.plot.showGrid(x=True, y=True)
        self.plot.setYRange(0, 4096)
        self.layout.addItem(self.plot, 0, 0)

        self.curve = self.plot.plot(pen='y')
        self.show_data = [0] * 300

    def update_plot(self, new_row):
        if len(new_row) == 100:
            center_sum = new_row[44]  # 使用第48个通道
            new_data = [center_sum]
            self.show_data = self.show_data[len(new_data):] + new_data

            x_data = np.arange(len(self.show_data))
            y_data = np.array(self.show_data)
            self.curve.setData(x_data, y_data)
            self.plot.setTitle(f"中心点波形")

class MatrixVisualizer:
    def __init__(self, layout, interplotation=False, rotation_angle=0, flip_horizontal=False, flip_vertical=False):
        self.layout = layout
        self.interplotation = interplotation
        self.rotation_angle = rotation_angle
        self.flip_horizontal = flip_horizontal
        self.flip_vertical = flip_vertical

        self.image_item = pg.ImageItem()
        self.plot = pg.PlotItem(title="10x10传感器数据矩阵")
        self.plot.addItem(self.image_item)
        self.plot.setLabels(left='Y轴', bottom='X轴')
        self.layout.addItem(self.plot, 0, 0)

        self.color_map = pg.colormap.get('viridis')
        self.image_item.setColorMap(self.color_map)

        self.data = np.zeros((10, 10))
        self.fps_label = pg.LabelItem(justify='left')
        self.layout.addItem(self.fps_label, 1, 0)

        self.frame_count = 0
        self.start_time = QtCore.QTime.currentTime()

    def receive_data(self, new_row):
        if len(new_row) == 100:
            self.data = np.array(new_row).reshape(10, 10)

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
        self.setWindowTitle("图像与波形双视图")
        self.resize(1500, 800)

        self.central_widget = QSplitter()
        self.setCentralWidget(self.central_widget)

        # 创建图像显示组件
        self.image_layout = pg.GraphicsLayoutWidget()
        self.image_visualizer = MatrixVisualizer(self.image_layout, interplotation=False, rotation_angle=0, flip_horizontal=True, flip_vertical=False)
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