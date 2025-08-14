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
import os
import threading
from datetime import datetime

class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 用于图像更新
    waveform_ready = pyqtSignal(list)  # 用于波形更新

    def __init__(self, port='COM14', normalization_low=0, normalization_high=2550):
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

        self.normalization_low = normalization_low
        self.normalization_high = normalization_high

        self.save_dir = "Routine_acq_raw_data_8x8"
        self.raw_data_queue = Queue()
        self.writer_thread = None
        self.is_saving = False
        self.data_buffer = []
        self.init_raw_data_saving()

    def init_raw_data_saving(self):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.npz_path = os.path.join(self.save_dir, f"raw_data_8x8_{timestamp}.npz")
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
                'description': '常规采集8x8传感器数据',
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
                                average = np.mean(frames, axis=0).reshape(10, 10)
                                average = average[2:, 2:]
                                matrix_8x8 = average

                                # 定义8x8映射表
                                # mapping = [
                                #     [(0,0), (0,1), (0,4), (0,5), (1,0), (1,1), (1,4), (1,5)],
                                #     [(0,3), (0,2), (0,7), (0,6), (1,3), (1,2), (1,7), (1,6)],
                                #     [(2,0), (2,1), (2,4), (2,5), (3,0), (3,1), (3,4), (3,5)],
                                #     [(2,3), (2,2), (2,7), (2,6), (3,3), (3,2), (3,7), (3,6)],
                                #     [(4,0), (4,1), (4,4), (4,5), (5,0), (5,1), (5,4), (5,5)],
                                #     [(4,3), (4,2), (4,7), (4,6), (5,3), (5,2), (5,7), (5,6)],
                                #     [(6,0), (6,1), (6,4), (6,5), (7,0), (7,1), (7,4), (7,5)],
                                #     [(6,3), (6,2), (6,7), (6,6), (7,3), (7,2), (7,7), (7,6)],
                                # ]

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

                                # 创建目标8x8矩阵
                                target_matrix = np.zeros((8, 8))

                                for i in range(8):
                                    for j in range(8):
                                        x, y = mapping[i][j]
                                        target_matrix[i][j] = matrix_8x8[x][y]

                                average = target_matrix.flatten()

                                if self.matrix_flag == 0:
                                    self.matrix_init = average.copy()
                                    self.matrix_flag = 1
                                else:
                                    result = average - self.matrix_init
                                    clipped_result = np.clip(result, self.normalization_low, self.normalization_high)
                                    normalized_result = (clipped_result - self.normalization_low) / (self.normalization_high - self.normalization_low)
                                    
                                    # average[42]=4095 # 中心左上 42   中心左下43   中心右上34   中心右下35
                                    self.waveform_ready.emit(average.tolist())
                                    # average = average/4096.0
                                    # self.data_ready.emit(average.tolist())
                                    self.data_ready.emit(normalized_result.tolist())# normalized_result.tolist()    average.tolist()
                                
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

class RoutineWaveformVisualizer:
    def __init__(self, layout):
        self.layout = layout
        self.plot = pg.PlotItem(title="某通道波形")
        self.plot.setLabels(left='数值', bottom='帧编号')
        self.plot.showGrid(x=True, y=True)
        self.plot.setYRange(0, 4096)
        self.layout.addItem(self.plot, 0, 0)

        self.curve = self.plot.plot(pen='y')
        self.show_data = [0] * 300

    def update_plot(self, new_row):
        if len(new_row) == 64:
            center_sum = new_row[34]  # 使用第48个通道
            new_data = [center_sum]
            self.show_data = self.show_data[len(new_data):] + new_data

            x_data = np.arange(len(self.show_data))
            y_data = np.array(self.show_data)
            self.curve.setData(x_data, y_data)
            self.plot.setTitle(f"某通道波形 - 第 {len(self.show_data)} 帧")

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
        self.setWindowTitle("图像与波形双视图")
        self.resize(1500, 800)

        self.central_widget = QSplitter()
        self.setCentralWidget(self.central_widget)

        # 创建图像显示组件
        self.image_layout = pg.GraphicsLayoutWidget()
        self.image_visualizer = MatrixVisualizer(self.image_layout, interplotation=False, rotation_angle=0, flip_horizontal=True, flip_vertical=True)
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