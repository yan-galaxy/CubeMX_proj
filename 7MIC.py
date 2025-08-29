
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

    def __init__(self, port='COM6', normalization_low=0, normalization_high=3.5):
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

        self.save_dir = "7MIC_raw_data"
        self.raw_data_queue = Queue()
        self.writer_thread = None
        self.is_saving = False
        self.data_buffer = []
        self.init_raw_data_saving()

    def init_raw_data_saving(self):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.npz_path = os.path.join(self.save_dir, f"mic_{timestamp}.npz")
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
                'description': '7MIC采集 10KHz',
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
                            
                            if len(parsed) == 700:
                                # parsed = parsed[:100]
                                # parsed = parsed[100:200]
                                # parsed = parsed[200:300]
                                # parsed = parsed[300:400]
                                # parsed = parsed[400:500]
                                # parsed = parsed[500:600]
                                # parsed = parsed[600:]
                                self.raw_data_queue.put(parsed.copy())

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
        
        # 为7个麦克风创建独立的波形显示
        for i in range(7):
            plot = pg.PlotItem(title=f"麦克风 {i+1}")
            plot.setLabels(left='数值', bottom='帧编号')
            plot.showGrid(x=True, y=True)
            plot.setYRange(0, 4095)
            
            curve = plot.plot(pen=pg.intColor(i))  # 使用不同颜色区分
            show_data = [0] * 1000  # 显示3000个点
            
            self.plots.append(plot)
            self.curves.append(curve)
            self.show_data.append(show_data)
            # 将每个图放在2行4列的网格中
            self.layout.addItem(plot, i // 4, i % 4)

    def update_plot(self, new_row):
        if len(new_row) == 700:
            # 每100个数据点中取前7个，分别对应7个麦克风
            for mic in range(7):
                mic_data = new_row[mic*100:(mic+1)*100]
                
                # 更新显示数据
                self.show_data[mic] = self.show_data[mic][len(mic_data):] + mic_data
                
                # 更新图形
                x_data = np.arange(len(self.show_data[mic]))
                y_data = np.array(self.show_data[mic])
                self.curves[mic].setData(x_data, y_data)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("7麦克风波形显示")
        self.resize(1500, 800)

        self.central_widget = QSplitter()
        self.setCentralWidget(self.central_widget)

        # 创建波形显示组件（7个麦克风）
        self.waveform_layout = pg.GraphicsLayoutWidget()
        self.waveform_visualizer = MultiChannelWaveformVisualizer(self.waveform_layout)
        self.central_widget.addWidget(self.waveform_layout)

        # 初始化串口线程
        self.worker = SerialWorker()
        self.worker.waveform_ready.connect(self.waveform_visualizer.update_plot)
        self.worker.start()

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