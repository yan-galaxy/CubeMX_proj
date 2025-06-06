import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import serial
from queue import Queue
from PyQt5.QtCore import QThread, pyqtSignal

from scipy.ndimage import zoom  # 使用scipy的缩放函数 插值需要
import threading
import os
from datetime import datetime
import numpy as np  # 新增numpy导入

# 串口数据处理线程
class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 数据就绪信号
    
    def __init__(self,port='COM11',normalization_low = -20,normalization_high = 110):
        super().__init__()
        self.port = port         # 串口号
        self.baudrate = 115200     # 波特率
        self.ser = None            # 串口对象
        self.running = False       # 线程运行标志
        self.buffer = bytearray()  # 接收缓冲区
        self.FRAME_HEADER = b'\x55\xAA\xBB\xCC'
        self.FRAME_TAIL = b'\xAA\x55\x66\x77'
        self.matrix_init = []
        self.matrix_flag = 0

        self.normalization_low = normalization_low
        self.normalization_high = normalization_high

        # 原始数据本地保存
        self.save_dir = "INCRMA_raw_data"
        self.raw_data_queue = Queue()
        self.writer_thread = None
        self.is_saving = False
        self.npy_file = None  # NPZ文件对象
        self.buffer_capacity = 1000  # 缓冲区容量
        self.data_buffer = np.zeros((self.buffer_capacity, 2100), dtype=np.uint16)  # 数据缓冲区
        self.buffer_index = 0  # 缓冲区当前索引
        self.frame_count = 0   # 帧计数器
        self.init_raw_data_saving()  # 初始化保存系统

    def init_raw_data_saving(self):
        """创建保存目录并启动写入线程"""
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        # 使用时间戳命名文件
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.npy_path = os.path.join(self.save_dir, f"raw_data_{timestamp}.npz")

        # 启动后台写入线程
        self.is_saving = True
        self.writer_thread = threading.Thread(
            target=self.write_raw_data_to_file, 
            daemon=True
        )
        self.writer_thread.start()

    def write_raw_data_to_file(self):
        """后台线程：从队列取数据并写入.npz文件"""
        while self.is_saving:
            if not self.raw_data_queue.empty():
                # 批量获取数据
                batch_size = min(self.raw_data_queue.qsize(), self.buffer_capacity - self.buffer_index)
                for _ in range(batch_size):
                    self.data_buffer[self.buffer_index] = self.raw_data_queue.get()
                    self.buffer_index += 1
                
                # 达到容量时写入文件
                if self.buffer_index >= self.buffer_capacity:
                    self.save_buffer_to_npz()
                    self.buffer_index = 0  # 重置缓冲区
            else:
                self.msleep(100)  # 避免CPU空转

    def save_buffer_to_npz(self):
        """将缓冲区数据保存到.npz文件"""
        try:
            # 1. 确保目录存在
            os.makedirs(os.path.dirname(self.npy_path), exist_ok=True)
            
            # 2. 临时文件路径
            temp_path = self.npy_path + ".tmp"
            
            # 3. 读取现有数据（如果存在）
            if os.path.exists(self.npy_path):
                with np.load(self.npy_path) as data:
                    existing = data["frames"]
            else:
                existing = np.empty((0, 2100), dtype=np.uint16)  # 空数组
            
            # 4. 合并数据
            combined = np.vstack((existing, self.data_buffer[:self.buffer_index]))
            
            # 5. 保存到临时文件
            np.savez_compressed(temp_path, frames=combined)
            
            # 6. 原子替换
            if os.path.exists(temp_path):
                os.replace(temp_path, self.npy_path)
        except Exception as e:
            print(f"保存NPZ文件失败: {e}")
            # 清理临时文件
            if os.path.exists(temp_path):
                os.remove(temp_path)

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
                            # 原始字节数据转为numpy数组 16位，直接转换
                            parsed = np.frombuffer(payload, dtype=np.uint16)

                            if len(parsed) == 1000:  # 原比较器板子接收10x10的数据
                                frames = np.array(parsed).reshape(10, 100)
                                average = np.mean(frames, axis=0)  # 保持为NumPy数组

                                if self.matrix_flag == 0 :# 获得初始帧作为基准帧
                                    self.matrix_init = average.copy()  # 保存为数组
                                    self.matrix_flag = 1
                                else:
                                    result = self.matrix_init - average
                                    clipped_result = np.clip(result, a_min=self.normalization_low, a_max=self.normalization_high)
                                    normalized_result = (clipped_result - self.normalization_low) / (self.normalization_high - self.normalization_low)
                                    result = normalized_result
                                    self.data_ready.emit(result.tolist())
                            elif len(parsed) == 2100:# INCRMA板子接收11x10的数据
                                # 立即保存原始数据
                                self.raw_data_queue.put(parsed.copy())

                                frames = np.array(parsed).reshape(10, 21, 10)
                                Rref_first_rows = frames[:, 0, :]
                                expanded_Rref = np.expand_dims(Rref_first_rows, axis=1)
                                Rref_first_rows = np.repeat(expanded_Rref, repeats=10, axis=1)
                                Rref_ohm = [2.082,2.093,2.095,2.088,2.082,1.985,1.987,1.987,1.988,1.990]

                                RMA_raw_part = frames[:, 1:11, :]
                                INCRMA_part = frames[:, 11:, :]
                                RMA_numerator = (4095 - Rref_first_rows)
                                RMA_denominator = (4095 - RMA_raw_part)
                                INCRMA_numerator = (RMA_raw_part - INCRMA_part)
                                INCRMA_denominator = (Rref_first_rows - INCRMA_part)
                                
                                RMA_denominator = np.where(RMA_denominator == 0, 1e-6, RMA_denominator)
                                INCRMA_denominator = np.where(INCRMA_denominator == 0, 1e-6, INCRMA_denominator)
                                
                                RMA_normalized = (RMA_numerator / RMA_denominator)*Rref_ohm
                                INCRMA_normalized = (INCRMA_numerator / INCRMA_denominator)*Rref_ohm
                                
                                avg_matrix = np.mean(INCRMA_normalized, axis=0)
                                
                                with np.printoptions(precision=3, suppress=True):
                                    print('avg_matrix:\n', avg_matrix)
                                    avg_mean = np.mean(avg_matrix)
                                    avg_std = np.std(avg_matrix)
                                    print(f'均值: {avg_mean:.3f}, 标准差: {avg_std:.3f}')

                                if self.matrix_flag == 0 :
                                    self.matrix_init = avg_matrix.copy()
                                    self.matrix_flag = 1
                                    with np.printoptions(precision=3, suppress=True):
                                        print('self.matrix_init:\n', self.matrix_init)
                                else:
                                    result = self.matrix_init - avg_matrix
                                    normal_result = result/self.matrix_init
                                    normal_result = np.where(normal_result < 0, 0, normal_result)
                                    normal_result = np.where(normal_result > 1, 1, normal_result)
                                    result = normal_result
                                    result = result.flatten()
                                    self.data_ready.emit(result.tolist())

                            else:
                                print('串口数据不完整')
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
            # 确保线程退出时保存剩余数据
            if self.buffer_index > 0:
                self.save_buffer_to_npz()

    def stop(self):
        """停止线程时关闭保存流程"""
        self.running = False
        self.is_saving = False
        # 保存剩余缓冲区数据
        if self.buffer_index > 0:
            self.save_buffer_to_npz()
        if self.writer_thread and self.writer_thread.is_alive():
            self.writer_thread.join()

# 主窗口类
class MatrixVisualizer(QMainWindow):
    def __init__(self,interplotation = False,rotation_angle = 0,flip_horizontal = False,flip_vertical = False):
        super().__init__()
        self.setWindowTitle("实时传感器矩阵可视化")
        self.interplotation = interplotation
        self.rotation_angle = rotation_angle
        self.flip_horizontal = flip_horizontal
        self.flip_vertical = flip_vertical
        
        self.central_widget = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.central_widget)
        
        self.image_item = pg.ImageItem()
        self.plot = self.central_widget.addPlot(title="10x10传感器数据矩阵")
        self.plot.addItem(self.image_item)
        self.plot.setLabels(left='Y轴', bottom='X轴')

        self.color_map = pg.colormap.get('viridis')
        self.image_item.setColorMap(self.color_map)

        self.fps_label = pg.LabelItem(justify='left')
        self.central_widget.addItem(self.fps_label, 1, 0)
        
        self.data = np.zeros((10, 10))
        
        self.data_queue = Queue()
        self.worker = SerialWorker()
        self.worker.data_ready.connect(self.receive_data)
        self.worker.start()
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(5)

        self.frame_count = 0
        self.start_time = QtCore.QTime.currentTime()

    def receive_data(self, new_row):
        """接收数据并存入队列"""
        self.data_queue.put(new_row)

    def update_plot(self):
        """定时器触发的数据更新函数"""
        while not self.data_queue.empty():
            full_data = self.data_queue.get()
            if len(full_data) == 100:
                self.data = np.array(full_data).reshape(10, 10)

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

                if self.interplotation :
                    interpolated_data = zoom(self.data, (5, 5), order=3)
                    self.data = interpolated_data

                self.image_item.setImage(self.data, levels=(0.0, 1.0))
                self.frame_count += 1
                              
        current_time = QtCore.QTime.currentTime()
        if self.start_time.msecsTo(current_time) >= 1000:
            fps = self.frame_count
            self.fps_label.setText(f"FPS: {fps}")
            self.frame_count = 0
            self.start_time = current_time

    def setup_display(self):
        """添加颜色条"""
        color_bar = pg.ColorBarItem(
            values=(0.0, 1.0),
            width=25,
            colorMap=self.color_map
        )
        color_bar.setImageItem(self.image_item)
        color_bar.setFixedHeight(700)
        self.central_widget.addItem(self.plot, 0, 0)
        self.central_widget.addItem(color_bar, 0, 1)

    def closeEvent(self, event):
        """窗口关闭时停止串口线程"""
        self.worker.stop()
        self.worker.wait()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    main_win = MatrixVisualizer(interplotation = True,rotation_angle = 270,flip_horizontal = False,flip_vertical = False)
    main_win.setup_display()
    main_win.resize(800, 800)
    main_win.show()
    
    sys.exit(app.exec_())