# INCRMA_visualization_interpolation_npz.py
import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import serial
from queue import Queue
from PyQt5.QtCore import QThread, pyqtSignal

from scipy.ndimage import zoom  # 使用scipy的缩放函数 插值需要

import os
import threading
from datetime import datetime

class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 数据就绪信号
    
    def __init__(self, port='COM11', normalization_low=-20, normalization_high=110):
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
        self.data_buffer = []  # 使用列表缓存数据
        self.init_raw_data_saving()  # 初始化保存系统

    def init_raw_data_saving(self):
        """创建保存目录并启动写入线程"""
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        # 使用时间戳命名文件
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.npz_path = os.path.join(self.save_dir, f"raw_data_{timestamp}.npz")
        
        # 启动后台写入线程
        self.is_saving = True
        self.writer_thread = threading.Thread(
            target=self.write_raw_data_to_file, 
            daemon=True
        )
        self.writer_thread.start()

    def write_raw_data_to_file(self):
        """后台线程：从队列取数据并缓存到内存"""
        while self.is_saving:
            if not self.raw_data_queue.empty():
                data = self.raw_data_queue.get()
                self.data_buffer.append(data.tolist())
            else:
                self.msleep(100)  # 避免CPU空转
                
    def stop(self):
        self.running = False
        """停止线程时关闭保存流程"""
        self.is_saving = False
        if self.writer_thread and self.writer_thread.is_alive():
            self.writer_thread.join()
            
        print('正在保存缓冲区数据到npz文件')
        # 保存缓冲区数据到NPZ文件
        if len(self.data_buffer) > 0:
            # 添加元数据信息
            metadata = {
                'description': 'INCRMA原始传感器数据',
                'format_version': '1.0',
                'normalization_range': [self.normalization_low, self.normalization_high],
                'timestamp': datetime.now().isoformat()
            }
            
            # 保存为压缩的NPZ文件
            np.savez_compressed(
                self.npz_path,
                data=np.array(self.data_buffer),
                **metadata
            )
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

                            if len(parsed) == 1000:  # 原比较器板子接收10x10的数据
                                frames = np.array(parsed).reshape(10, 100)
                                average = np.mean(frames, axis=0)  # 保持为NumPy数组  

                                if self.matrix_flag == 0:  # 获得初始帧作为基准帧
                                    self.matrix_init = average.copy()
                                    self.matrix_flag = 1
                                else:
                                    result = self.matrix_init - average
                                    clipped_result = np.clip(result, a_min=self.normalization_low, a_max=self.normalization_high)
                                    normalized_result = (clipped_result - self.normalization_low) / (self.normalization_high - self.normalization_low)
                                    result = normalized_result
                                    self.data_ready.emit(result.tolist())
                            elif len(parsed) == 2100:  # INCRMA板子接收11x10的数据
                                # 立即保存原始数据
                                self.raw_data_queue.put(parsed.copy())

                                frames = np.array(parsed).reshape(10, 21, 10)
                                Rref_first_rows = frames[:, 0, :]
                                expanded_Rref = np.expand_dims(Rref_first_rows, axis=1)
                                Rref_first_rows = np.repeat(expanded_Rref, repeats=10, axis=1)
                                Rref_ohm = [2.082,2.093,2.095,2.088,2.082,1.985,1.987,1.987,1.988,1.990]

                                RMA_raw_part = frames[:, 1:11, :]   # 第二行开始前10行 (10,10,10)
                                INCRMA_part = frames[:, 11:, :]  # 后10行 (10,10,10)
                                
                                RMA_numerator = (4095 - Rref_first_rows)
                                RMA_denominator = (4095 - RMA_raw_part)
                                INCRMA_numerator = (RMA_raw_part - INCRMA_part)
                                INCRMA_denominator = (Rref_first_rows - INCRMA_part)
                                
                                RMA_denominator = np.where(RMA_denominator == 0, 1e-6, RMA_denominator)
                                INCRMA_denominator = np.where(INCRMA_denominator == 0, 1e-6, INCRMA_denominator)
                                
                                RMA_normalized = (RMA_numerator / RMA_denominator) * Rref_ohm
                                INCRMA_normalized = (INCRMA_numerator / INCRMA_denominator) * Rref_ohm
                                
                                avg_matrix = np.mean(INCRMA_normalized, axis=0)
                                # 打印结果（已保留三位小数）
                                with np.printoptions(precision=3, suppress=True):
                                    print('avg_matrix:\n', avg_matrix)
                                    # 计算均值和标准差
                                    avg_mean = np.mean(avg_matrix)
                                    avg_std = np.std(avg_matrix)
                                    print(f'均值: {avg_mean:.3f}, 标准差: {avg_std:.3f}')

                                if self.matrix_flag == 0:  # 获得初始帧作为基准帧
                                    self.matrix_init = avg_matrix.copy()
                                    self.matrix_flag = 1
                                    with np.printoptions(precision=3, suppress=True):
                                        print('self.matrix_init:\n', self.matrix_init)
                                else:
                                    result = self.matrix_init - avg_matrix
                                    normal_result = result / self.matrix_init
                                    normal_result = np.where(normal_result < 0, 0, normal_result)
                                    normal_result = np.where(normal_result > 1, 1, normal_result)
                                    result = normal_result
                                    result = result.flatten()  # 必须转为一维数组
                                    self.data_ready.emit(result.tolist())

                            else:
                                print('当前帧串口数据不完整')
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

class MatrixVisualizer(QMainWindow):
    def __init__(self, interplotation=False, rotation_angle=0, flip_horizontal=False, flip_vertical=False):
        super().__init__()
        self.setWindowTitle("实时传感器矩阵可视化")
        self.interplotation = interplotation
        
        # 新增旋转和翻转参数
        self.rotation_angle = rotation_angle          # 0/90/180/270
        self.flip_horizontal = flip_horizontal     # 水平镜像
        self.flip_vertical = flip_vertical       # 垂直镜像
        
        # 创建主窗口组件
        self.central_widget = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.central_widget)
        
        # 初始化图像显示区域
        self.image_item = pg.ImageItem()
        self.plot = self.central_widget.addPlot(title="10x10传感器数据矩阵")
        self.plot.addItem(self.image_item)
        self.plot.setLabels(left='Y轴', bottom='X轴')

        # 设置颜色映射
        self.color_map = pg.colormap.get('viridis')
        self.image_item.setColorMap(self.color_map)

        self.fps_label = pg.LabelItem(justify='left')
        self.central_widget.addItem(self.fps_label, 1, 0)  # 放置在下方
        
        # 初始化数据矩阵
        self.data = np.zeros((10, 10))  # 10x10的初始数据
        
        # 初始化串口数据队列和线程
        self.data_queue = Queue()
        self.worker = SerialWorker()
        self.worker.data_ready.connect(self.receive_data)
        self.worker.start()
        
        # 创建定时器定期更新界面
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

                # 应用旋转
                if self.rotation_angle == 90:
                    self.data = np.rot90(self.data, 1)
                elif self.rotation_angle == 180:
                    self.data = np.rot90(self.data, 2)
                elif self.rotation_angle == 270:
                    self.data = np.rot90(self.data, 3)

                # 应用镜像翻转
                if self.flip_horizontal:
                    self.data = np.fliplr(self.data)
                if self.flip_vertical:
                    self.data = np.flipud(self.data)

                if self.interplotation:
                    interpolated_data = zoom(self.data, (5, 5), order=3)
                    self.data = interpolated_data

                self.image_item.setImage(self.data, levels=(0.0, 1.0))
                self.frame_count += 1  
                              
        current_time = QtCore.QTime.currentTime()
        if self.start_time.msecsTo(current_time) >= 1000:  # 每1秒计算一次
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
        # 调整布局
        self.central_widget.addItem(self.plot, 0, 0)
        self.central_widget.addItem(color_bar, 0, 1)

    def closeEvent(self, event):
        """窗口关闭时停止串口线程"""
        self.hide()  # 先隐藏图形界面
        self.worker.stop()
        self.worker.wait()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 创建主窗口实例
    main_win = MatrixVisualizer(interplotation=True, rotation_angle=270)
    main_win.setup_display()
    main_win.resize(800, 800)
    main_win.show()
    
    sys.exit(app.exec_())