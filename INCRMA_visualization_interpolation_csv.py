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
import csv  # 新增CSV模块
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
        self.csv_file = None  # CSV文件对象
        self.csv_writer = None  # CSV写入器
        self.init_raw_data_saving()  # 初始化保存系统

    def init_raw_data_saving(self):
        """创建保存目录并启动写入线程"""
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        # 使用时间戳命名文件
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.save_dir, f"raw_data_{timestamp}.csv")

        # 创建并打开CSV文件
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        # 写入表头（示例：FrameIndex + 2100个数据列）
        header = ["FrameIndex"] + [f"data_{i}" for i in range(2100)]
        self.csv_writer.writerow(header)
        
        # 启动后台写入线程
        self.is_saving = True
        self.writer_thread = threading.Thread(
            target=self.write_raw_data_to_file, 
            daemon=True
        )
        self.writer_thread.start()

    def write_raw_data_to_file(self):
        """后台线程：从队列取数据并写入.csv文件"""
        frame_index = 0  # 帧计数器
        
        while self.is_saving:
            if not self.raw_data_queue.empty():
                data = self.raw_data_queue.get()
                
                # 将numpy数组转为列表
                data_list = data.tolist()
                
                # 添加帧索引
                row = [frame_index] + data_list
                self.csv_writer.writerow(row)
                frame_index += 1
                
            else:
                self.msleep(100)  # 避免CPU空转
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
                            # parsed = []
                            # for i in range(0, len(payload), 2):
                            #     low = payload[i]
                            #     high = payload[i+1]
                            #     value = low | (high << 8)
                            #     parsed.append(value)
                            
                            # 原始字节数据转为numpy数组 16位，直接转换
                            parsed = np.frombuffer(payload, dtype=np.uint16)

                            # print('len(parsed):',len(parsed))# 测试结果为1000
                            # print('parsed[0]:',parsed[0])
                            # print('parsed[1]:',parsed[1])
                            # print('parsed[100]:',parsed[100])

                            if len(parsed) == 1000:  # 原比较器板子接收10x10的数据
                                # print('len(parsed):',len(parsed))
                                
                                frames = np.array(parsed).reshape(10, 100)
                                # 计算十帧的平均值
                                # average = np.mean(frames, axis=0).tolist()  # 转换为列表
                                average = np.mean(frames, axis=0)  # 保持为NumPy数组  ***这是从单片机接收来的原数据***

                                # print('average:\n', average) # 力越大数值越小
                                # self.data_ready.emit(average.tolist())
                                if self.matrix_flag == 0 :# 获得初始帧作为基准帧
                                    self.matrix_init = average.copy()  # 保存为数组
                                    self.matrix_flag = 1
                                else:
                                    result = self.matrix_init - average  # 数组支持元素级减法  力越大数值越大
                                    # 限幅到[-50, 500]范围内    ***每更换一个器件就需要重新设定范围***
                                    clipped_result = np.clip(result, a_min=self.normalization_low, a_max=self.normalization_high)
                                    # 归一化到[0, 1]范围
                                    normalized_result = (clipped_result - (self.normalization_low)) / (self.normalization_high - self.normalization_low)  # 分母是550（500-(-50)）
                                    result = normalized_result
                                    self.data_ready.emit(result.tolist())  # 如果需要转回列表再发射
                            elif len(parsed) == 2100:# INCRMA板子接收11x10的数据
                                # 立即保存原始数据
                                self.raw_data_queue.put(parsed.copy())

                                frames = np.array(parsed).reshape(10, 21, 10)# 10帧矩阵 矩阵有21行10列

                                # 方法3：完全不使用循环，直接在矩阵上计算
                                # 提取所有帧的基准行（第一行）
                                Rref_first_rows = frames[:, 0, :]  # 形状 (10帧 × 10列)
                                expanded_Rref = np.expand_dims(Rref_first_rows, axis=1)# 添加新维度，变为 (10, 1, 10)
                                Rref_first_rows = np.repeat(expanded_Rref, repeats=10, axis=1)# 沿第二个轴复制 10 次，形状变为 (10, 10, 10)
                                Rref_ohm = [2.082,2.093,2.095,2.088,2.082,1.985,1.987,1.987,1.988,1.990]
                                # Rref_ohm = [51.1,51.2,51.2,51.1,51.1,49.1,48.7,49.0,48.8,48.9]

                                # 后续拆分成10和10
                                RMA_raw_part = frames[:, 1:11, :]   # 第二行开始前10行 (10,10,10)
                                INCRMA_part = frames[:, 11:, :]  # 后10行 (10,10,10)
                                # 计算分子和分母
                                RMA_numerator = (4095 - Rref_first_rows)  # 形状 (10×10)
                                RMA_denominator = (4095 - RMA_raw_part)  # 形状 (10×10×10)
                                INCRMA_numerator = (RMA_raw_part - INCRMA_part)  # 形状 (10×10)
                                INCRMA_denominator = (Rref_first_rows - INCRMA_part)  # 形状 (10×10×10)
                                # 处理分母为0的情况（避免除零错误）
                                RMA_denominator = np.where(RMA_denominator == 0, 1e-6, RMA_denominator)
                                INCRMA_denominator = np.where(INCRMA_denominator == 0, 1e-6, INCRMA_denominator)
                                # 推理计算电阻值
                                RMA_normalized = (RMA_numerator / RMA_denominator)*Rref_ohm  # 形状 (10×10×10)
                                INCRMA_normalized = (INCRMA_numerator / INCRMA_denominator)*Rref_ohm  # 形状 (10×10×10)
                                # 求均值 10帧的均值
                                avg_matrix = np.mean(INCRMA_normalized, axis=0)
                                # # 打印结果（已保留三位小数）
                                # with np.printoptions(precision=3, suppress=True):
                                #     print('avg_matrix:\n', avg_matrix)
                                #     # 计算均值和标准差
                                #     avg_mean = np.mean(avg_matrix)
                                #     avg_std = np.std(avg_matrix)
                                #     print(f'均值: {avg_mean:.3f}, 标准差: {avg_std:.3f}')

                                

                                # # 添加对数处理
                                # avg_matrix_log = np.log(avg_matrix + 1)  # 防止出现负数
                                # # 打印结果（已保留三位小数）
                                # with np.printoptions(precision=3, suppress=True):
                                #     print('avg_matrix_log:\n', avg_matrix_log)
                                # avg_matrix = avg_matrix_log




                                if self.matrix_flag == 0 :# 获得初始帧作为基准帧
                                    self.matrix_init = avg_matrix.copy()  # 保存为数组
                                    self.matrix_flag = 1
                                    # with np.printoptions(precision=3, suppress=True):
                                    #     print('self.matrix_init:\n', self.matrix_init)
                                else:
                                    result = self.matrix_init - avg_matrix  # 数组支持元素级减法  力越大数值越大

                                    normal_result = result/self.matrix_init
                                    normal_result = np.where(normal_result < 0, 0, normal_result)
                                    normal_result = np.where(normal_result > 1, 1, normal_result)
                                    result = normal_result


                                    # with np.printoptions(precision=3, suppress=True):
                                    #     print('normal_result:\n', normal_result)
                                    # # 限幅到[normalization_low, normalization_high]范围内    ***每更换一个器件就需要重新设定范围***
                                    # clipped_result = np.clip(result, a_min=self.normalization_low, a_max=self.normalization_high)
                                    # # 归一化到[0, 1]范围
                                    # normalized_result = (clipped_result - (self.normalization_low)) / (self.normalization_high - self.normalization_low)  # 分母是550（500-(-50)）
                                    # result = normalized_result

                                    # with np.printoptions(precision=3, suppress=True):
                                    #     print('result:\n', result)
                                    result = result.flatten()# 必须转为一维数组
                                    # print('result.tolist()\n', np.array(result.tolist()).shape)
                                    self.data_ready.emit(result.tolist())  # 如果需要转回列表再发送


                                # self.data_ready.emit(avg_matrix.tolist())

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

    def stop(self):
        self.running = False
        """停止线程时关闭保存流程"""
        self.is_saving = False
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
        if self.writer_thread and self.writer_thread.is_alive():
            self.writer_thread.join()
        # super().stop()
    


# 主窗口类
class MatrixVisualizer(QMainWindow):
    def __init__(self,interplotation = False,rotation_angle = 0,flip_horizontal = False,flip_vertical = False):
        super().__init__()
        self.setWindowTitle("实时传感器矩阵可视化")
        self.interplotation = interplotation # 插值标志
        # self.interplotation = False

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
        #'viridis', 'plasma', 'inferno', 'magma', 'cividis'  # Matplotlib风格
        #'CET-L17', 'CET-L18', 'CET-L19'  # 来自ColorBrewer的色阶方案
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
        
        # # 新增：添加控制台打印定时器
        # self.print_timer = QtCore.QTimer()
        # self.print_timer.timeout.connect(self.print_matrix)
        # self.print_timer.start(500)  # 每500ms打印一次

        self.frame_count = 0
        self.start_time = QtCore.QTime.currentTime()

    # 新增方法：定时打印矩阵数据
    def print_matrix(self):
        pass
        print("当前传感器数据矩阵:")
        for row in self.data:
            print(', '.join(f'{value:5.2f}' for value in row))
        print('-' * 40)

    def receive_data(self, new_row):
        """接收数据并存入队列"""
        self.data_queue.put(new_row)

    def update_plot(self):
        """定时器触发的数据更新函数"""
        # self.frame_count += 1  

        while not self.data_queue.empty():
            full_data = self.data_queue.get()
            if len(full_data) == 100:
                self.data = np.array(full_data).reshape(10, 10)

                # 新增：应用旋转
                if self.rotation_angle == 90:# 逆时针90°
                    self.data = np.rot90(self.data, 1)
                elif self.rotation_angle == 180:
                    self.data = np.rot90(self.data, 2)
                elif self.rotation_angle == 270:
                    self.data = np.rot90(self.data, 3)

                # 新增：应用镜像翻转
                if self.flip_horizontal:# 上下调换
                    self.data = np.fliplr(self.data)
                if self.flip_vertical:  # 左右调换
                    self.data = np.flipud(self.data)

                if self.interplotation :
                    # (10, 10)插值到100x100（10倍放大）
                    interpolated_data = zoom(self.data, (5, 5), order=3)  # 3阶插值
                    self.data = interpolated_data

                self.image_item.setImage(self.data, levels=(0.0, 1.0))
                self.frame_count += 1  
                              
        current_time = QtCore.QTime.currentTime()
        if self.start_time.msecsTo(current_time) >= 1000:  # 每1秒计算一次
            fps = self.frame_count
            self.fps_label.setText(f"FPS: {fps}")
            self.frame_count = 0
            self.start_time = current_time

        # # 固定坐标范围
        # self.plot.setXRange(0, 100)
        # self.plot.setYRange(0, 100)

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
        self.worker.stop()
        self.worker.wait()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 创建主窗口实例
    main_win = MatrixVisualizer(interplotation = True,rotation_angle = 270,flip_horizontal = False,flip_vertical = False)# False True
    main_win.setup_display()  # 初始化显示布局
    main_win.resize(800, 800)
    main_win.show()
    
    sys.exit(app.exec_())