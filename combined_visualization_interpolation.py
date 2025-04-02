import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import serial
from queue import Queue
from PyQt5.QtCore import QThread, pyqtSignal

from scipy.ndimage import zoom  # 使用scipy的缩放函数 插值需要

# 串口数据处理线程
class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 数据就绪信号
    
    def __init__(self,port='COM7',normalization_low = -70,normalization_high = 500):
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
                            # print("接收到数据长度：", len(payload))# 已测试接收到 去除帧头帧尾 长度为2000(每个是8bit数据，并且两两组合代表一个16bit数据)
                            # print('payload[0]:',payload[0])#例 11
                            # print('payload[1]:',payload[1])#例 30
                            # print('payload[0]+payload[1]*256:',payload[0]+payload[1]*256)#例 7691
                            # print('payload[200]:',payload[200])#例 11
                            # print('payload[201]:',payload[201])#例 30
                            # print('payload[200]+payload[201]*256:',payload[200]+payload[201]*256)#例 7691
                            parsed = []
                            for i in range(0, len(payload), 2):
                                low = payload[i]
                                high = payload[i+1]
                                value = low | (high << 8)
                                parsed.append(value)
                            # print('len(parsed):',len(parsed))# 测试结果为1000
                            # print('parsed[0]:',parsed[0])
                            # print('parsed[1]:',parsed[1])
                            # print('parsed[100]:',parsed[100])

                            if len(parsed) == 1000:  # 确保数据完整
                                # 计算十帧的平均值
                                frames = np.array(parsed).reshape(10, 100)
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
    


# 主窗口类
class MatrixVisualizer(QMainWindow):
    def __init__(self,interplotation):
        super().__init__()
        self.setWindowTitle("实时传感器矩阵可视化")
        self.interplotation = interplotation # 插值标志
        # self.interplotation = False
        
        
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
        
        # 新增：添加控制台打印定时器
        self.print_timer = QtCore.QTimer()
        self.print_timer.timeout.connect(self.print_matrix)
        self.print_timer.start(500)  # 每500ms打印一次

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
    main_win = MatrixVisualizer(interplotation = False)
    main_win.setup_display()  # 初始化显示布局
    main_win.resize(800, 800)
    main_win.show()
    
    sys.exit(app.exec_())