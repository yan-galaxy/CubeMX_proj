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
    
    def __init__(self,port='COM7',normalization_low = -20,normalization_high = 110):
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
                            elif len(parsed) == 1100:# RMA板子接收11x10的数据
                                frames = np.array(parsed).reshape(10, 11, 10)# 10帧矩阵 矩阵有11行10列

                                # 方法1：手动使用每个循环计算，效率低
                                # for i in range(0, len(frames), 1):
                                #     print('frames[',i,']:\n', frames[i])
                                #     # 归一化处理：每列的后10个数据除以该列的第一个数据
                                #     normalized_matrix = np.zeros((10, 10))
                                #     for col in range(10):
                                #         first_val = frames[i, 0, col]  # 获取列首元素
                                #         if first_val != 0:  # 避免除以零
                                #             normalized_matrix[:, col] = (4095-first_val) / (4095-frames[i, 1:, col]) * 10.0  # 从第二行开始归一化
                                #         else:
                                #             # 处理分母为0的情况（可选：设为0或保留原始数据）
                                #             normalized_matrix[:, col] = 0  # 或其他默认值

                                #     # print('normalized_matrix:\n', normalized_matrix)
                                #     with np.printoptions(precision=3, suppress=True, formatter={'float_kind': lambda x: "%.3f" % x}):
                                #         print('normalized_matrix:\n', normalized_matrix)



                                # 方法2：10帧循环，每一帧数据使用numpy的广播机制，效率更高
                                # frames_list = []  # 存储所有帧的归一化矩阵
                                # for i in range(len(frames)): # 矩阵整体计算，不是逐行计算，效率高
                                #     frame_matrix = frames[i]  # 当前帧的11x10矩阵
                                #     # 提取基准行（第一行）
                                #     first_row = frame_matrix[0, :]
                                #     # 提取需要归一化的数据（去除第一行）
                                #     data_to_normalize = frame_matrix[1:, :]  # 形状 (10行 × 10列)
                                #     # 计算分子和分母
                                #     numerator = (4095 - first_row)  # 形状 (10,)
                                #     denominator = (4095 - data_to_normalize)  # 形状 (10行 × 10列)
                                #     # 处理分母为0的情况（避免除零错误）
                                #     denominator = np.where(denominator == 0, 1e-6, denominator)
                                #     # 归一化计算（利用广播机制）
                                #     normalized_matrix = (numerator / denominator) * 10.0  # 形状 (10×10)
                                #     # 打印结果（已保留三位小数）
                                #     with np.printoptions(precision=3, suppress=True):
                                #         print('normalized_matrix[',i,']:\n', normalized_matrix)
                                #     # 存储当前帧的归一化矩阵
                                #     frames_list.append(normalized_matrix)
                                #     avg_matrix = np.mean(frames_list, axis=0)


                                # 方法3：完全不使用循环，直接在矩阵上计算
                                # 提取所有帧的基准行（第一行）
                                first_rows = frames[:, 0, :]  # 形状 (10帧 × 10列)
                                # 提取需要归一化的数据（去除第一行）
                                data_to_normalize = frames[:, 1:, :]  # 形状 (10帧 × 10行 × 10列)
                                # 计算分子和分母
                                numerator = (4095 - first_rows)  # 形状 (10×10)
                                denominator = (4095 - data_to_normalize)  # 形状 (10×10×10)
                                # 处理分母为0的情况（避免除零错误）
                                denominator = np.where(denominator == 0, 1e-6, denominator)
                                # 归一化计算（利用广播机制）
                                # 将分子扩展为 (10×1×10) 以匹配分母的 (10×10×10)
                                normalized_matrices = (numerator[:, np.newaxis, :] / denominator) * 10.0  # 形状 (10×10×10)
                                
                                # 求均值
                                avg_matrix = np.mean(normalized_matrices, axis=0)
                                # # 打印结果（已保留三位小数）
                                # with np.printoptions(precision=3, suppress=True):
                                #     print('avg_matrix:\n', avg_matrix)




                                if self.matrix_flag == 0 :# 获得初始帧作为基准帧
                                    self.matrix_init = avg_matrix.copy()  # 保存为数组
                                    self.matrix_flag = 1
                                    with np.printoptions(precision=3, suppress=True):
                                        print('self.matrix_init:\n', self.matrix_init)
                                else:
                                    result = self.matrix_init - avg_matrix  # 数组支持元素级减法  力越大数值越大

                                    normal_result = result/self.matrix_init
                                    normal_result = np.where(normal_result < 0, 0, normal_result)
                                    normal_result = np.where(normal_result > 1, 0, normal_result)
                                    result = normal_result


                                    # with np.printoptions(precision=3, suppress=True):
                                    #     print('normal_result:\n', normal_result)
                                    # # 限幅到[normalization_low, normalization_high]范围内    ***每更换一个器件就需要重新设定范围***
                                    # clipped_result = np.clip(result, a_min=self.normalization_low, a_max=self.normalization_high)
                                    # # 归一化到[0, 1]范围
                                    # normalized_result = (clipped_result - (self.normalization_low)) / (self.normalization_high - self.normalization_low)  # 分母是550（500-(-50)）
                                    # result = normalized_result
                                    # # with np.printoptions(precision=3, suppress=True):
                                    # #     print('result:\n', result)

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
    main_win = MatrixVisualizer(interplotation = True,rotation_angle = 90,flip_horizontal = False,flip_vertical = True)
    main_win.setup_display()  # 初始化显示布局
    main_win.resize(800, 800)
    main_win.show()
    
    sys.exit(app.exec_())