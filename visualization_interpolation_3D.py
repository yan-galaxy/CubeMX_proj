import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import serial
from queue import Queue
from PyQt5.QtCore import QThread, pyqtSignal

from scipy.ndimage import zoom  # 使用scipy的缩放函数 插值需要
from PyQt5.QtWidgets import QGraphicsEllipseItem

# 串口数据处理线程
class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 数据就绪信号
    
    def __init__(self,port='COM9',normalization_low = -5,normalization_high = 150):
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
                                
                                frames = np.array(parsed).reshape(10, 100)
                                # 计算十帧的平均值
                                # average = np.mean(frames, axis=0).tolist()  # 转换为列表
                                average = np.mean(frames, axis=0)  # 保持为NumPy数组  ***这是从单片机接收来的原数据***

                                # print('average:\n', average) # 力越大数值越小
                                # self.data_ready.emit(average.tolist())
                                if self.matrix_flag == 0 :# 获得初始帧作为基准帧
                                    self.matrix_init = average.copy()  # 保存为数组
                                    self.matrix_flag = 1
                                    # 计算统计量
                                    avg = np.mean(self.matrix_init)
                                    max_val = np.max(self.matrix_init)
                                    min_val = np.min(self.matrix_init)
                                    var_val = np.var(self.matrix_init)

                                    print(f"初始矩阵统计信息:")
                                    print(f"平均值: {avg:.2f}")
                                    print(f"最大值: {max_val:.2f}")
                                    print(f"最小值: {min_val:.2f}")
                                    print(f"方差: {var_val:.2f}")
                                else:
                                    result = self.matrix_init - average  # 数组支持元素级减法  力越大数值越大
                                    # print('result:\n', result)
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
    def __init__(self,interplotation = False,rotation_angle = 0,flip_horizontal = False,flip_vertical = False):
        super().__init__()
        self.setWindowTitle("实时传感器矩阵可视化")
        self.interplotation = interplotation # 插值标志
        # self.interplotation = False

        self.single_data_values = []  # 存储实时数据点
        self.single_data_plot = None  # 曲线图对象
        self.single_data_curve = None # 曲线图曲线

        self.circle_items = []  # 存储所有圆形项
        self.base_radius = 1   # 基准半径
        self.radius_scale = 30 # 半径缩放系数（根据数据范围调整）
        self.init_coord = np.zeros(16).reshape(4, 4)

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
                self.frame_count += 1
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

                raw_data = self.data
                if self.interplotation :
                    # (10, 10)插值到100x100（10倍放大）
                    interpolated_data = zoom(self.data, (5, 5), order=3)  # 3阶插值
                    self.data = interpolated_data

                self.image_item.setImage(self.data, levels=(0.0, 1.0))
                  

                # 提取目标传感器数据（假设选择中心点（4,4））
                # target_row, target_col = 4, 4  # 10x10矩阵的中心位置（索引从0开始）
                # current_value = self.data[target_row, target_col]
                # 所有的总和
                current_value = np.sum(self.data)

                # 存储数据（最多保留100个点）
                self.single_data_values.append(current_value)
                if len(self.single_data_values) > 100:
                    self.single_data_values.pop(0)
                
                # 更新曲线数据
                if self.single_data_curve:
                    self.single_data_curve.setData(self.single_data_values)



                # 将10x10数据降采样到4x4
                # 步骤1：去除最外层一圈数据（保留中间8x8）
                clip8x8_matrix = raw_data[1:-1, 1:-1]  # 索引从1到8（不包含9）
                # 正确的reshape和transpose顺序
                original_blocks = clip8x8_matrix.reshape(4, 2, 4, 2).transpose(0, 2, 1, 3)
                
                # 步骤2：计算每个2x2块的平均值
                downsampled = original_blocks.mean(axis=(2, 3))  # 对每个块的行和列求平均
                # 步骤3：计算横向差（列方向）
                horizontal_diff_x = (
                    original_blocks[:,:,:,1].mean(axis=2)  # 右边列的平均值
                    - original_blocks[:,:,:,0].mean(axis=2)  # 左边列的平均值
                )

                # 步骤4：计算纵向差（行方向）
                vertical_diff_y = (
                    original_blocks[:,:,0,:].mean(axis=2)  # 上面行的平均值
                    - original_blocks[:,:,1,:].mean(axis=2)  # 下面行的平均值
                )

                # print(original_blocks)
                # print(original_blocks.shape)
                # print(clip8x8_matrix)



                spacing=50
                
                for idx, (circle, x0, y0) in enumerate(self.circle_items):
                    # 获取对应数据值（0-1范围）
                    value = downsampled.flat[idx]  # 扁平化后索引
                    
                    # 计算新半径（根据数据值缩放）
                    new_radius = self.base_radius * (1 + self.radius_scale * value)
                    
                    # 计算位置偏移（根据数据值调整）
                    offset_x = horizontal_diff_x.flat[idx] * spacing  # 中心对称偏移
                    offset_y = vertical_diff_y.flat[idx] * spacing
                    
                    # 更新圆形参数
                    rect = QtCore.QRectF(
                        x0 + offset_x - new_radius,
                        y0 + offset_y - new_radius,
                        2*new_radius,
                        2*new_radius
                    )
                    circle.setRect(rect)
                              
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
        """添加颜色条和新图形"""
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

        # # 添加单个传感器数据曲线图
        # self.single_data_plot = self.central_widget.addPlot(row=0, col=1, colspan=1)  # 调整到第二列
        # self.single_data_plot.setTitle("合力实时数据曲线")
        # self.single_data_plot.setLabels(left='归一化值', bottom='数据点序号')
        # self.single_data_plot.setYRange(0.0, 300.0)
        # self.single_data_curve = self.single_data_plot.plot(pen='y')

        # 新增：添加4x4圆形矩阵到第三列
        self.circle_plot = self.central_widget.addPlot(row=0, col=1)
        self.circle_plot.setTitle("4x4圆形矩阵")
        self.circle_plot.hideAxis('left')
        self.circle_plot.hideAxis('bottom')
        self.circle_plot.setXRange(-100, 100)
        self.circle_plot.setYRange(-100, 100)
        self.circle_plot.setAspectLocked(True, ratio=1)  # 锁定1:1纵横比

        # # 设置圆形参数
        # radius = 2  # 半径
        # spacing = 30  # 圆心间距
        # colors = ['g', 'r', 'b', 'y']  # 可选：不同颜色（可注释掉）

        # # 生成4x4矩阵坐标
        # x_centers = np.linspace(-spacing*1.5, spacing*1.5, 4)  # 自动计算坐标
        # y_centers = np.linspace(-spacing*1.5, spacing*1.5, 4)

        # # 循环创建圆形
        # for i, x in enumerate(x_centers):
        #     for j, y in enumerate(y_centers):
        #         # 计算矩形位置
        #         rect = QtCore.QRectF(
        #             x - radius,  # 左上角x
        #             y - radius,  # 左上角y
        #             2*radius,    # 宽度
        #             2*radius     # 高度
        #         )
        #         circle = QGraphicsEllipseItem(rect)

        #         # 设置样式（可选颜色变化）
        #         # color = colors[(i+j)%4]  # 交替颜色
        #         # circle.setPen(pg.mkPen(color, width=2))
        #         # circle.setBrush(pg.mkBrush(color))
        #         circle.setPen(pg.mkPen('g', width=2))  # 绿色边框
        #         circle.setBrush(pg.mkBrush(0, 255, 0, 255))  # 绿色半透明填充  第四个参数是透明度，取值范围0-255

        #         self.circle_plot.addItem(circle)


        spacing = 30  # 圆心间距
        for i in range(4):
            for j in range(4):
                # 计算初始位置（保持中心对称）
                x_center = (i - 1.5) * spacing
                y_center = (j - 1.5) * spacing
                
                rect = QtCore.QRectF(
                    x_center - self.base_radius, 
                    y_center - self.base_radius, 
                    2*self.base_radius, 
                    2*self.base_radius
                )
                circle = QGraphicsEllipseItem(rect)
                circle.setPen(pg.mkPen('g', width=2))
                circle.setBrush(pg.mkBrush(0, 255, 0, 255))  # 半透明绿色 第四个参数是透明度，取值范围0-255
                self.circle_plot.addItem(circle)
                self.circle_items.append( (circle, x_center, y_center) )  # 保存初始位置

    def closeEvent(self, event):
        """窗口关闭时停止串口线程"""
        self.worker.stop()
        self.worker.wait()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 创建主窗口实例
    main_win = MatrixVisualizer(interplotation = False,rotation_angle = 0,flip_horizontal = False,flip_vertical = False)
    main_win.setup_display()  # 初始化显示布局
    main_win.resize(1600, 900)
    # main_win.showFullScreen()  # 全屏模式
    # main_win.showMaximized()  # 启动时窗口最大化
    main_win.show()
    
    sys.exit(app.exec_())