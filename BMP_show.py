# BMP_show.py - 四通道传感器可视化（增强版）
import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QHBoxLayout, QVBoxLayout, QGraphicsDropShadowEffect
from PyQt5.QtGui import QColor, QLinearGradient, QBrush, QPen
from PyQt5.QtCore import Qt, QTime, QTimer, QThread, pyqtSignal
import pyqtgraph as pg
import serial
from queue import Queue

class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 数据就绪信号
    
    def __init__(self, port='COM6', normalization_ranges=None):
        super().__init__()
        self.port = port         # 串口号
        self.baudrate = 115200   # 波特率
        self.ser = None          # 串口对象
        self.running = False     # 线程运行标志
        self.buffer = bytearray()  # 接收缓冲区
        self.FRAME_HEADER = b'\x55\xAA\xBB\xCC'
        self.FRAME_TAIL = b'\xAA\x55\x66\x77'
        self.last_values = [0.0, 0.0, 0.0, 0.0]  # 平滑滤波缓存
        
        # 默认阈值范围：[[低阈值, 高阈值], ...]
        if normalization_ranges is None:
            # 默认全部使用相同范围（保持向后兼容）
            self.normalization_ranges = [[0.0, 0.5]] * 4
        else:
            self.normalization_ranges = normalization_ranges

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
                            
                            if len(payload) == 16:  # 16字节数据(4 float)
                                parsed = np.frombuffer(payload, dtype='<f4')  # 显式指定小端序
                                parsed = parsed[::-1]  # 全部反转
                                # 新增：每个通道单独归一化
                                normalized = []
                                for i in range(4):
                                    low, high = self.normalization_ranges[i]
                                    value = parsed[i]
                                    # 计算归一化值（带安全范围限制）
                                    norm_value = (value - low) / (high - low) if high != low else 0.0
                                    # 确保在0-1范围（处理极端情况）
                                    norm_value = max(0.0, min(1.0, norm_value))
                                    normalized.append(norm_value)
                                    
                                # 应用平滑滤波
                                smoothed = [0.8 * self.last_values[i] + 0.2 * n for i, n in enumerate(normalized)]
                                self.last_values = smoothed
                                self.data_ready.emit(smoothed)
                            else:
                                print(f'数据长度错误: {len(payload)}')
                                
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

class FourSquareVisualizer(QMainWindow):
    def __init__(self, normalization_ranges=None):
        super().__init__()
        self.setWindowTitle("四通道传感器可视化")
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e1e;
            }
            QLabel {
                color: #ffffff;
                font-family: 'Segoe UI';
                font-size: 14px;
            }
            QWidget#container {
                background-color: #2d2d2d;
                border-radius: 10px;
                padding: 8px;
            }
        """)
        
        # 创建主窗口组件
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QHBoxLayout(self.central_widget)
        self.layout.setSpacing(200)
        self.layout.setContentsMargins(20, 20, 20, 20)
        
        # 创建四个矩形显示区域
        self.rect_labels = []
        self.color_map = pg.colormap.get('inferno')  # 更鲜艳的色图  plasma viridis inferno magma cividis
        
        for i in reversed(range(4)):
            # 创建容器widget
            container = QWidget()
            container.setObjectName("container")
            container.setFixedSize(250, 350)
            layout = QVBoxLayout(container)
            layout.setSpacing(8)
            layout.setContentsMargins(5, 5, 5, 5)
            
            # 计算图形视图的可用尺寸（扣除边距和间距）
            margins = layout.contentsMargins()
            spacing = layout.spacing()
            view_width = container.width() - margins.left() - margins.right()
            view_height = container.height() - margins.top() - margins.bottom() - 2 * spacing  # 两个间距
            
            # 创建图形视图和场景
            graphics_view = pg.GraphicsView()
            scene = pg.GraphicsScene()
            scene.setSceneRect(0, 0, view_width, view_height)  # 场景大小匹配可用区域
            
            # 创建独立渐变背景（每个通道独立）
            bg = pg.QtWidgets.QGraphicsRectItem(0, 0, view_width, view_height)
            gradient = QLinearGradient(0, 0, 0, view_height)
            gradient.setColorAt(0, QColor('#3a3a3a'))
            gradient.setColorAt(1, QColor('#2a2a2a'))
            bg.setBrush(QBrush(gradient))
            scene.addItem(bg)
            
            rect_scale = 0.99  # 矩形占视图的比例（0~1，可调整）
            rect_width = int(view_width * rect_scale)
            rect_height = int(view_height * rect_scale)
            # 居中计算
            x = (view_width - rect_width) // 2
            y = (view_height - rect_height) // 2
            rect_item = pg.QtWidgets.QGraphicsRectItem(x, y, rect_width, rect_height)
            rect_item.setBrush(pg.mkBrush('lightgray'))
            rect_item.setPen(pg.mkPen('#4a4a4a', width=2))
            rect_item.setGraphicsEffect(self.create_shadow_effect())  # 添加阴影
            scene.addItem(rect_item)
            
            graphics_view.setScene(scene)
            layout.addWidget(graphics_view)
            
            # 创建通道标签
            label = QLabel(f"通道 {i+1}")
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("font-size: 16px; font-weight: bold;")
            layout.addWidget(label)
            
            # 创建数值标签
            value_label = QLabel("0.0%")
            value_label.setAlignment(Qt.AlignCenter)
            value_label.setStyleSheet("color: #00ffcc; font-size: 18px; font-family: Consolas;")
            layout.addWidget(value_label)
            
            # 记录当前通道的图形项和标签
            self.rect_labels.append((rect_item, value_label))
            self.layout.addWidget(container)
        
        # 初始化串口数据队列和线程
        self.data_queue = Queue()
        self.worker = SerialWorker(normalization_ranges=normalization_ranges)
        self.worker.data_ready.connect(self.receive_data)
        self.worker.start()
        
        # 创建定时器定期更新界面
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(50)  # 每50ms更新一次
        
        # FPS显示
        self.fps_label = QLabel("实时更新")
        self.fps_label.setStyleSheet("color: #00ffcc; font-size: 14px;")
        self.layout.addWidget(self.fps_label)
        
        self.frame_count = 0
        self.start_time = QTime.currentTime()

    def create_shadow_effect(self):
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(15)
        shadow.setColor(QColor(0, 0, 0, 100))
        shadow.setOffset(0, 5)
        return shadow

    def receive_data(self, data):
        """接收数据并存入队列"""
        if len(data) == 4:  # 确保是四个浮点数
            self.data_queue.put(data)

    def update_display(self):
        """定时器触发的数据更新函数"""
        while not self.data_queue.empty():
            data = self.data_queue.get()
            for i, value in enumerate(data):
                # 添加颜色过渡动画
                self.animate_color(i, value)
                # 更新标签显示数值
                self.rect_labels[i][1].setText(f"{value*100:.1f}%")
            
            self.frame_count += 1
            
        current_time = QTime.currentTime()
        if self.start_time.msecsTo(current_time) >= 1000:  # 每1秒计算一次
            fps = self.frame_count
            self.fps_label.setText(f"刷新率: {fps} FPS")
            self.frame_count = 0
            self.start_time = current_time

    def animate_color(self, index, value):
        """带过渡效果的颜色更新"""
        color = self.color_map.map(value, mode='byte')
        current_brush = self.rect_labels[index][0].brush()
        current_color = current_brush.color()
        
        # 简单的颜色插值过渡
        r = int(current_color.red() * 0.7 + color[0] * 0.3)
        g = int(current_color.green() * 0.7 + color[1] * 0.3)
        b = int(current_color.blue() * 0.7 + color[2] * 0.3)
        new_color = QColor(r, g, b)
        
        self.rect_labels[index][0].setBrush(pg.mkBrush(new_color))

    def closeEvent(self, event):
        """窗口关闭时停止串口线程"""
        self.hide()  # 先隐藏图形界面
        self.worker.stop()
        self.worker.wait()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")  # 使用现代样式
    
    # 自定义各通道归一化范围：
    # [ [通道0低阈值, 高阈值], [通道1低阈值, 高阈值], ... ]
    custom_ranges = [
        [0.0, 0.5],      # 通道0  最左边
        [0.0, 0.4],      # 通道1
        [0.0, 0.3],      # 通道2
        [0.0, 0.15],      # 通道3  最右边
    ]
    
    # 创建主窗口实例
    visualizer = FourSquareVisualizer(normalization_ranges=custom_ranges)
    visualizer.resize(1400, 720)
    visualizer.show()
    
    sys.exit(app.exec_())