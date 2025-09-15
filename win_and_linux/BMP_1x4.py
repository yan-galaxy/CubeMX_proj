import sys
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, QHBoxLayout, QVBoxLayout, 
                             QGraphicsDropShadowEffect, QDialog, QComboBox, QPushButton, QMessageBox)
from PyQt5.QtGui import QColor, QLinearGradient, QBrush, QPen
from PyQt5.QtCore import Qt, QTime, QTimer, QThread, pyqtSignal
import pyqtgraph as pg
import serial
from serial.tools import list_ports  # 跨平台串口检测
from queue import Queue
import os
import threading
from datetime import datetime


class SerialSelectionDialog(QDialog):
    """跨平台串口选择对话框（Windows/Ubuntu通用）"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("选择串口")
        self.setFixedSize(300, 150)  # 固定窗口大小，避免跨平台布局错乱
        self.selected_port = None  # 存储用户选择的串口

        # 1. 布局初始化
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        # 2. 串口选择下拉框
        self.port_combo = QComboBox()
        self.port_combo.setPlaceholderText("请选择可用串口")
        self.layout.addWidget(self.port_combo)

        # 3. 加载可用串口（跨平台）
        self.load_available_ports()

        # 4. 按钮布局（确认/取消）
        self.button_layout = QVBoxLayout()
        self.confirm_btn = QPushButton("确认选择")
        self.cancel_btn = QPushButton("取消")
        self.button_layout.addWidget(self.confirm_btn)
        self.button_layout.addWidget(self.cancel_btn)
        self.layout.addLayout(self.button_layout)

        # 5. 按钮信号连接
        self.confirm_btn.clicked.connect(self.on_confirm)
        self.cancel_btn.clicked.connect(self.reject)

    def load_available_ports(self):
        """加载当前系统所有可用串口（Windows: COMx / Ubuntu: /dev/ttyUSBx）"""
        self.port_combo.clear()
        available_ports = list(list_ports.comports())  # 跨平台串口检测
        
        if not available_ports:
            # 无可用串口时提示
            QMessageBox.warning(self, "警告", "未检测到可用串口！\n请检查设备连接后重试")
            self.confirm_btn.setEnabled(False)  # 禁用确认按钮
            return
        
        # 添加可用串口到下拉框（显示端口名+描述，方便用户识别）
        for port in available_ports:
            port_info = f"{port.device} - {port.description}"  # 例：COM3 - USB Serial Port
            self.port_combo.addItem(port_info, port.device)  # 存储真实端口名（如COM3）作为用户数据

        # 默认选择第一个串口
        self.port_combo.setCurrentIndex(0)

    def on_confirm(self):
        """确认选择，保存串口并关闭对话框"""
        self.selected_port = self.port_combo.currentData()  # 获取真实端口名（如COM3 / /dev/ttyUSB0）
        self.accept()  # 关闭对话框并返回QDialog.Accepted


class SerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 数据就绪信号
    error_signal = pyqtSignal(str)  # 串口错误信号（跨平台错误提示）
    
    def __init__(self, port):
        super().__init__()
        self.port = port         # 串口号
        self.baudrate = 115200   # 波特率
        self.ser = None          # 串口对象
        self.running = False     # 线程运行标志
        self.buffer = bytearray()  # 接收缓冲区
        self.FRAME_HEADER = b'\x55\xAA\xBB\xCC'
        self.FRAME_TAIL = b'\xAA\x55\x66\x77'
        
        # CSV数据保存相关
        self.save_dir = "BMP_raw_data_1x4"
        self.raw_data_queue = Queue()
        self.writer_thread = None
        self.is_saving = False
        self.csv_file = None
        self.csv_path = None
        self.init_raw_data_saving()

    def init_raw_data_saving(self):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.save_dir, f"raw_data_{timestamp}.csv")
        
        # 初始化 CSV 文件并写入表头
        self.csv_file = open(self.csv_path, 'w', newline='')
        header = ','.join([f'Channel_{i}' for i in range(4)]) + '\n'
        self.csv_file.write(header)
        self.csv_file.flush()

        self.is_saving = True
        self.writer_thread = threading.Thread(target=self.write_raw_data_to_file, daemon=True)
        self.writer_thread.start()

    def write_raw_data_to_file(self):
        while self.is_saving:
            if not self.raw_data_queue.empty():
                data = self.raw_data_queue.get()
                if self.csv_file:
                    line = ','.join([f'{x:.6f}' for x in data]) + '\n'
                    self.csv_file.write(line)
                    self.csv_file.flush()  # 确保数据写入磁盘
            else:
                self.msleep(100)

    def stop(self):
        self.running = False
        self.is_saving = False
        if self.writer_thread and self.writer_thread.is_alive():
            self.writer_thread.join()
        print('保存完毕')

        # 关闭 CSV 文件
        if self.csv_file:
            self.csv_file.close()

    def run(self):
        try:
            # 跨平台串口连接（自动适配Windows COMx / Ubuntu /dev/ttyUSBx）
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
                                print(parsed)
                                parsed = parsed[::-1]  # 全部反转
                                
                                # 保存原始数据到CSV
                                self.raw_data_queue.put(parsed.copy())
                                
                                # 发送原始数据用于波形显示
                                self.data_ready.emit(parsed.tolist())
                            else:
                                print(f'数据长度错误: {len(payload)}')
                                
                            self.buffer = self.buffer[end_index + len(self.FRAME_TAIL):]
                            start_index = self.buffer.find(self.FRAME_HEADER)
                        else:
                            break
        except Exception as e:
            # 发送错误信号到主线程，弹出可视化提示
            error_msg = f"串口通信异常: {str(e)}\nUbuntu用户请检查是否加入dialout组"
            self.error_signal.emit(error_msg)
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print('串口关闭')


class WaveformVisualizer(QMainWindow):
    def __init__(self, port, y_range=None):
        super().__init__()
        self.worker = None  # 串口线程（需在选择串口后初始化）
        
        # 设置Y轴范围，默认为[0, 1.0]，可以根据需要调整
        self.y_range = y_range if y_range is not None else [0, 1.0]
        
        self.setWindowTitle("四通道原始数据波形显示")
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e1e;
            }
            QLabel {
                color: #ffffff;
                font-family: 'Segoe UI';
                font-size: 14px;
            }
        """)
        
        # 创建主窗口组件
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)
        self.layout.setSpacing(10)
        self.layout.setContentsMargins(10, 10, 10, 10)
        
        # 创建四个波形显示区域
        self.plots = []
        self.curves = []
        self.data_buffers = []
        
        # 为每个通道创建波形图
        for i in range(4):
            # 创建通道标签
            channel_label = QLabel(f"通道 {i+1}")
            channel_label.setAlignment(Qt.AlignCenter)
            channel_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #00ffcc;")
            self.layout.addWidget(channel_label)
            
            # 创建波形图
            plot_widget = pg.PlotWidget()
            plot_widget.setBackground('#2d2d2d')
            plot_widget.setLabel('left', '数值')
            plot_widget.setLabel('bottom', '时间')
            plot_widget.showGrid(x=True, y=True, alpha=0.5)
            
            # 设置Y轴范围
            plot_widget.setYRange(self.y_range[0], self.y_range[1])
            
            # 添加曲线
            curve = plot_widget.plot(pen=pg.mkPen(color=pg.intColor(i), width=2))
            self.layout.addWidget(plot_widget)
            
            # 初始化数据缓冲区（显示最近300个数据点）
            self.data_buffers.append([0.0] * 300)
            self.plots.append(plot_widget)
            self.curves.append(curve)
        
        # 添加间距
        self.layout.addSpacing(20)
        
        # FPS显示
        self.fps_label = QLabel("实时更新")
        self.fps_label.setStyleSheet("color: #00ffcc; font-size: 14px; qproperty-alignment: AlignCenter;")
        self.layout.addWidget(self.fps_label)
        
        # 初始化串口数据队列和线程
        self.data_queue = Queue()
        self.worker = SerialWorker(port=port)
        self.worker.data_ready.connect(self.receive_data)
        self.worker.error_signal.connect(self.show_serial_error)  # 绑定错误提示
        self.worker.start()
        
        # 创建定时器定期更新界面
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(50)  # 每50ms更新一次
        
        self.frame_count = 0
        self.start_time = QTime.currentTime()

    def receive_data(self, data):
        """接收数据并存入队列"""
        if len(data) == 4:  # 确保是四个浮点数
            self.data_queue.put(data)

    def update_display(self):
        """定时器触发的数据更新函数"""
        updated = False
        while not self.data_queue.empty():
            data = self.data_queue.get()
            for i in range(4):
                # 更新数据缓冲区
                self.data_buffers[i] = self.data_buffers[i][1:] + [data[i]]
                # 更新波形显示
                x_data = list(range(len(self.data_buffers[i])))
                y_data = self.data_buffers[i]
                self.curves[i].setData(x_data, y_data)
            
            self.frame_count += 1
            updated = True
            
        if updated:
            current_time = QTime.currentTime()
            if self.start_time.msecsTo(current_time) >= 1000:  # 每1秒计算一次
                fps = self.frame_count
                self.fps_label.setText(f"刷新率: {fps} FPS")
                self.frame_count = 0
                self.start_time = current_time

    def show_serial_error(self, error_msg):
        """显示串口错误提示（可视化弹窗）"""
        QMessageBox.critical(self, "串口错误", error_msg)
        self.close()  # 错误后关闭主窗口

    def closeEvent(self, event):
        """窗口关闭时停止串口线程"""
        self.hide()  # 先隐藏图形界面
        if self.worker:
            self.worker.stop()
            self.worker.wait()
        event.accept()


def select_serial_port():
    """显示串口选择对话框，获取用户选择的端口"""
    dialog = SerialSelectionDialog()
    if dialog.exec_() == QDialog.Accepted:
        return dialog.selected_port
    else:
        return None


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")  # 使用现代样式
    
    # 先显示串口选择对话框（必须先选串口，再初始化主界面）
    selected_port = select_serial_port()
    if not selected_port:
        # 用户取消选择或无可用串口，直接退出
        sys.exit(0)
    
    # 创建主窗口实例，设置Y轴范围为[0, 1.0]，可根据需要调整
    visualizer = WaveformVisualizer(port=selected_port, y_range=[99, 130.0])
    visualizer.resize(1000, 800)
    visualizer.show()
    
    sys.exit(app.exec_())