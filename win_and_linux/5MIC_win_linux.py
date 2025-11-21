import sys
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QSplitter, QDialog, 
                             QVBoxLayout, QHBoxLayout, QComboBox, QPushButton, QMessageBox, QWidget, QLabel, QCheckBox)
from PyQt5.QtCore import QTimer  # 添加QTimer导入
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import serial
from serial.tools import list_ports  # 跨平台串口检测
from queue import Queue
from PyQt5.QtCore import QThread, pyqtSignal
from scipy.ndimage import zoom
import os
import threading
from datetime import datetime
import csv
import struct
import time

# 帧数配置宏定义 - 可以修改这个值来改变每次接收的帧数 (100、150、200等  必须为10的倍数)
FRAMES_PER_PACKET = 100
# 数据保存选项的宏定义，默认不保存数据
SAVE_DATA_DEFAULT = False

# 不准改我的注释！！！不准删！！！

class SerialSelectionDialog(QDialog):
    """跨平台串口选择对话框（Windows/Ubuntu通用）"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("选择串口")
        self.setFixedSize(400, 350)  # 调整窗口大小以容纳新控件
        self.selected_7mic_port = None  # 存储用户选择的5MIC串口
        self.save_data = SAVE_DATA_DEFAULT  # 存储用户是否选择保存数据

        # 1. 布局初始化
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        # 2. 5MIC串口选择下拉框
        mic_label = QLabel("选择5MIC串口:")
        self.mic_port_combo = QComboBox()
        self.mic_port_combo.setPlaceholderText("请选择5MIC串口")
        
        # 4. 添加控件到布局
        self.layout.addWidget(mic_label)
        self.layout.addWidget(self.mic_port_combo)

        # 5. 添加保存数据选项
        self.save_data_checkbox = QCheckBox("保存数据到文件")
        self.save_data_checkbox.setChecked(SAVE_DATA_DEFAULT)
        self.layout.addWidget(self.save_data_checkbox)

        # 7. 添加定时器用于实时检测串口变化
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_for_port_changes)
        self.timer.start(200)  # 每200ms检查一次串口变化

        # 8. 按钮布局（确认/取消）
        self.button_layout = QHBoxLayout()
        self.confirm_btn = QPushButton("确认选择")
        self.cancel_btn = QPushButton("取消")
        self.button_layout.addWidget(self.confirm_btn)
        self.button_layout.addWidget(self.cancel_btn)
        self.layout.addLayout(self.button_layout)

        # 9. 按钮信号连接
        self.confirm_btn.clicked.connect(self.on_confirm)
        self.cancel_btn.clicked.connect(self.reject)

        # 6. 加载可用串口（跨平台）
        self.load_available_ports()

    def load_available_ports(self):
        """加载当前系统所有可用串口（Windows: COMx / Ubuntu: /dev/ttyUSBx）"""
        self.mic_port_combo.clear()
        available_ports = list(list_ports.comports())  # 跨平台串口检测
        
        if not available_ports:
            # 无可用串口时提示
            QMessageBox.warning(self, "警告", "未检测到可用串口！\n请检查设备连接后重试")
            self.confirm_btn.setEnabled(False)  # 禁用确认按钮
            return
        
        # 添加"不使用"选项
        self.mic_port_combo.addItem("None", None)
        
        # 添加可用串口到下拉框（显示端口名+描述，方便用户识别）
        for port in available_ports:
            port_info = f"{port.device} - {port.description}"  # 例：COM3 - USB Serial Port
            self.mic_port_combo.addItem(port_info, port.device)  # 存储真实端口名（如COM3）作为用户数据

        # 默认选择第一个串口作为5MIC
        if self.mic_port_combo.count() > 0:
            self.mic_port_combo.setCurrentIndex(1 if self.mic_port_combo.count() >= 1 else 0)
            
        # # 当有可用端口时，启用确认按钮
        # self.confirm_btn.setEnabled(True)
        self.confirm_btn.setEnabled(True)  # 禁用确认按钮

    
    def check_for_port_changes(self):
        """检查串口变化并更新下拉列表"""
        current_ports = list(list_ports.comports())
        current_mic_port = self.mic_port_combo.currentData()
        
        # 获取当前系统中的所有端口名称
        current_port_names = [port.device for port in current_ports]
        
        # 检查是否有端口被移除
        ports_removed = False
        ports_added = False
        
        # 检查5MIC端口是否被移除
        if current_mic_port and current_mic_port != "None" and current_mic_port not in current_port_names:
            ports_removed = True
            
        # 检查是否有新端口加入
        mic_items = []
        for i in range(1, self.mic_port_combo.count()):  # 跳过"None"选项
            mic_items.append(self.mic_port_combo.itemData(i))
            
        for port in current_ports:
            if port.device not in mic_items:
                ports_added = True
                break
        
        # 如果有端口被移除或添加，更新下拉列表
        if ports_removed or ports_added:
            # 保存当前选择
            saved_mic_port = current_mic_port
            
            # 重新加载端口列表
            self.load_available_ports()
            
            # 尝试恢复之前的选择
            self.restore_port_selection(saved_mic_port)

    def restore_port_selection(self, mic_port):
        """恢复之前的端口选择"""
        # 恢复5MIC端口选择
        for i in range(self.mic_port_combo.count()):
            if self.mic_port_combo.itemData(i) == mic_port:
                self.mic_port_combo.setCurrentIndex(i)
                break

    def on_confirm(self):
        """确认选择，保存串口并关闭对话框"""
        self.selected_7mic_port = self.mic_port_combo.currentData()  # 获取真实端口名
        self.save_data = self.save_data_checkbox.isChecked()  # 获取是否保存数据的选项
            
        self.accept()  # 关闭对话框并返回QDialog.Accepted

    def closeEvent(self, event):
        """窗口关闭事件，停止定时器"""
        self.timer.stop()
        super().closeEvent(event)

class MICSerialWorker(QThread):
    data_ready = pyqtSignal(list)  # 用于图像更新
    waveform_ready = pyqtSignal(list)  # 用于波形更新
    fps_ready = pyqtSignal(float)  # 用于帧率更新
    error_signal = pyqtSignal(str)  # 串口错误信号（跨平台错误提示）

    def __init__(self, port, save_data=False, normalization_low=0, normalization_high=3.5):
        super().__init__()
        self.port = port  # 外部传入选择的串口（不再硬编码）
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

        self.save_dir = "5MIC_raw_data"
        self.raw_data_queue = Queue()
        self.writer_thread = None
        self.is_saving = False
        self.csv_file_path = ""  # 存储CSV文件路径，未保存时为空
        self.save_data = save_data  # 接收外部传入的保存选项
        
        # 若选择保存，初始化数据保存相关资源
        if self.save_data:
            self.init_raw_data_saving()
        
        # 帧率计算相关变量
        self.frame_count = 0
        self.last_time = time.time()
        self.fps_update_interval = 1.0  # 每秒更新一次FPS

    def init_raw_data_saving(self):
        """仅在选择保存数据时，初始化CSV文件和写入线程"""
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        # 生成唯一的CSV文件名（含时间戳）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file_path = os.path.join(self.save_dir, f"mic_{timestamp}.csv")
        self.is_saving = True
        
        # 新增：创建CSV文件并写入表头（7个麦克风通道）
        with open(self.csv_file_path, 'w', encoding='utf-8') as f:
            header = ','.join([f'MIC{i+1}' for i in range(5)])  # 表头：MIC1,MIC2,...MIC7
            f.write(header + '\n')
        
        # 启动实时写入线程（守护线程，主程序退出时自动结束）
        self.writer_thread = threading.Thread(target=self.write_raw_data_to_csv, daemon=True)
        self.writer_thread.start()

    def write_raw_data_to_csv(self):
        """仅在选择保存数据时，执行CSV写入逻辑"""
        while self.is_saving:
            try:
                # 阻塞式获取数据，队列空时线程自动挂起（不占CPU）
                parsed_data = self.raw_data_queue.get(block=True, timeout=None)
                
                # 新格式解析：[mic1_f1, mic2_f1, mic3_f1, mic4_f1, mic5_f1, mic1_f2, ...]
                points_per_channel = FRAMES_PER_PACKET
                reshaped_data = np.zeros((points_per_channel, 5), dtype=np.uint16)
                for frame_idx in range(points_per_channel):
                    # 每帧的5个麦克风数据：frame_idx*5 到 frame_idx*5+5
                    frame_data = parsed_data[frame_idx*5 : (frame_idx+1)*5]
                    reshaped_data[frame_idx, :] = frame_data
                
                # 实时追加写入CSV
                with open(self.csv_file_path, 'a', encoding='utf-8') as f:
                    np.savetxt(f, reshaped_data, delimiter=',', fmt='%d')
                    
                self.raw_data_queue.task_done()  # 标记任务完成，避免内存泄漏
                
            except Exception as e:
                # 仅在非主动停止时打印异常
                if self.is_saving:
                    print(f"实时写入CSV异常: {e}")
                break

    def stop(self):
        self.running = False
        self.is_saving = False  # 停止线程循环
        
        # 若开启了保存，唤醒阻塞的写入线程并释放资源
        if self.save_data and self.raw_data_queue.empty():
            self.raw_data_queue.put(np.array([]))  # 放入空数组触发线程唤醒
        
        # 等待写入线程安全结束（超时2秒，防止卡死）
        if self.save_data and self.writer_thread and self.writer_thread.is_alive():
            self.writer_thread.join(timeout=2)
        
        # 仅在开启保存时，打印保存路径
        if self.save_data:
            print(f"实时CSV保存已停止，文件路径：{self.csv_file_path}")

    def run(self):
        try:
            # 跨平台串口连接
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            while self.running:
                try:
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
                                
                                # 检查数据长度是否符合预期
                                expected_length = FRAMES_PER_PACKET * 5
                                if len(parsed) == expected_length or len(parsed) == 900:
                                    if len(parsed) == 900:
                                        parsed = parsed[400:]
                                    # 仅在开启保存时，将数据放入队列
                                    if self.save_data:
                                        self.raw_data_queue.put(parsed.copy())

                                    if self.matrix_flag == 0:
                                        self.matrix_init = parsed.copy()
                                        self.matrix_flag = 1
                                    else:
                                        result = parsed
                                        clipped_result = np.clip(result, self.normalization_low, self.normalization_high)
                                        normalized_result = (clipped_result - self.normalization_low) / (self.normalization_high - self.normalization_low)
                                        
                                        self.waveform_ready.emit(parsed.tolist())
                                        self.data_ready.emit(normalized_result.tolist())
                                    
                                    # 帧率计算
                                    self.frame_count += 1
                                    current_time = time.time()
                                    if current_time - self.last_time >= self.fps_update_interval:
                                        fps = self.frame_count / (current_time - self.last_time)
                                        self.fps_ready.emit(fps)
                                        self.frame_count = 0
                                        self.last_time = current_time

                                self.buffer = self.buffer[end_index + len(self.FRAME_TAIL):]
                                start_index = self.buffer.find(self.FRAME_HEADER)
                            else:
                                break
                except Exception as e:
                    # 捕获读取数据时的异常，但不终止线程
                    print(f"读取5MIC数据时出错: {str(e)}")
                    self.msleep(100)  # 等待一段时间再继续
        except Exception as e:
            # 发送错误信号到主线程，弹出可视化提示
            error_msg = f"5MIC串口通信异常: {str(e)}\nUbuntu用户请检查是否加入dialout组"
            self.error_signal.emit(error_msg)
        finally:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                    print('5MIC串口关闭')
                except OSError as e:
                    print(f"关闭5MIC串口时出错: {str(e)}")
            self.running = False

# ---------------------- 5MIC波形可视化类 ----------------------
class MultiChannelWaveformVisualizer:
    def __init__(self, widget):
        self.layout = widget.addLayout()
        self.plots = []
        self.curves = []
        self.show_data = []
        
        # 定义每个麦克风的位置 (行, 列)
        positions = [
            (0, 2),  # MIC 1
            (0, 0),  # MIC 2
            (1, 1),  # MIC 3
            (2, 2),  # MIC 4
            (2, 0),  # MIC 5
            # (1, 2),  # MIC 6
            # (0, 2),  # MIC 7
        ]
    
        # 为7个麦克风创建独立的波形显示
        for i in range(5):
            plot = pg.PlotItem(title=f"MIC {i+1}")
            plot.setLabels(left='数值', bottom='帧编号')
            plot.showGrid(x=True, y=True)
            plot.setYRange(0, 4095)
            
            # 使用更粗的线条，宽度为2
            curve = plot.plot(pen=pg.mkPen(pg.intColor(i), width=1.0))  # 使用不同颜色区分，线条更粗
            # 显示点数根据帧数动态调整
            show_data = [0] * FRAMES_PER_PACKET * 10  # 显示FRAMES_PER_PACKET*10个点
            
            self.plots.append(plot)
            self.curves.append(curve)
            self.show_data.append(show_data)
            
            # 使用预定义的位置
            row, col = positions[i]
            self.layout.addItem(plot, row, col)

    def update_plot(self, new_row):
        # 检查数据长度是否符合预期
        expected_length = FRAMES_PER_PACKET * 5
        if len(new_row) == expected_length:
            points_per_channel = FRAMES_PER_PACKET
            # 新格式：[mic1_f1, mic2_f1, mic3_f1, mic4_f1, mic5_f1, mic1_f2, mic2_f2, ...]
            for mic in range(5):
                # 按步长5取对应麦克风的所有帧数据
                mic_data = new_row[mic::5]  # 从索引mic开始，每5个取一个
                # 更新显示数据
                self.show_data[mic] = self.show_data[mic][len(mic_data):] + mic_data
                # 更新图形
                x_data = np.arange(len(self.show_data[mic]))
                y_data = np.array(self.show_data[mic])
                self.curves[mic].setData(x_data, y_data)



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.mic_worker = None  # 5MIC串口线程
        self.fps_label = None  # 帧率显示标签
        self.save_data = SAVE_DATA_DEFAULT  # 是否保存数据

        # 1. 先显示串口选择对话框（必须先选串口，再初始化主界面）
        self.select_serial_port()
        # 用户取消选择或无可用串口，直接退出
        if self.selected_7mic_port is None:
            sys.exit(0)

        # 2. 初始化主界面（选择串口成功后才执行）
        self.init_main_ui()

    def select_serial_port(self):
        """显示串口选择对话框，获取用户选择的端口"""
        dialog = SerialSelectionDialog()
        if dialog.exec_() == QDialog.Accepted:
            self.selected_7mic_port = dialog.selected_7mic_port
            self.save_data = dialog.save_data
        else:
            self.selected_7mic_port = None

    def init_main_ui(self):
        """初始化主界面（图像+波形显示）"""
        self.setWindowTitle("5MIC波形显示")
        self.resize(1500, 800)
        # 添加以下行来使窗口最大化显示
        self.showMaximized()

        self.central_widget = QSplitter()
        self.setCentralWidget(self.central_widget)

        # ---------------------- 左侧：5MIC波形显示 ----------------------
        # 左侧总容器（统一黑色背景和边框）
        self.left_container = QWidget()
        self.left_container.setStyleSheet("""
            QWidget {
                background-color: black;
                border: 2px solid black;
            }
        """)
        self.left_layout = QVBoxLayout(self.left_container)
        self.left_layout.setContentsMargins(0, 0, 0, 0)  # 消除内边距
        self.left_layout.setSpacing(2)  # 组件间距2px

        # 1. 左侧：5MIC波形显示
        self.mic_waveform_widget = pg.GraphicsLayoutWidget()
        self.mic_waveform_visualizer = MultiChannelWaveformVisualizer(self.mic_waveform_widget)
        
        # 2. 在GraphicsLayoutWidget中添加FPS显示
        self.fps_label = pg.LabelItem(justify='right')
        self.fps_label.setText("FPS: --", color='green')
        self.mic_waveform_widget.addItem(self.fps_label, row=3, col=0, colspan=3)
        
        self.left_layout.addWidget(self.mic_waveform_widget)
        
        # 根据选择的传感器决定显示哪些组件
        if self.selected_7mic_port:
            self.left_layout.addWidget(self.mic_waveform_widget, stretch=1)
            self.central_widget.addWidget(self.left_container)
            
        # ---------------------- 线程初始化与信号连接 ----------------------
        # 1. 5MIC传感器线程
        if self.selected_7mic_port:
            self.mic_worker = MICSerialWorker(port=self.selected_7mic_port, save_data=self.save_data)
            self.mic_worker.waveform_ready.connect(self.mic_waveform_visualizer.update_plot)
            self.mic_worker.fps_ready.connect(self.update_fps_display)
            self.mic_worker.error_signal.connect(self.show_mic_serial_error)
            self.mic_worker.start()
        

    def update_fps_display(self, fps):
        """更新帧率显示"""
        self.fps_label.setText(f"FPS: {fps:.1f}", color='green')
        
    def show_mic_serial_error(self, error_msg):
        """显示5MIC串口错误提示（可视化弹窗）"""
        QMessageBox.critical(self, "5MIC串口错误", error_msg)
        # 不再关闭整个程序，只停止对应的worker
        if self.mic_worker:
            self.mic_worker.stop()
            self.mic_worker.wait()
            self.mic_worker = None

    def closeEvent(self, event):
        """窗口关闭时释放资源"""
        self.hide()
        if self.mic_worker:
            self.mic_worker.stop()
            self.mic_worker.wait()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec_())