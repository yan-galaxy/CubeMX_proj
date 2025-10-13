import serial
import time
from serial.tools import list_ports
import sys
from PyQt5.QtWidgets import (QApplication, QDialog, QVBoxLayout, QComboBox, 
                             QPushButton, QMessageBox)
import threading

class SerialSelectionDialog(QDialog):
    """跨平台串口选择对话框（Windows/Ubuntu通用）"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("选择串口")
        self.setFixedSize(300, 150)
        self.selected_port = None

        # 布局初始化
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        # 串口选择下拉框
        self.port_combo = QComboBox()
        self.port_combo.setPlaceholderText("请选择可用串口")
        self.layout.addWidget(self.port_combo)

        # 加载可用串口
        self.load_available_ports()

        # 按钮布局
        self.confirm_btn = QPushButton("确认选择")
        self.cancel_btn = QPushButton("取消")
        self.layout.addWidget(self.confirm_btn)
        self.layout.addWidget(self.cancel_btn)

        # 按钮信号连接
        self.confirm_btn.clicked.connect(self.on_confirm)
        self.cancel_btn.clicked.connect(self.reject)

    def load_available_ports(self):
        """加载当前系统所有可用串口"""
        self.port_combo.clear()
        available_ports = list(list_ports.comports())
        
        if not available_ports:
            QMessageBox.warning(self, "警告", "未检测到可用串口！\n请检查设备连接后重试")
            self.confirm_btn.setEnabled(False)
            return
        
        # 添加可用串口到下拉框
        for port in available_ports:
            port_info = f"{port.device} - {port.description}"
            self.port_combo.addItem(port_info, port.device)

        # 默认选择第一个串口
        self.port_combo.setCurrentIndex(0)

    def on_confirm(self):
        """确认选择，保存串口并关闭对话框"""
        self.selected_port = self.port_combo.currentData()
        self.accept()

def select_serial_port():
    """显示串口选择对话框，获取用户选择的端口"""
    app = QApplication(sys.argv)  # 创建临时的 QApplication 实例
    dialog = SerialSelectionDialog()
    if dialog.exec_() == QDialog.Accepted:
        selected_port = dialog.selected_port
        app.quit()  # 退出临时的 QApplication
        return selected_port
    else:
        app.quit()  # 退出临时的 QApplication
        return None

# 配置串口参数
baudrate = 115200  # 根据实际情况修改为你的波特率

# 显示串口选择对话框
port = select_serial_port()
if not port:
    print("未选择串口或无可用串口")
    sys.exit(1)

# 初始化数据速率计算变量
start_time = None
total_bytes_received = 0

def serial_monitor():
    global start_time, total_bytes_received
    
    try:
        # 打开串口
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"成功打开串口 {port}")

        # 记录开始时间
        start_time = time.time()
        last_print_time = start_time

        while True:
            if ser.in_waiting > 0:
                # 读取串口数据
                data = ser.read(ser.in_waiting)  # 读取所有可用的字节
                total_bytes_received += len(data)  # 更新接收到的总字节数

            # 每秒计算并显示一次数据速率
            current_time = time.time()
            elapsed_time = current_time - last_print_time
            if elapsed_time >= 1.0:  # 每秒计算一次数据速率
                # 计算速率
                bytes_per_second = total_bytes_received / elapsed_time
                bits_per_second = bytes_per_second * 8
                
                # 转换为 Mbps 和 MB/s
                mbps = bits_per_second / 1_000_000
                mbytes_per_second = bytes_per_second / 1_000_000
                
                # 显示速率
                print(f"数据接收速率: {mbps:.4f} Mbit/s ({mbytes_per_second:.4f} MBytes/s)")

                # 重置计数器和时间
                last_print_time = current_time
                total_bytes_received = 0

            # time.sleep(0.002)  # 添加一个小延迟以避免CPU占用过高

    except serial.SerialException as e:
        print(f"无法打开串口 {port}: {e}")
    except KeyboardInterrupt:
        print("程序被用户中断")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"关闭串口 {port}")

# 运行串口监控
serial_monitor()