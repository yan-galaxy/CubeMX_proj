import serial
import struct
import time
import csv

# 不准改我的任何注释！！！不准删！！！
class KunweiDataReader:
    def __init__(self, port, baudrate=460800):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.buffer = b''  # 数据缓冲区
        # 帧头和帧尾定义
        self.FRAME_HEADER = b'\x48\xAA'
        self.FRAME_TAIL = b'\x0D\x0A'
        # 启动前需发送的十六进制数据（48 AA 0D 0A）
        self.INIT_SEND_DATA = b'\x48\xAA\x0D\x0A'
        
        # 定时打印相关变量
        self.print_interval = 0.5  # 打印间隔（秒）
        self.last_print_time = time.time()
        
        # 丢包率计算相关变量
        self.expected_frames_per_interval = 500  # 500ms内理论帧数(1kHz)
        self.received_frames = 0  # 实际接收帧数
        self.last_frame = None  # 保存最后一帧数据用于定时打印
        
        # CSV数据保存相关
        self.save_data = True  # 是否保存数据到CSV
        self.csv_filename = f"kunwei_data_{time.strftime('%Y%m%d_%H%M%S')}.csv"  # 带时间戳的文件名
        self.csv_file = None
        self.csv_writer = None
            
    def start(self):
        """启动数据读取线程"""
        self.running = True
        self.run()
        
    def stop(self):
        """停止数据读取"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            print('串口已关闭')
            
    def parse_frame(self, frame):
        """解析单帧数据为浮点数"""
        try:
            # 提取有效载荷（去掉帧头和帧尾）
            payload = frame[len(self.FRAME_HEADER):-len(self.FRAME_TAIL)]
            
            # 检查有效载荷长度是否符合要求（6个浮点数，每个4字节，共24字节）
            if len(payload) != 24:
                return None, f"有效载荷长度错误，应为24字节，实际为{len(payload)}字节"
            
            components = []
            for i in range(6):  # 6个分量
                # 提取4字节
                start = i * 4
                comp_bytes = payload[start:start+4]
                # 调整字节顺序（低字节在前 → 高字节在前）
                reversed_bytes = comp_bytes[::-1]
                # 解析为浮点数
                comp_float = struct.unpack('>f', reversed_bytes)[0]
                components.append(comp_float)
                
            return components, "解析成功"
            
        except Exception as e:
            return None, f"解析错误: {str(e)}"
    
    def run(self):
        try:
            # 打开串口
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"串口 {self.port} 已打开，波特率: {self.baudrate}，等待数据...")
            # 启动接收前发送指定十六进制数据（48 AA 0D 0A）
            self.ser.write(self.INIT_SEND_DATA)
            print(f"已发送启动初始化数据: {self.INIT_SEND_DATA.hex().upper()}（十六进制）")
            
            print(f"每{self.print_interval*1000}ms打印一次数据，计算丢包率...")
            self.last_print_time = time.time()
            
            # 初始化CSV文件（如果需要保存）- 表头新增“Timestamp(ns)”列（首列）
            if self.save_data:
                self.csv_file = open(self.csv_filename, 'w', newline='')
                self.csv_writer = csv.writer(self.csv_file)
                # 表头顺序：时间戳(ns)、Fx、Fy、Fz、Mx、My、Mz
                self.csv_writer.writerow(['Timestamp(ns)', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])  
                print(f"数据将保存到当前目录下的 {self.csv_filename} 文件")
            
            while self.running:
                current_time = time.time()
                
                # 检查是否到了打印时间
                if current_time - self.last_print_time >= self.print_interval:
                    # 计算丢包率
                    packet_loss_rate = (1 - self.received_frames / self.expected_frames_per_interval) * 100
                    
                    # 打印数据
                    if self.last_frame:
                        Fx, Fy, Fz, Mx, My, Mz = self.last_frame
                        print(f"\n{self.print_interval*1000}ms数据统计:")
                        print(f"  接收帧数: {self.received_frames}/{self.expected_frames_per_interval}")
                        print(f"  丢包率: {packet_loss_rate:.2f}%")
                        print(f"  最新数据:")
                        print(f"    Fx: {Fx:.6f}, Fy: {Fy:.6f}, Fz: {Fz:.6f}")
                        print(f"    Mx: {Mx:.6f}, My: {My:.6f}, Mz: {Mz:.6f}")
                        print("----------------------------------------")
                    else:
                        print(f"\n{self.print_interval*1000}ms内未收到有效数据")
                        print(f"  接收帧数: 0/{self.expected_frames_per_interval}")
                        print(f"  丢包率: 100.00%")
                        print("----------------------------------------")
                    
                    # 重置计数器和时间
                    self.received_frames = 0
                    self.last_print_time = current_time
                
                if self.ser.in_waiting > 0:
                    # 读取所有可用数据
                    data = self.ser.read(self.ser.in_waiting)
                    self.buffer += data
                    
                    # 在缓冲区中查找帧头
                    start_index = self.buffer.find(self.FRAME_HEADER)
                    while start_index != -1:
                        # 查找帧尾
                        end_index = self.buffer.find(self.FRAME_TAIL, start_index + len(self.FRAME_HEADER))
                        
                        if end_index != -1:
                            # 提取完整帧
                            frame = self.buffer[start_index:end_index + len(self.FRAME_TAIL)]
                            
                            # 解析帧数据
                            components, msg = self.parse_frame(frame)
                            if components:
                                # 获取ns级时间戳（使用perf_counter_ns，系统级高精度时间）
                                ns_timestamp = time.perf_counter_ns()
                                # 保存最后一帧数据（仅原6个分量，不影响打印逻辑）
                                self.last_frame = components
                                # 递增接收帧数计数器
                                self.received_frames += 1
                                # 写入CSV文件 - 拼接“时间戳+6个分量”，时间戳作为首列
                                if self.save_data and self.csv_writer:
                                    self.csv_writer.writerow([ns_timestamp] + components)

                                # 原本的即时打印方式，已注释
                                # Fx, Fy, Fz, Mx, My, Mz = components
                                # print(f"原始数据:")
                                # print(f"  Fx: {Fx:.6f}, Fy: {Fy:.6f}, Fz: {Fz:.6f}")
                                # print(f"  Mx: {Mx:.6f}, My: {My:.6f}, Mz: {Mz:.6f}")
                                # print("----------------------------------------")
                            else:
                                # 解析失败，仅在调试时取消注释打印
                                # print(f"帧解析失败: {msg}")
                                # print(f"错误帧数据: {frame.hex().upper()}")
                                pass
                            
                            # 移除已处理的帧
                            self.buffer = self.buffer[end_index + len(self.FRAME_TAIL):]
                            start_index = self.buffer.find(self.FRAME_HEADER)
                        else:
                            # 未找到帧尾，保留剩余缓冲区内容
                            self.buffer = self.buffer[start_index:]
                            break
                            
        except Exception as e:
            print(f"串口通信异常: {str(e)}")
            print("Ubuntu用户请检查是否加入dialout组")
        finally:
            # 关闭CSV文件
            if self.csv_file:
                self.csv_file.close()
                print(f"CSV文件 {self.csv_filename} 已成功关闭，数据已保存")
            self.stop()


# 使用示例
if __name__ == "__main__":
    # 根据实际端口修改
    port = "COM16"  # Windows
    # port = "/dev/ttyUSB0"  # Linux
    
    reader = KunweiDataReader(port, baudrate=460800)
    try:
        reader.start()
    except KeyboardInterrupt:
        print("程序被用户中断")
        reader.stop()