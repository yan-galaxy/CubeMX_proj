import serial
import time

# 配置串口参数
port = 'COM14'  # 根据实际情况修改为你的串口号，例如 '/dev/ttyUSB0' 在Linux或macOS上
baudrate = 115200  # 根据实际情况修改为你的波特率

# 初始化数据速率计算变量
start_time = None
total_bytes_received = 0

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

        time.sleep(0.002)  # 添加一个小延迟以避免CPU占用过高

except serial.SerialException as e:
    print(f"无法打开串口 {port}: {e}")
except KeyboardInterrupt:
    print("程序被用户中断")
finally:
    if ser.is_open:
        ser.close()
        print(f"关闭串口 {port}")