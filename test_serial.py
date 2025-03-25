import serial
import time

# 配置串口参数
port = 'COM9'  # 根据实际情况修改为你的串口号，例如 '/dev/ttyUSB0' 在Linux或macOS上
baudrate = 115200  # 根据实际情况修改为你的波特率

# 定义帧头和帧尾
FRAME_HEADER = b'\x55\xAA\xBB\xCC'
FRAME_TAIL = b'\xAA\x55\x66\x77'

# 缓冲区用于存储接收到的数据
buffer = bytearray()

# 初始化帧速率计算变量
start_time = None
frame_count = 0
total_bytes_received = 0

# 存储解析后的数据
parsed_data = []

try:
    # 打开串口
    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"成功打开串口 {port}")

    # 记录开始时间
    start_time = time.time()

    while True:
        if ser.in_waiting > 0:
            # 读取串口数据
            # data = ser.readline().decode('utf-8').rstrip()
            data = ser.read(ser.in_waiting)  # 读取所有可用的字节
            # hex_data = data.hex()  # 将字节数据转换为十六进制字符串
            # print(f"接收到的数据: {data}")
            # print(f"接收到的二进制数据: {data}")
            # print(f"接收到的二进制数据 (十六进制): {hex_data}")
            buffer.extend(data)  # 将新数据添加到缓冲区
            total_bytes_received += len(data)  # 更新接收到的总字节数

            # 检查缓冲区中是否有完整的帧
            start_index = buffer.find(FRAME_HEADER)
            end_index = buffer.find(FRAME_TAIL, start_index + len(FRAME_HEADER))
            
            while start_index != -1 and end_index != -1:
                # 提取完整的帧
                frame = buffer[start_index:end_index + len(FRAME_TAIL)]
                # 打印帧内容（去掉帧头和帧尾）
                payload = frame[len(FRAME_HEADER):-len(FRAME_TAIL)]
                hex_payload = payload.hex()
                # print(f"接收到的完整帧 (十六进制): {hex_payload}")

                # 解析16位数据
                for i in range(0, len(payload), 2):
                    low_byte = payload[i]
                    high_byte = payload[i + 1]
                    value = low_byte | (high_byte << 8)
                    parsed_data.append(value)

                # 增加帧计数器
                frame_count += 1

                # 移除已处理的帧
                buffer = buffer[end_index + len(FRAME_TAIL):]

                # 继续查找下一个帧
                start_index = buffer.find(FRAME_HEADER)
                end_index = buffer.find(FRAME_TAIL, start_index + len(FRAME_HEADER))

        # 计算帧速率
        current_time = time.time()
        elapsed_time = current_time - start_time
        if elapsed_time >= 0.2:  # 每秒计算一次帧速率
            frames_per_second = frame_count / elapsed_time *10
            bytes_per_second = total_bytes_received / elapsed_time / 1024.0
            print(f"接收到的完整帧速率: {frames_per_second:.2f} 帧/秒")
            print(f"接收到的数据速率: {bytes_per_second:.2f} Kbtyes/秒")

            # # 打印解析后的数据
            # if parsed_data:
            #     for i in range(0, min(100, len(parsed_data)), 10):
            #         # 按列打印解析后的数据
            #         column_data = parsed_data[i:i+10]
            #         print(f"第{i//10+1}列数据: {', '.join(f'{value:.2f}' for value in column_data)}")
            
            if parsed_data:
                Rcode = 2523
                # print(f"解析后的数据: {parsed_data}")
                # print(f"解析后的100个数据: {parsed_data[:100]}")
                for i in range(0, min(100, len(parsed_data)), 10):
                    # 对每个数进行运算 锯齿波整个时间是43.0us 计数周期是5ns
                    vout_data = [ (43.0-(value*5/1000))/43.0*3.3 for value in parsed_data[i:i+10]]
                    # 对 vout_data 中的每个值进行额外的运算（例如加上0.5）
                    Rsensor_data = [0.2973*Rcode/(value-0.2973) for value in vout_data]
                    # print(f"第{i//10+1}列电压: {vout_data}")

                    # print(f"第{i//10+1}列电阻: {Rsensor_data}")
                    # print(f"第{i//10+1}列电阻: {[f'{value:.2f}' for value in Rsensor_data]}")
                    
                    # # 显示换算后的电阻值（部分会显示为负数，有待查bug）
                    # print(f"第{i//10+1}列电阻: {', '.join(f'{value:5.2f}' for value in Rsensor_data)}")
                    # 修改后代码：
                    print(f"第{(i//10+1):02d}列原始数据: {', '.join(f'{value:5.2f}' for value in parsed_data[i:i+10])}")
                parsed_data.clear()  # 清空解析后的数据列表

            # 重置计数器和时间
            start_time = current_time
            frame_count = 0
            total_bytes_received = 0


        # time.sleep(0.01)  # 添加一个小延迟以避免CPU占用过高

except serial.SerialException as e:
    print(f"无法打开串口 {port}: {e}")
except KeyboardInterrupt:
    print("程序被用户中断")
finally:
    if ser.is_open:
        ser.close()
        print(f"关闭串口 {port}")