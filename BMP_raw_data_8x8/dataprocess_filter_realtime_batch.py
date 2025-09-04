import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy import signal  # 导入信号处理模块
import time  # 添加时间模块用于性能测量

# 每次修改时不准不准删除任何原有注释！！！！
plt.rcParams["font.family"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams['axes.unicode_minus'] = False

class FilterHandler:
    """滤波器处理类，封装高通和低通滤波的相关方法"""
    
    def __init__(self, fs, high_cutoff, low_cutoff, order=4):
        """
        初始化滤波器参数
        
        参数:
        fs: 采样频率
        high_cutoff: 高通滤波器截止频率
        low_cutoff: 低通滤波器截止频率
        order: 滤波器阶数，默认为4
        """
        self.fs = fs
        self.nyquist = 0.5 * fs
        
        # 设计高通滤波器
        normal_high_cutoff = high_cutoff / self.nyquist
        self.b_high, self.a_high = signal.butter(order, normal_high_cutoff, 
                                                btype='high', analog=False)
        
        # 设计低通滤波器
        normal_low_cutoff = low_cutoff / self.nyquist
        self.b_low, self.a_low = signal.butter(order, normal_low_cutoff, 
                                              btype='low', analog=False)
        
        # 初始化滤波器状态变量
        self.zi_high = None
        self.zi_low = None
    
    def initialize_states(self, initial_value):
        """初始化滤波器状态，使用第一个数据点作为初值"""
        self.zi_high = signal.lfilter_zi(self.b_high, self.a_high) * initial_value
        self.zi_low = signal.lfilter_zi(self.b_low, self.a_low) * 0
    
    def apply_high_pass(self, data_point):
        """对单个数据点应用高通滤波"""
        filtered_point, self.zi_high = signal.lfilter(self.b_high, self.a_high, 
                                                    [data_point], zi=self.zi_high)
        return filtered_point[0]
    
    def apply_low_pass(self, data_point):
        """对单个数据点应用低通滤波"""
        filtered_point, self.zi_low = signal.lfilter(self.b_low, self.a_low, 
                                                   [data_point], zi=self.zi_low)
        return filtered_point[0]

# 读取CSV文件
file_path = 'raw_data_8x8_20250903_145631.csv'
# 确保在当前脚本目录下查找文件
script_dir = os.path.dirname(os.path.abspath(__file__))
full_file_path = os.path.join(script_dir, file_path)

data = pd.read_csv(full_file_path)

# 生成所有通道的列名（0~63）
channel_columns = [f'Channel_{i}' for i in range(64)]

# 获取数据点总数（假设所有通道数据长度相同）
num_data_points = len(data[channel_columns[0]])

# 初始化存储结构：字典的键为通道名，值为处理结果
results = {
    'raw': {channel: [] for channel in channel_columns},
    'high_pass': {channel: [] for channel in channel_columns},
    'abs': {channel: [] for channel in channel_columns},
    'low_pass': {channel: [] for channel in channel_columns}
}

# 为每个通道创建滤波器实例
filter_handlers = {
    channel: FilterHandler(fs=66.7, high_cutoff=0.2, low_cutoff=10.0, order=1)
    for channel in channel_columns
}

# 初始化所有滤波器（使用各通道第一个数据点）
for channel in channel_columns:
    initial_value = data[channel].values[0]
    filter_handlers[channel].initialize_states(initial_value)
    # 保存第一个原始数据点
    results['raw'][channel].append(initial_value)

# 新增：存储"单次循环（处理所有通道一个数据点）"的时间
full_loop_processing_times = []

# 记录总处理时间
total_start_time = time.perf_counter()

# 按数据点索引循环（每次处理所有通道的同一个数据点）
for point_idx in range(num_data_points):
    # 记录单次循环开始时间（处理所有通道的当前数据点）
    loop_start_time = time.perf_counter()
    
    # 遍历所有通道，处理当前数据点
    for channel in channel_columns:
        # 获取当前通道的当前数据点
        data_point = data[channel].values[point_idx]
        
        # 高通滤波
        hp_val = filter_handlers[channel].apply_high_pass(data_point)
        
        # 不处理
        abs_val = hp_val
        # # 取绝对值
        # abs_val = np.abs(hp_val)
        # # 只保留正数部分，负数置为0
        # abs_val = np.maximum(hp_val, 0)
        
        # 低通滤波
        lp_val = filter_handlers[channel].apply_low_pass(abs_val)
        
        # 保存处理结果（跳过第一个点，因为初始化时已保存）
        if point_idx > 0:
            results['raw'][channel].append(data_point)
        results['high_pass'][channel].append(hp_val)
        results['abs'][channel].append(abs_val)
        results['low_pass'][channel].append(lp_val)
    
    # 记录单次循环结束时间并计算耗时（跳过第一个点的初始化循环）
    if point_idx > 0:
        loop_end_time = time.perf_counter()
        loop_processing_time = loop_end_time - loop_start_time
        full_loop_processing_times.append(loop_processing_time)
    
# 计算总处理时间
total_end_time = time.perf_counter()
total_time = total_end_time - total_start_time

# 转换结果为numpy数组
for channel in channel_columns:
    results['raw'][channel] = np.array(results['raw'][channel])
    results['high_pass'][channel] = np.array(results['high_pass'][channel])
    results['abs'][channel] = np.array(results['abs'][channel])
    results['low_pass'][channel] = np.array(results['low_pass'][channel])

# 计算关键时间指标
total_data_points = num_data_points * 64  # 总数据点数（通道数×每个通道数据点）
average_full_loop_time = np.mean(full_loop_processing_times)  # 单次循环（全部通道一个数据点）平均时间

# 输出时间统计结果
print(f"总数据点数: {total_data_points} 点")
print(f"64个通道总处理时间: {total_time:.4f} 秒")
print(f"单次循环（处理所有64个通道的一个数据点）平均时间: {average_full_loop_time*1000:.6f} 毫秒")
print(f"整体处理速率: {total_data_points/total_time:.0f} 点/秒")

# 可视化部分：展示前4个通道作为示例
plt.figure(figsize=(14, 7))  # 每次都固定这个形状，不能修改！！！

# 选择前4个通道进行展示
display_channels = channel_columns[:4]
for i, channel in enumerate(display_channels, 1):
    # 原始数据
    plt.subplot(4, 4, i)
    plt.plot(results['raw'][channel], linestyle='-', linewidth=0.8)
    plt.title(f'{channel} 原始数据')
    plt.grid(True, alpha=0.3)
    plt.xticks([])  # 隐藏x轴刻度，避免拥挤
    
    # 高通滤波后
    plt.subplot(4, 4, i+4)
    plt.plot(results['high_pass'][channel], linestyle='-', color='r', linewidth=0.8)
    plt.title('高通滤波后')
    plt.grid(True, alpha=0.3)
    plt.xticks([])
    
    # 绝对值后
    plt.subplot(4, 4, i+8)
    plt.plot(results['abs'][channel], linestyle='-', color='g', linewidth=0.8)
    plt.title('取绝对值后')
    plt.grid(True, alpha=0.3)
    plt.xticks([])
    
    # 低通滤波后
    plt.subplot(4, 4, i+12)
    plt.plot(results['low_pass'][channel], linestyle='-', color='b', linewidth=0.8)
    plt.title('低通滤波后')
    plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()
