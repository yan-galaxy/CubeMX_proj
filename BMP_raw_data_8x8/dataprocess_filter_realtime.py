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

# 提取第一列数据
channel_0_data = data['Channel_0'].values  # 转换为numpy数组便于处理

# 初始化滤波器处理器
filter_handler = FilterHandler(fs=67.0, high_cutoff=0.5, low_cutoff=0.5, order=4)
filter_handler.initialize_states(channel_0_data[0])

# 模拟实时处理：逐个数据点处理所有操作
filtered_data = []
abs_data = []
lowpass_filtered_data = []

# 添加用于记录处理时间的变量
processing_times = []

for data_point in channel_0_data:
    # 记录开始处理时间
    start_time = time.perf_counter()
    
    # 高通滤波
    filtered_value = filter_handler.apply_high_pass(data_point)
    filtered_data.append(filtered_value)
    
    # 取绝对值
    abs_value = np.abs(filtered_value)
    abs_data.append(abs_value)
    
    # 低通滤波
    lowpass_value = filter_handler.apply_low_pass(abs_value)
    lowpass_filtered_data.append(lowpass_value)
    
    # 记录处理结束时间并计算处理时间
    end_time = time.perf_counter()
    processing_times.append(end_time - start_time)

# 计算平均处理时间
average_processing_time = np.mean(processing_times)
print(f"单个数据点完整处理平均时间: {average_processing_time*1000:.4f} 毫秒")
print(f"每秒可处理数据点数: {1/average_processing_time:.0f} 点/秒")

# 转换为numpy数组便于绘图
filtered_data = np.array(filtered_data)
abs_data = np.array(abs_data)
lowpass_filtered_data = np.array(lowpass_filtered_data)

# 创建可视化图表
plt.figure(figsize=(14, 7))  # 每次都固定这个形状，不能修改！！！

# 绘制原始数据
plt.subplot(4, 1, 1)
plt.plot(channel_0_data, linestyle='-')
plt.title('原始数据波形')
plt.xlabel('样本索引')
plt.ylabel('数值')
plt.grid(True)

# 绘制高通滤波后数据
plt.subplot(4, 1, 2)
plt.plot(filtered_data, linestyle='-', color='r')
plt.title('高通滤波后波形')
plt.xlabel('样本索引')
plt.ylabel('数值')
plt.grid(True)

# 绘制取绝对值后的数据
plt.subplot(4, 1, 3)
plt.plot(abs_data, linestyle='-', color='g')
plt.title('高通滤波后取绝对值')
plt.xlabel('样本索引')
plt.ylabel('数值')
plt.grid(True)

# 绘制低通滤波后的数据
plt.subplot(4, 1, 4)
plt.plot(lowpass_filtered_data, linestyle='-', color='b')
plt.title('低通滤波后波形')
plt.xlabel('样本索引')
plt.ylabel('数值')
plt.grid(True)

plt.tight_layout()
plt.show()
