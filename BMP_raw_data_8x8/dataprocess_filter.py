import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy import signal

# 每次修改时不准删除任何原有注释！！！！
plt.rcParams["font.family"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams['axes.unicode_minus'] = False

# 读取CSV文件
file_path = 'raw_data_8x8_20250903_163708.csv' # raw_data_8x8_20250903_145631 raw_data_8x8_20250903_163708
# 确保在当前脚本目录下查找文件
script_dir = os.path.dirname(os.path.abspath(__file__))
full_file_path = os.path.join(script_dir, file_path)

data = pd.read_csv(full_file_path)

# 提取第一列数据
channel_0_data = data['Channel_6']

# 设计高通滤波器 (截止频率设置为10Hz，采样频率假设为1000Hz)
fs = 68.0  # 采样频率
cutoff_high = 0.5  # 高通截止频率
nyquist = 0.5 * fs
normal_cutoff_high = cutoff_high / nyquist
b_high, a_high = signal.butter(4, normal_cutoff_high, btype='high', analog=False)

# 应用高通滤波器
highpass_data = signal.filtfilt(b_high, a_high, channel_0_data)

# 对高通滤波后的信号取绝对值
abs_data = np.abs(highpass_data)

# 设计低通滤波器
cutoff_low = 0.5  # 低通截止频率
normal_cutoff_low = cutoff_low / nyquist
b_low, a_low = signal.butter(4, normal_cutoff_low, btype='low', analog=False)

# 应用低通滤波器到绝对值信号
lowpass_data = signal.filtfilt(b_low, a_low, abs_data)

# 创建可视化图表 - 显示所有处理步骤
plt.figure(figsize=(14, 7))  # 每次都固定这个形状，不能修改！！！

# 原始数据
plt.subplot(4, 1, 1)
plt.plot(channel_0_data, linestyle='-')
plt.title('原始信号 Channel 0 Data')
plt.xlabel('Sample Index')
plt.ylabel('Value')
plt.grid(True)

# 高通滤波后数据
plt.subplot(4, 1, 2)
plt.plot(highpass_data, linestyle='-')
plt.title('高通滤波后 Channel 0 Data (截止频率: 0.5Hz)')
plt.xlabel('Sample Index')
plt.ylabel('Value')
plt.grid(True)

# 取绝对值后数据
plt.subplot(4, 1, 3)
plt.plot(abs_data, linestyle='-')
plt.title('取绝对值后数据')
plt.xlabel('Sample Index')
plt.ylabel('Value')
plt.grid(True)

# 低通滤波后数据
plt.subplot(4, 1, 4)
plt.plot(lowpass_data, linestyle='-')
plt.title('低通滤波后 Channel 0 Data (截止频率: 0.5Hz)')
plt.xlabel('Sample Index')
plt.ylabel('Value')
plt.grid(True)

plt.tight_layout()
plt.show()


# 打印一些基本统计数据
print(f"数据点数量: {len(channel_0_data)}")
print(f"原始数据 - 最大值: {channel_0_data.max()}")
print(f"原始数据 - 最小值: {channel_0_data.min()}")
print(f"原始数据 - 平均值: {channel_0_data.mean():.6f}")
print(f"原始数据 - 标准差: {channel_0_data.std():.6f}")
print("")
print(f"高通滤波后数据 - 最大值: {highpass_data.max()}")
print(f"高通滤波后数据 - 最小值: {highpass_data.min()}")
print(f"高通滤波后数据 - 平均值: {highpass_data.mean():.6f}")
print(f"高通滤波后数据 - 标准差: {highpass_data.std():.6f}")
print("")
print(f"取绝对值后数据 - 最大值: {abs_data.max()}")
print(f"取绝对值后数据 - 最小值: {abs_data.min()}")
print(f"取绝对值后数据 - 平均值: {abs_data.mean():.6f}")
print(f"取绝对值后数据 - 标准差: {abs_data.std():.6f}")
print("")
print(f"低通滤波后数据 - 最大值: {lowpass_data.max()}")
print(f"低通滤波后数据 - 最小值: {lowpass_data.min()}")
print(f"低通滤波后数据 - 平均值: {lowpass_data.mean():.6f}")
print(f"低通滤波后数据 - 标准差: {lowpass_data.std():.6f}")