import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy import signal
import tkinter as tk
from tkinter import filedialog
import sys

def select_files():
    """通过可视化界面选择数据文件"""
    # 创建一个隐藏的根窗口
    root = tk.Tk()
    root.withdraw()  # 隐藏主窗口
    root.attributes('-topmost', True)  # 确保对话框在最前面

    print("请选择坤维传感器数据文件 (kunwei_data_*.csv):")
    kunwei_data_file = filedialog.askopenfilename(
        title="选择坤维传感器数据文件",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
    )
    
    if not kunwei_data_file:
        print("未选择坤维传感器数据文件，程序退出。")
        sys.exit()

    print("请选择MIC数据文件 (mic_*.csv):")
    mic_data_file = filedialog.askopenfilename(
        title="选择MIC数据文件",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
    )
    
    if not mic_data_file:
        print("未选择MIC数据文件，程序退出。")
        sys.exit()
    
    root.destroy()
    return kunwei_data_file, mic_data_file

# 通过可视化界面选择数据文件
kunwei_data_file, mic_data_file = select_files()

# 分块读取坤维传感器数据并提取Fz列 (Fz是第三列，索引为2)
chunk_size = 10000  # 每次读取10000行
fz_data_list = []
for chunk in pd.read_csv(kunwei_data_file, chunksize=chunk_size):
    fz_data_list.append(chunk['Fz'].values)
fz_data = np.concatenate(fz_data_list)

# 对kunwei数据进行高通滤波
# 采样率: 1000Hz，截止频率: 1Hz
fs_kunwei = 1000  # 采样率
cutoff_kunwei = 0.2  # 截止频率
nyquist_kunwei = 0.5 * fs_kunwei
normal_cutoff_kunwei = cutoff_kunwei / nyquist_kunwei
# 设计4阶贝塞尔高通滤波器 bessel butter
b_kunwei, a_kunwei = signal.bessel(4, normal_cutoff_kunwei, btype='high', analog=False)
# 应用滤波器
fz_filtered = signal.filtfilt(b_kunwei, a_kunwei, fz_data)

# 分块读取MIC数据并提取MIC4列 (MIC4是第四列，索引为3)
mic4_data_list = []
for chunk in pd.read_csv(mic_data_file, chunksize=chunk_size):
    mic4_data_list.append(chunk['MIC4'].values)
mic4_data = np.concatenate(mic4_data_list).astype(np.float32)  # 转换为float32

# 对MIC4数据进行高通滤波
# 采样率: 10kHz，截止频率: 1Hz
fs_mic = 10000  # 采样率
cutoff_mic = 1  # 截止频率
nyquist_mic = 0.5 * fs_mic
normal_cutoff_mic = cutoff_mic / nyquist_mic
# 设计4阶贝塞尔高通滤波器
b_mic, a_mic = signal.bessel(4, normal_cutoff_mic, btype='high', analog=False)
# 应用滤波器
mic4_filtered_high = signal.filtfilt(b_mic, a_mic, mic4_data)

# 对MIC4数据再进行低通滤波，截止频率750Hz
cutoff_mic_low = 750  # 低通截止频率
nyquist_mic_low = 0.5 * fs_mic
normal_cutoff_mic_low = cutoff_mic_low / nyquist_mic_low
# 设计8阶贝塞尔低通滤波器 (使用SOS格式提高数值稳定性)
sos_mic_low = signal.bessel(8, normal_cutoff_mic_low, btype='low', analog=False, output='sos')
# 应用滤波器
mic4_filtered_low = signal.sosfiltfilt(sos_mic_low, mic4_filtered_high)

# 创建时间轴（以秒为单位）
fz_time = np.arange(len(fz_data)) / fs_kunwei  # 转换为秒
mic4_time = np.arange(len(mic4_data)) / fs_mic  # 转换为秒

# # 为了减少内存使用，对时间轴进行下采样
# downsample_factor = 100
# fz_time_ds = fz_time[::downsample_factor]
# mic4_time_ds = mic4_time[::downsample_factor]

# 创建图形和子图
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(13, 7))
fig.suptitle('Fz Force and MIC4 Data Waveform (Original and High-pass Filtered)', fontsize=16)

# 绘制高通滤波后的Fz力数据波形
line1, = ax1.plot(fz_time, fz_filtered, 'b-', linewidth=0.8)
ax1.set_title('High-pass Filtered Fz Force Data')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Fz (N)')
ax1.grid(True)

# 绘制原始MIC4数据波形
line2, = ax2.plot(mic4_time, mic4_data, 'r-', linewidth=0.8)
ax2.set_title('Original MIC4 Data')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Amplitude')
ax2.grid(True)

# 绘制高通、低通滤波后的MIC4数据波形
line3, = ax3.plot(mic4_time, mic4_filtered_high, 'g-', linewidth=0.8)
ax3.set_title('High-pass Filtered MIC4 Data')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Amplitude')
ax3.grid(True)

# 绘制高通、低通滤波后的MIC4数据波形
# line5, = ax4.plot(mic4_time, mic4_filtered_high, 'r-', linewidth=0.8) #低通滤波前对比
line4, = ax4.plot(mic4_time, mic4_filtered_low, 'c-', linewidth=0.8)
ax4.set_title('High-pass and Low-pass Filtered MIC4 Data')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Amplitude')
ax4.grid(True)

# 设置布局
plt.tight_layout()
plt.show()