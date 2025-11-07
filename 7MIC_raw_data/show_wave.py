import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy import signal

# 读取数据文件
kunwei_data_file = r"c:\Users\galaxy\Desktop\CubeMX_proj\7MIC_raw_data\large_file\kunwei_data_20251107_173727_标定循环压_速度10000_一次性_z轴步进10_200-10_增加间隔激励.csv"
mic_data_file = r"c:\Users\galaxy\Desktop\CubeMX_proj\7MIC_raw_data\large_file\mic_20251107_173727_标定循环压_速度10000_一次性_z轴步进10_200-10_增加间隔激励.csv"

# 读取坤维传感器数据 (Fz是第三列，索引为2)
kunwei_df = pd.read_csv(kunwei_data_file)
fz_data = kunwei_df['Fz'].values

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

# 读取MIC数据 (MIC4是第四列，索引为3)
mic_df = pd.read_csv(mic_data_file)
mic4_data = mic_df['MIC4'].values.astype(np.float32)  # 转换为float32

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
# 设计4阶贝塞尔低通滤波器
b_mic_low, a_mic_low = signal.bessel(8, normal_cutoff_mic_low, btype='low', analog=False)
# 应用滤波器
mic4_filtered_low = signal.filtfilt(b_mic_low, a_mic_low, mic4_filtered_high)


# 创建时间轴（以秒为单位）
fz_time = np.arange(len(fz_data)) / fs_kunwei  # 转换为秒
mic4_time = np.arange(len(mic4_data)) / fs_mic  # 转换为秒

# 为了减少内存使用，对时间轴进行下采样
downsample_factor = 100
fz_time_ds = fz_time[::downsample_factor]
mic4_time_ds = mic4_time[::downsample_factor]

# 创建图形和子图
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(13, 7))
fig.suptitle('Fz Force and MIC4 Data Waveform (Original and High-pass Filtered)', fontsize=16)

# # 绘制原始Fz力数据波形
# line1, = ax1.plot(fz_time_ds, fz_data[::downsample_factor], 'b-', linewidth=0.8)
# ax1.set_title('Original Fz Force Data')
# ax1.set_xlabel('Time (s)')
# ax1.set_ylabel('Fz (N)')
# ax1.grid(True)

# 绘制高通滤波后的Fz力数据波形
line1, = ax1.plot(fz_time_ds, fz_filtered[::downsample_factor], 'b-', linewidth=0.8)
ax1.set_title('High-pass Filtered Fz Force Data')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Fz (N)')
ax1.grid(True)

# 绘制原始MIC4数据波形
line2, = ax2.plot(mic4_time_ds, mic4_data[::downsample_factor], 'r-', linewidth=0.8)
ax2.set_title('Original MIC4 Data')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Amplitude')
ax2.grid(True)

# 绘制高通、低通滤波后的MIC4数据波形
line3, = ax3.plot(mic4_time_ds, mic4_filtered_high[::downsample_factor], 'g-', linewidth=0.8)
ax3.set_title('High-pass Filtered MIC4 Data')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Amplitude')
ax3.grid(True)

# 绘制高通、低通滤波后的MIC4数据波形
# line5, = ax4.plot(mic4_time_ds, mic4_filtered_high[::downsample_factor], 'r-', linewidth=0.8) #低通滤波前对比
line4, = ax4.plot(mic4_time_ds, mic4_filtered_low[::downsample_factor], 'c-', linewidth=0.8)
ax4.set_title('High-pass and Low-pass Filtered MIC4 Data')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Amplitude')
ax4.grid(True)

# 设置布局
plt.tight_layout()
plt.show()