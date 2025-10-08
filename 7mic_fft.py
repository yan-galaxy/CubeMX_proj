import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
from scipy import signal
import os
import tkinter as tk
from tkinter import filedialog

# 不准改我的注释！！！不准删！！！
plt.rcParams["font.family"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

def read_7mic_csv_data(csv_path):
    """
    读取7MIC CSV文件数据
    每列代表一个麦克风通道
    """
    try:
        data = pd.read_csv(csv_path)
        print(f"成功加载数据，形状: {data.shape}")
        print(f"列名: {list(data.columns)}")
        return data
    except Exception as e:
        print(f"读取CSV文件失败: {e}")
        return None

def apply_filter(channel_data, filter_type, cutoff_freq, sampling_rate, order=2):
    """
    对通道数据应用滤波器
    
    参数:
    channel_data: 要滤波的数据
    filter_type: 滤波器类型 ('lowpass', 'highpass')
    cutoff_freq: 截止频率 (Hz)
    sampling_rate: 采样率 (Hz)
    order: 滤波器阶数
    
    返回:
    filtered_data: 滤波后的数据
    """
    # 归一化截止频率 (0到1之间，1对应奈奎斯特频率)
    nyquist = 0.5 * sampling_rate
    normalized_cutoff = cutoff_freq / nyquist
    
    # 设计滤波器
    if filter_type == 'lowpass':
        b, a = signal.butter(order, normalized_cutoff, btype='low')
    elif filter_type == 'highpass':
        b, a = signal.butter(order, normalized_cutoff, btype='high')
    else:
        raise ValueError("filter_type 必须是 'lowpass' 或 'highpass'")
    
    # 应用实时滤波器，并设置初始条件使滤波器从第一个数据点开始
    zi = signal.lfilter_zi(b, a)
    # 将初始状态设置为第一个数据点的值
    zi = zi * channel_data[0]
    filtered_data, _ = signal.lfilter(b, a, channel_data, zi=zi)
    return filtered_data

def perform_channel_fft_analysis(data, channel_index, sampling_rate=10000):
    """
    对指定通道执行FFT分析
    
    参数:
    data: DataFrame，包含所有通道的数据
    channel_index: 要分析的通道索引 (0-6)
    sampling_rate: 采样率，默认10000Hz (根据7MIC.py中的设置)
    
    返回:
    xf: 频率轴
    yf: FFT幅度
    """
    # 获取通道数据
    channel_data = data.iloc[:, channel_index].values
    
    # 计算FFT
    n = len(channel_data)
    yf = fft(channel_data)
    xf = fftfreq(n, 1/sampling_rate)
    
    # 只返回正频率部分
    indices = np.where(xf >= 0)
    return xf[indices], np.abs(yf[indices])

def plot_single_channel_data_and_fft(csv_path, channel_index=0, sampling_rate=10000, 
                                   filter_type=None, cutoff_freq=None):
    """
    绘制单个通道的原始数据和FFT分析结果
    
    参数:
    csv_path: CSV文件路径
    channel_index: 要分析的通道索引 (默认为0，即第一个通道)
    sampling_rate: 采样率，默认10000Hz
    filter_type: 滤波器类型 ('lowpass', 'highpass', None)
    cutoff_freq: 截止频率 (Hz)，当filter_type不为None时必须指定
    """
    # 读取数据
    data = read_7mic_csv_data(csv_path)
    if data is None:
        return
    
    num_channels = data.shape[1]
    num_points_per_frame = 100
    num_frames = data.shape[0] // num_points_per_frame
    
    print(f"数据包含 {num_channels} 个通道，{num_frames} 个帧，每帧 {num_points_per_frame} 个点")
    
    if channel_index >= num_channels:
        print(f"通道索引 {channel_index} 超出范围，最大为 {num_channels-1}")
        return
    
    # 创建图形
    fig, axes = plt.subplots(2, 1, figsize=(12, 7))
    fig.suptitle(f'7MIC通道 {channel_index+1} 数据波形和FFT分析 - {os.path.basename(csv_path)}', fontsize=16)
    
    # 计算时间轴
    time = np.arange(num_points_per_frame * num_frames) / sampling_rate
    
    # 获取通道数据
    channel_data = data.iloc[:, channel_index].values

    time = time[:10000]
    channel_data = channel_data[:10000]
    
    # 绘制波形图（在同一幅子图中显示原始信号和滤波后信号）
    axes[0].plot(time, channel_data, label='原始信号', alpha=0.7)
    
    # 应用滤波器（如果指定）
    if filter_type and cutoff_freq:
        filtered_data = apply_filter(channel_data, filter_type, cutoff_freq, sampling_rate)
        axes[0].plot(time, filtered_data, label=f'滤波后信号 ({filter_type} @ {cutoff_freq}Hz)', alpha=0.7)
        filter_title = f" ({filter_type} @ {cutoff_freq}Hz)"
        # 用于FFT分析的数据
        fft_data = filtered_data
    else:
        filter_title = ""
        # 用于FFT分析的数据
        fft_data = channel_data
    
    axes[0].set_title(f'通道 {channel_index+1} 原始波形 vs 滤波后波形')
    axes[0].set_xlabel('时间 (秒)')
    axes[0].set_ylabel('幅值')
    axes[0].grid(True)
    axes[0].legend()
    
    # 执行FFT分析
    # 原始信号FFT
    n_orig = len(channel_data)
    yf_orig = fft(channel_data)
    xf_orig = fftfreq(n_orig, 1/sampling_rate)
    indices_orig = np.where(xf_orig >= 0)
    xf_orig = xf_orig[indices_orig]
    yf_orig = np.abs(yf_orig[indices_orig])
    
    # 只显示5Hz及以上的频率成分
    freq_indices_orig = np.where(xf_orig >= 5)
    
    # 绘制原始信号FFT
    axes[1].plot(xf_orig[freq_indices_orig], yf_orig[freq_indices_orig], 
                 label='原始信号FFT', alpha=0.7)
    
    # 如果应用了滤波，则也绘制滤波后信号的FFT
    if filter_type and cutoff_freq:
        n_filtered = len(fft_data)
        yf_filtered = fft(fft_data)
        xf_filtered = fftfreq(n_filtered, 1/sampling_rate)
        indices_filtered = np.where(xf_filtered >= 0)
        xf_filtered = xf_filtered[indices_filtered]
        yf_filtered = np.abs(yf_filtered[indices_filtered])
        
        # 只显示5Hz及以上的频率成分
        freq_indices_filtered = np.where(xf_filtered >= 5)
        
        # 绘制滤波后信号FFT
        axes[1].plot(xf_filtered[freq_indices_filtered], yf_filtered[freq_indices_filtered], 
                     label=f'滤波后信号FFT ({filter_type} @ {cutoff_freq}Hz)', alpha=0.7)
    
    axes[1].set_title(f'通道 {channel_index+1} FFT分析对比')
    axes[1].set_xlabel('频率 (Hz)')
    axes[1].set_ylabel('幅度')
    axes[1].set_xlim(0, sampling_rate/2)  # 只显示奈奎斯特频率以下的部分
    axes[1].grid(True)
    axes[1].legend()
    
    plt.tight_layout()
    plt.show()

def select_file_and_analyze():
    """
    通过可视化界面选择文件并进行分析
    """
    # 创建一个隐藏的根窗口
    root = tk.Tk()
    root.withdraw()  # 隐藏主窗口
    
    # 打开文件选择对话框
    file_path = filedialog.askopenfilename(
        title="选择CSV文件",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
    )
    
    # 如果用户选择了文件
    if file_path:
        print(f"选择的文件: {file_path}")
        
        # 获取用户输入的其他参数
        channel_to_analyze = int(input("请输入要分析的通道索引 (0-6): ") or "0")
        sampling_rate = int(input("请输入采样率 (默认10000): ") or "10000")
        
        print("滤波器设置:")
        print("1. 低通滤波器 (lowpass)")
        print("2. 高通滤波器 (highpass)")
        print("3. 不使用滤波器")
        filter_choice = input("请选择滤波器类型 (1/2/3, 默认为1): ") or "1"
        
        filter_type = None
        cutoff_freq = None
        if filter_choice == "1":
            filter_type = 'lowpass'
            cutoff_freq = float(input("请输入截止频率 (Hz, 默认750): ") or "750")
        elif filter_choice == "2":
            filter_type = 'highpass'
            cutoff_freq = float(input("请输入截止频率 (Hz, 默认750): ") or "750")
        
        # 进行分析
        plot_single_channel_data_and_fft(file_path, channel_to_analyze, sampling_rate,
                                       filter_type, cutoff_freq)
    else:
        print("未选择文件")

# 直接在代码中指定文件路径和通道
if __name__ == "__main__":
    # 使用可视化界面选择文件
    select_file_and_analyze()