# fft_analysis.py
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
from scipy.signal import stft
import os
import sys


plt.rcParams["font.family"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

def read_csv_data(csv_path):
    """读取CSV文件数据"""
    try:
        data = np.loadtxt(csv_path, delimiter=',')
        print(f"成功加载数据，形状: {data.shape}")
        return data
    except Exception as e:
        print(f"读取CSV文件失败: {e}")
        return None

def perform_fft_analysis(data, sampling_rate=1000):
    """执行整体FFT分析"""
    n = data.shape[0]
    yf = fft(data)
    xf = fftfreq(n, 1/sampling_rate)
    
    # 只显示正频率部分
    indices = np.where(xf > 0)
    return xf[indices], np.abs(yf[indices])

def perform_stft_analysis(data, sampling_rate=1000, nperseg=256):
    """执行短时FFT分析"""
    f, t, Zxx = stft(data, fs=sampling_rate, nperseg=nperseg)
    return f, t, np.abs(Zxx)

def plot_fft_results(xf, yf, channel=0):
    """绘制整体FFT结果"""
    plt.figure(figsize=(12, 6))
    plt.plot(xf, yf)  # 修改这里，直接使用yf而不是yf[:, channel]
    plt.title(f'整体FFT分析 - 通道 {channel}')
    plt.xlabel('频率 (Hz)')
    plt.ylabel('幅度')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_stft_results(f, t, Zxx, channel=0):
    """绘制短时FFT结果"""
    plt.figure(figsize=(12, 8))
    plt.pcolormesh(t, f, Zxx, shading='gouraud')  # 修改这里，直接使用Zxx而不是Zxx[channel]
    plt.title(f'短时FFT分析 - 通道 {channel}')
    plt.ylabel('频率 (Hz)')
    plt.xlabel('时间 (秒)')
    plt.colorbar(label='幅度')
    plt.tight_layout()
    plt.show()

def main():
    # 弹出文件选择对话框
    import tkinter as tk
    from tkinter import filedialog
    root = tk.Tk()
    root.withdraw()
    
    csv_path = filedialog.askopenfilename(
        title="选择CSV文件",
        filetypes=[("CSV文件", "*.csv")]
    )
    
    if not csv_path:
        print("未选择文件，程序退出。")
        return
    
    # 读取数据
    data = read_csv_data(csv_path)
    if data is None:
        return
    
    # 参数设置
    sampling_rate = 1000  # 根据实际采样率修改
    channel = 44          # 默认分析中间通道
    
    # 执行整体FFT分析
    xf, yf = perform_fft_analysis(data[:, channel], sampling_rate)
    
    # 执行短时FFT分析
    f, t, Zxx = perform_stft_analysis(data[:, channel], sampling_rate)
    
    # 绘制结果
    plot_fft_results(xf, yf, channel)
    plot_stft_results(f, t, Zxx, channel)

if __name__ == "__main__":
    main()