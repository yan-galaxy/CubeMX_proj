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
    # return f, t, np.abs(Zxx)

    # 截取大于5Hz的频率部分
    indices = np.where(f > 5)[0]
    return f[indices], t, np.abs(Zxx)[indices, :]

def plot_raw_data(time, signal, channels):
    """绘制原始数据波形"""
    plt.figure(figsize=(12, 6))
    plt.plot(time, signal)
    plt.title(f'原始数据波形 - 通道 {channels[0]}、{channels[1]}、{channels[2]}、{channels[3]}平均')
    plt.xlabel('时间 (秒)')
    plt.ylabel('幅值')
    plt.grid(True)
    plt.tight_layout()
    plt.show()
def plot_fft_results(xf, yf):
    """绘制整体FFT结果"""
    plt.figure(figsize=(12, 6))
    plt.plot(xf, yf)
    plt.title(f'整体FFT分析')
    plt.xlabel('频率 (Hz)')
    plt.ylabel('幅度')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_stft_results(f, t, Zxx):
    """绘制短时FFT结果"""
    plt.figure(figsize=(12, 8))
    plt.pcolormesh(t, f, Zxx, shading='gouraud')
    plt.title(f'短时FFT分析')
    plt.ylabel('频率 (Hz)')
    plt.xlabel('时间 (秒)')
    plt.colorbar(label='幅度')
    plt.ylim(5, f[-1])  # 设置y轴下限为5Hz
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
    # channel = 44          # 默认分析中间通道
    # analysis_data = data[:, channel]
    # print(len(analysis_data))
    # analysis_data = analysis_data[17000:] # 截掉前面还没开始振动的部分
    channels = [43, 44, 54, 55]  # 中间四个通道
    # 计算时间轴和平均数据
    time = np.arange(len(data)) / sampling_rate
    analysis_data = np.mean(data[:, channels], axis=1)
    
    
    
    # 计算时间轴
    time = np.arange(len(analysis_data)) / sampling_rate
    
    # 绘制原始数据
    plot_raw_data(time, analysis_data, channels)

    # 执行整体FFT分析
    xf, yf = perform_fft_analysis(analysis_data, sampling_rate)
    
    # 执行短时FFT分析
    f, t, Zxx = perform_stft_analysis(analysis_data, sampling_rate)
    
    # 绘制结果
    plot_fft_results(xf, yf)
    plot_stft_results(f, t, Zxx)

if __name__ == "__main__":
    main()