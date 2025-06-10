import numpy as np
import pywt
import matplotlib.pyplot as plt
from scipy.signal import stft  # 新增STFT模块

# 生成完整信号
def generate_signal(t):
    return (
        np.sin(2*np.pi*5*t) 
        # + np.sin(2*np.pi*10*t) 
        # + np.sin(2*np.pi*15*t) 
        # + np.sin(2*np.pi*20*t) 
        + np.sin(2*np.pi*25*t)
        # + np.sin(2*np.pi*49*t) 
            )

# 设置参数
dt = 0.01
t = np.arange(0, 10, dt)  # 完整时间轴
signal = generate_signal(t)
fs = 1/dt  # 计算采样率（新增）

# === CWT分析部分（修正后）===
# CWT参数优化（修正尺度范围）
scales = np.arange(1, 256)  # 原128 → 64（对应50Hz极限频率）
cmor_str = 'cmor5.0-1.0'  # 高频优化参数
# cmor 小波必须指定参数形式：cmorB-C
# B = 带宽频率 (bandwidth frequency)  控制时间分辨率，B越大高频响应越好
# C = 中心频率 (center frequency)  影响频率定位精度

# 一次性计算完整CWT
coefficients = pywt.cwt(signal, scales, cmor_str, method='conv')[0]

# 修正频率轴计算（添加小波中心频率校正因子）
center_freq = 1.0  # cmor70.0-1.0小波的中心频率理论值
frequencies = (center_freq / scales) * (fs / 2)  # 新的频率计算公式

# 动态范围计算（分频段归一化）
power_cwt = np.abs(coefficients)**2
vmax_val = np.percentile(power_cwt, 99)  # 计算全局99%分位数（标量）

# === STFT分析部分（修正后）===
# STFT参数设置（修改nperseg以匹配理论限制）
nperseg = 256  # 原128 → 64（避免频率分辨率过度）
noverlap = nperseg // 2  # 重叠点数

# 计算STFT
f_stft, t_stft, Zxx = stft(signal, fs=fs, window='hann', 
                          nperseg=nperseg, noverlap=noverlap)
power_stft = np.abs(Zxx)**2

# # 创建对比画布
# fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8))
# 创建对比画布（修改为3个子图）
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 8))  # 修改这里

# 新增：绘制CWT原始尺度表示
scale_mesh = ax3.pcolormesh(t, scales[::-1], power_cwt,  # 反转尺度轴
                          shading='gouraud', cmap='viridis', vmax=vmax_val)
ax3.set_title('Continuous Wavelet Transform (Scale Representation)')
ax3.set_xlabel('Time (seconds)')
ax3.set_ylabel('Scale')
ax3.set_ylim([256, 1])  # 设置尺度范围
ax3.set_xlim([0, 10])
plt.colorbar(scale_mesh, ax=ax3, label='Power Density')


# 绘制CWT结果
cwt_mesh = ax1.pcolormesh(t, frequencies, power_cwt, 
                         shading='gouraud', cmap='viridis', vmax=vmax_val)
ax1.set_title('Continuous Wavelet Transform (CWT)')
ax1.set_xlabel('Time (seconds)')
ax1.set_ylabel('Frequency (Hz)')
ax1.set_ylim([0, 50])  # 原100 → 50（符合奈奎斯特极限）
ax1.set_xlim([0, 10])
plt.colorbar(cwt_mesh, ax=ax1, label='Power Density')

# 绘制STFT结果
stft_mesh = ax2.pcolormesh(t_stft, f_stft, power_stft, 
                          shading='gouraud', cmap='viridis', vmax=vmax_val)
ax2.set_title('Short-Time Fourier Transform (STFT)')
ax2.set_xlabel('Time (seconds)')
ax2.set_ylabel('Frequency (Hz)')
ax2.set_ylim([0, 50])  # 原100 → 50
ax2.set_xlim([0, 10])
plt.colorbar(stft_mesh, ax=ax2, label='Power Density')

plt.tight_layout()
plt.show()