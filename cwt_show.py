import numpy as np
import pywt
import matplotlib.pyplot as plt

# 生成完整信号
def generate_signal(t):
    return (np.sin(2*np.pi*5*t) + np.sin(2*np.pi*10*t) + 
            np.sin(2*np.pi*20*t) + np.sin(2*np.pi*30*t) + 
            np.sin(2*np.pi*60*t))

# 设置参数
dt = 0.01
t = np.arange(0, 10, dt)  # 完整时间轴
signal = generate_signal(t)

# CWT参数优化
scales = np.arange(1, 128)
cmor_str = 'cmor4.0-2.0'  # 高频优化参数
# cmor 小波必须指定参数形式：cmorB-C
# B = 带宽频率 (bandwidth frequency)  控制时间分辨率，B越大高频响应越好
# C = 中心频率 (center frequency)  影响频率定位精度

# 一次性计算完整CWT
coefficients = pywt.cwt(signal, scales, cmor_str, method='conv')[0]
frequencies = pywt.scale2frequency(cmor_str, scales) / dt

# 动态范围计算（分频段归一化）
power = np.abs(coefficients)**2
vmax_val = np.percentile(power, 99)  # 计算全局99%分位数（标量）

# 创建画布
plt.figure(figsize=(14, 6))
plt.pcolormesh(t, frequencies, power, 
               shading='gouraud', cmap='viridis', vmax=vmax_val)
plt.colorbar(label='Power Density')

plt.title('Offline Continuous Wavelet Transform (CWT)')
plt.xlabel('Time (seconds)')
plt.ylabel('Frequency (Hz)')
plt.ylim([0, 100])
plt.xlim([0, 10])
plt.tight_layout()
plt.show()