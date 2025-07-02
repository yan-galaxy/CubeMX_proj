import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import pandas as pd
from matplotlib.colors import LinearSegmentedColormap

# 设置中文显示
# plt.rcParams["font.family"] = ["SimHei", "WenQuanYi Micro Hei", "Heiti TC"]
plt.rcParams["font.family"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

class FilterAnalyzer:
    """数字滤波器设计与性能分析类"""
    
    def __init__(self, fs=1000):
        """
        初始化滤波器分析器
        
        参数:
            fs: 采样频率(Hz)
        """
        self.fs = fs
        self.filter_coeffs = None
        self.filter_type = None
        self.filter_order = None
        self.cutoff_freq = None
    
    def design_filter(self, filter_type, cutoff_freq, filter_order=4, window='hamming'):
        """
        设计数字滤波器
        
        参数:
            filter_type: 滤波器类型，'lowpass' 或 'highpass'
            cutoff_freq: 截止频率(Hz)
            filter_order: 滤波器阶数
            window: 窗函数类型
        """
        nyquist = 0.5 * self.fs
        normal_cutoff = cutoff_freq / nyquist
        
        if filter_type == 'lowpass':
            b, a = signal.butter(filter_order, normal_cutoff, btype='low', analog=False)
        elif filter_type == 'highpass':
            b, a = signal.butter(filter_order, normal_cutoff, btype='high', analog=False)
        else:
            raise ValueError("滤波器类型必须是 'lowpass' 或 'highpass'")
        
        self.filter_coeffs = (b, a)
        self.filter_type = filter_type
        self.filter_order = filter_order
        self.cutoff_freq = cutoff_freq
        
        return b, a
    
    def plot_frequency_response(self, worN=8000):
        """
        绘制滤波器的频率响应
        
        参数:
            worN: 频率响应计算点数
        """
        if self.filter_coeffs is None:
            raise ValueError("请先设计滤波器")
        
        b, a = self.filter_coeffs
        w, h = signal.freqz(b, a, worN=worN)
        
        plt.figure(figsize=(12, 8))
        
        # 幅频响应
        plt.subplot(2, 1, 1)
        plt.plot(0.5*self.fs*w/np.pi, np.abs(h), 'b')# 20*np.log10(np.abs(h))
        plt.title(f'{self.filter_type}滤波器阶数: {self.filter_order}, 截止频率: {self.cutoff_freq}Hz')
        plt.xlabel('频率 (Hz)')
        plt.ylabel('幅度 ()')
        plt.grid(True)
        plt.axvline(self.cutoff_freq, color='green')  # 截止频率
        plt.axhline(0.707, color='red', linestyle='--')  # -3dB线  -3 0.707
        
        # 相频响应
        plt.subplot(2, 1, 2)
        angles = np.unwrap(np.angle(h))
        plt.plot(0.5*self.fs*w/np.pi, angles, 'g')
        plt.xlabel('频率 (Hz)')
        plt.ylabel('相位 (弧度)')
        plt.grid(True)
        
        plt.tight_layout()
        return plt
    
    def plot_impulse_response(self, duration=0.1):
        """
        绘制滤波器的脉冲响应
        
        参数:
            duration: 响应持续时间(秒)
        """
        if self.filter_coeffs is None:
            raise ValueError("请先设计滤波器")
        
        b, a = self.filter_coeffs
        t = np.arange(0, duration, 1/self.fs)
        impulse = np.zeros_like(t)
        impulse[0] = 1
        response = signal.lfilter(b, a, impulse)
        
        plt.figure(figsize=(12, 6))
        plt.plot(t, response)
        plt.title(f'{self.filter_type}滤波器脉冲响应')
        plt.xlabel('时间 (秒)')
        plt.ylabel('幅度')
        plt.grid(True)
        
        return plt
    
    def plot_step_response(self, duration=0.1):
        """
        绘制滤波器的阶跃响应
        
        参数:
            duration: 响应持续时间(秒)
        """
        if self.filter_coeffs is None:
            raise ValueError("请先设计滤波器")
        
        b, a = self.filter_coeffs
        t = np.arange(0, duration, 1/self.fs)
        step = np.ones_like(t)
        response = signal.lfilter(b, a, step)
        
        plt.figure(figsize=(12, 6))
        plt.plot(t, response)
        plt.title(f'{self.filter_type}滤波器阶跃响应')
        plt.xlabel('时间 (秒)')
        plt.ylabel('幅度')
        plt.grid(True)
        
        return plt
    
    def generate_test_signal(self, duration=1.0, frequencies=None, amplitudes=None):
        """
        生成测试信号
        
        参数:
            duration: 信号持续时间(秒)
            frequencies: 信号包含的频率分量(Hz)
            amplitudes: 各频率分量的幅度
        
        返回:
            t: 时间数组
            signal: 生成的测试信号
        """
        if frequencies is None:
            if self.filter_type == 'lowpass':
                frequencies = [5, 10, 25, 50, 150, 250, 300, 500]  # 包含高低频分量
            else:  # highpass
                frequencies = [5, 10, 25, 50, 150, 250, 300, 500]  # 包含高低频分量
        
        if amplitudes is None:
            amplitudes = [1.0] * len(frequencies)
        
        t = np.arange(0, duration, 1/self.fs)
        test_signal = np.zeros_like(t)
        
        for freq, amp in zip(frequencies, amplitudes):
            test_signal += amp * np.sin(2 * np.pi * freq * t)
        
        # 添加随机噪声
        noise = 0.2 * np.random.randn(len(t))
        test_signal += noise
        
        return t, test_signal
    
    def filter_signal(self, input_signal):
        """
        应用滤波器到输入信号
        
        参数:
            input_signal: 输入信号
        
        返回:
            filtered_signal: 滤波后的信号
        """
        if self.filter_coeffs is None:
            raise ValueError("请先设计滤波器")
        
        b, a = self.filter_coeffs
        return signal.lfilter(b, a, input_signal)
    
    def plot_signal_comparison(self, t, original_signal, filtered_signal):
        """
        绘制原始信号和滤波后信号的对比
        
        参数:
            t: 时间数组
            original_signal: 原始信号
            filtered_signal: 滤波后信号
        """
        plt.figure(figsize=(14, 10))
        
        # 原始信号
        plt.subplot(3, 1, 1)
        plt.plot(t, original_signal)
        plt.title('原始信号')
        plt.xlabel('时间 (秒)')
        plt.ylabel('幅度')
        plt.grid(True)
        
        # 滤波后信号
        plt.subplot(3, 1, 2)
        plt.plot(t, filtered_signal)
        plt.title(f'{self.filter_type}滤波后信号 (截止频率: {self.cutoff_freq}Hz)')
        plt.xlabel('时间 (秒)')
        plt.ylabel('幅度')
        plt.grid(True)
        
        # 频谱分析
        plt.subplot(3, 1, 3)
        n = len(original_signal)
        fft_freq = np.fft.fftfreq(n, 1/self.fs)
        fft_original = np.fft.fft(original_signal)
        fft_filtered = np.fft.fft(filtered_signal)
        
        plt.plot(fft_freq[:n//2], 20*np.log10(np.abs(fft_original[:n//2])), 'b', alpha=0.7, label='原始信号')
        plt.plot(fft_freq[:n//2], 20*np.log10(np.abs(fft_filtered[:n//2])), 'r', alpha=0.7, label='滤波后信号')
        plt.title('频谱分析')
        plt.xlabel('频率 (Hz)')
        plt.ylabel('幅度 (dB)')
        plt.grid(True)
        plt.legend()
        plt.xlim(0, self.fs/2)
        
        plt.tight_layout()
        return plt
    
    def calculate_filter_metrics(self):
        """
        计算滤波器性能指标
        
        返回:
            metrics: 包含滤波器性能指标的字典
        """
        if self.filter_coeffs is None:
            raise ValueError("请先设计滤波器")
        
        b, a = self.filter_coeffs
        w, h = signal.freqz(b, a, worN=8000)
        freq = 0.5 * self.fs * w / np.pi
        mag = 20 * np.log10(np.abs(h))
        
        # 计算截止频率处的衰减
        cutoff_idx = np.abs(freq - self.cutoff_freq).argmin()
        cutoff_attenuation = mag[cutoff_idx]
        
        # 计算通带波纹
        passband = freq < self.cutoff_freq if self.filter_type == 'lowpass' else freq > self.cutoff_freq
        passband_mag = mag[passband]
        passband_ripple = np.max(passband_mag) - np.min(passband_mag)
        
        # 计算阻带衰减
        stopband = freq > self.cutoff_freq if self.filter_type == 'lowpass' else freq < self.cutoff_freq
        stopband_mag = mag[stopband]
        stopband_attenuation = np.min(stopband_mag)
        
        # 计算过渡带宽度 (从-3dB到-20dB)
        idx_3dB = np.where(mag < -3)[0][0]
        idx_20dB = np.where(mag < -20)[0][0]
        transition_width = freq[idx_20dB] - freq[idx_3dB]
        
        metrics = {
            '截止频率衰减 (dB)': cutoff_attenuation,
            '通带波纹 (dB)': passband_ripple,
            '阻带衰减 (dB)': stopband_attenuation,
            '过渡带宽度 (Hz)': transition_width,
            '群延迟 (采样点)': self._calculate_group_delay()
        }
        
        return metrics
    
    def _calculate_group_delay(self):
        """计算滤波器的群延迟"""
        b, a = self.filter_coeffs
        w, gd = signal.group_delay((b, a))
        return np.mean(gd)
    
    def print_metrics_table(self):
        """打印滤波器性能指标表格"""
        metrics = self.calculate_filter_metrics()
        print(f"\n{self.filter_type.upper()}滤波器性能指标 (截止频率: {self.cutoff_freq}Hz, 阶数: {self.filter_order}):")
        print("=" * 50)
        print("{:<25} {:<15}".format("指标", "值"))
        print("-" * 50)
        for metric, value in metrics.items():
            print("{:<25} {:<15.4f}".format(metric, value))
        print("=" * 50)
        
        # 以DataFrame格式返回，便于进一步分析
        return pd.DataFrame(list(metrics.items()), columns=['指标', '值'])
    
    def visualize_filter_effectiveness(self, frequencies=None, amplitudes=None):
        """
        可视化滤波器对不同频率分量的影响
        
        参数:
            frequencies: 测试频率列表
            amplitudes: 各频率的幅度
        """
        if self.filter_coeffs is None:
            raise ValueError("请先设计滤波器")
        
        if frequencies is None:
            frequencies = np.logspace(1, np.log10(self.fs/2), 20)  # 从10Hz到Nyquist频率
        
        if amplitudes is None:
            amplitudes = np.ones_like(frequencies)
        
        # 创建自定义颜色映射
        colors = [(0.1, 0.2, 0.5), (0.9, 0.1, 0.1)]  # 蓝到红
        cmap = LinearSegmentedColormap.from_list('freq_cmap', colors, N=len(frequencies))
        
        plt.figure(figsize=(14, 10))
        
        # 绘制频率响应
        w, h = signal.freqz(*self.filter_coeffs, worN=8000)
        plt.subplot(2, 1, 1)
        plt.plot(0.5*self.fs*w/np.pi, 20*np.log10(np.abs(h)), 'k-', lw=2, label='滤波器频率响应')
        
        # 标记测试频率及其衰减
        for i, (freq, amp) in enumerate(zip(frequencies, amplitudes)):
            # 找到最接近的频率点
            idx = np.abs(0.5*self.fs*w/np.pi - freq).argmin()
            attenuation = 20*np.log10(np.abs(h[idx]))
            
            # 生成测试信号并滤波
            t, signal_data = self.generate_test_signal(duration=0.5, frequencies=[freq], amplitudes=[amp])
            filtered_signal = self.filter_signal(signal_data)
            
            # 计算实际衰减
            original_power = np.mean(signal_data**2)
            filtered_power = np.mean(filtered_signal**2)
            actual_attenuation = 10 * np.log10(filtered_power / original_power) if filtered_power > 0 else -np.inf
            
            # 在频率响应图上标记
            color = cmap(i/len(frequencies))
            plt.scatter(freq, attenuation, color=color, s=50, zorder=5)
            plt.annotate(f'{freq:.1f}Hz\n{attenuation:.1f}dB', 
                         (freq, attenuation), 
                         textcoords="offset points", 
                         xytext=(0,10), 
                         ha='center',
                         fontsize=8)
        
        plt.title(f'{self.filter_type}滤波器频率响应及测试频率衰减')
        plt.xlabel('频率 (Hz)')
        plt.ylabel('幅度 (dB)')
        plt.grid(True)
        plt.legend()
        plt.xscale('log')
        plt.xlim(1, self.fs/2)
        plt.ylim(-80, 5)
        
        # 绘制时域响应
        plt.subplot(2, 1, 2)
        t, _ = self.generate_test_signal(duration=0.5)
        
        for i, (freq, amp) in enumerate(zip(frequencies, amplitudes)):
            _, signal_data = self.generate_test_signal(duration=0.5, frequencies=[freq], amplitudes=[amp])
            filtered_signal = self.filter_signal(signal_data)
            
            color = cmap(i/len(frequencies))
            plt.plot(t[:100], filtered_signal[:100], color=color, 
                     label=f'{freq:.1f}Hz ({amp:.1f}幅度)', alpha=0.7)
        
        plt.title('不同频率信号的滤波效果（时域，前100个采样点）')
        plt.xlabel('时间 (秒)')
        plt.ylabel('幅度')
        plt.grid(True)
        plt.legend(loc='upper right', fontsize=8)
        
        plt.tight_layout()
        return plt


def main():
    """主函数：演示滤波器设计与测试流程"""
    # 创建滤波器分析器实例
    analyzer = FilterAnalyzer(fs=1000)
    
    # 设计低通滤波器
    print("设计低通滤波器...")
    analyzer.design_filter(filter_type='lowpass', cutoff_freq=150, filter_order=4)
    
    # 绘制频率响应
    analyzer.plot_frequency_response()
    plt.show()
    
    # 绘制脉冲响应
    analyzer.plot_impulse_response()
    plt.show()
    
    # 绘制阶跃响应
    analyzer.plot_step_response()
    plt.show()
    
    # 生成测试信号并滤波
    t, test_signal = analyzer.generate_test_signal(duration=1.0)
    filtered_signal = analyzer.filter_signal(test_signal)
    
    # 绘制信号对比
    analyzer.plot_signal_comparison(t, test_signal, filtered_signal)
    plt.show()
    
    # 打印性能指标
    lowpass_metrics = analyzer.print_metrics_table()
    
    # 可视化滤波器对不同频率的影响
    analyzer.visualize_filter_effectiveness()
    plt.show()
    
    # 设计高通滤波器
    print("\n设计高通滤波器...")
    analyzer.design_filter(filter_type='highpass', cutoff_freq=250, filter_order=4)
    
    # 绘制频率响应
    analyzer.plot_frequency_response()
    plt.show()
    
    # 生成测试信号并滤波
    t, test_signal = analyzer.generate_test_signal(duration=1.0)
    filtered_signal = analyzer.filter_signal(test_signal)
    
    # 绘制信号对比
    analyzer.plot_signal_comparison(t, test_signal, filtered_signal)
    plt.show()
    
    # 打印性能指标
    highpass_metrics = analyzer.print_metrics_table()
    
    # 可视化滤波器对不同频率的影响
    analyzer.visualize_filter_effectiveness()
    plt.show()
    
    print("\n滤波器设计与测试完成！")

if __name__ == "__main__":
    main()    
