import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.optimize import curve_fit

# 每次修改时不准删除任何原有注释！！！！
plt.rcParams["font.family"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams['axes.unicode_minus'] = False

class FirstOrderLagCompensator:
    def __init__(self, tau=None, base_value=0):
        """初始化一阶滞后补偿器"""
        self.tau = tau  # 时间常数
        self.base = base_value  # 无压力时的基准零值
        self.is_unloading = False  # 是否处于卸载状态
        self.unload_start_time = 0  # 卸载开始时间
        self.unload_initial偏差 = 0  # 卸载初始时的偏差（A值）
        self.history = []  # 存储近期读数用于状态判断
        self.history_len = 15  # 用于判断趋势的历史数据长度（15ms采样周期）
        self.loaded_threshold = None  # 加载状态判定阈值
        self.force_unloading = False  # 强制进入卸载状态的标志
        
    def set_loaded_threshold(self, threshold):
        """设置加载状态判定阈值"""
        self.loaded_threshold = threshold
    
    def force_unloading_state(self, flag=True):
        """强制设置为卸载状态（用于外部识别辅助）"""
        self.force_unloading = flag
        if flag:
            self.is_unloading = True

    def identify_parameters(self, time_data, sensor_data):
        """从卸载实验数据中辨识时间常数tau"""
        def model(t, A, tau, B):
            return A * np.exp(-t / tau) + B
        
        params, _ = curve_fit(model, time_data, sensor_data, 
                             p0=[max(sensor_data)-self.base, 1, self.base])
        self.tau = params[1]
        print(f"辨识得到时间常数tau: {self.tau:.2f}秒")
        return self.tau

    def is_unloading_state(self, current_reading):
        """判断是否处于卸载状态：从加载状态开始连续下降"""
        self.history.append(current_reading)
        if len(self.history) > self.history_len:
            self.history.pop(0)
        
        if len(self.history) < self.history_len or self.loaded_threshold is None:
            return False
            
        # 放宽判定条件，增加容错性
        was_loaded = any(val > self.loaded_threshold for val in self.history[-15:])  # 只看最近15个点
        is_decreasing = sum(self.history[i] > self.history[i+1] for i in range(len(self.history)-1)) > len(self.history)*0.7
        is_below_peak = current_reading < max(self.history) * 0.98  # 从95%放宽到98%
        
        return was_loaded and is_decreasing and is_below_peak

    def compensate(self, current_reading, current_time):
        """实时补偿函数"""
        if self.tau is None:
            return current_reading
        
        # 如果外部强制设置了卸载状态，就直接进入卸载处理
        if self.force_unloading and not self.is_unloading:
            self.is_unloading = True
            self.unload_start_time = current_time
            self.unload_initial偏差 = current_reading - self.base
            self.force_unloading = False  # 只生效一次
        
        if not self.is_unloading:
            # 检查是否进入卸载状态
            if self.is_unloading_state(current_reading):
                self.is_unloading = True
                self.unload_start_time = current_time
                self.unload_initial偏差 = current_reading - self.base
            return current_reading
        else:
            # 执行补偿计算
            delta_t = current_time - self.unload_start_time
            compensated偏差 = self.unload_initial偏差 * np.exp(-delta_t / self.tau)
            compensated_value = self.base + compensated偏差
            
            # 当补偿值接近基准值时，退出卸载状态
            if abs(compensated_value - self.base) < 0.01 * abs(self.unload_initial偏差):
                self.is_unloading = False
                return self.base
            return compensated_value


# ------------------------------
# 主程序：优化加载/卸载识别逻辑
# ------------------------------
if __name__ == "__main__":
    # 1. 读取数据
    file_path = 'raw_data_8x8_20250903_145631.csv'
    script_dir = os.path.dirname(os.path.abspath(__file__))
    full_file_path = os.path.join(script_dir, file_path)
    data = pd.read_csv(full_file_path)
    
    # 2. 数据准备
    channel_0_data = data['Channel_0'].values
    sample_period = 0.015  # 15ms
    time_data = np.arange(0, len(channel_0_data) * sample_period, sample_period)
    
    # 3. 基准值与阈值计算（更稳健）
    base_window_size = 100
    base_value = np.mean(channel_0_data[:base_window_size])
    base_std = np.std(channel_0_data[:base_window_size])
    loaded_threshold = base_value + 20 * base_std  # 保持用户设置的阈值倍数
    print(f"初始无压力基准值: {base_value:.6f}")
    print(f"加载状态判定阈值: {loaded_threshold:.6f}")
    
    # 4. 优化加载开始点识别：连续N个点超过阈值才判定为加载
    load_confirm_window = 10  # 连续15个点（225ms）超过阈值才确认加载
    load_start_idx = None
    # 滑动窗口检查连续超过阈值的点
    for i in range(len(channel_0_data) - load_confirm_window):
        # 窗口内所有点都超过阈值，且前一个点低于阈值（确保是上升沿）
        window = channel_0_data[i:i+load_confirm_window]
        if all(val > loaded_threshold for val in window) and channel_0_data[i-1] <= loaded_threshold:
            load_start_idx = i  # 记录加载开始的第一个点
            break
    # 若未找到，使用原始逻辑兜底（避免报错）
    if load_start_idx is None:
        for i in range(len(channel_0_data)):
            if channel_0_data[i] > loaded_threshold:
                load_start_idx = i
                break
    
    # 5. 优化卸载开始点识别：从加载后的峰值之后寻找
    if load_start_idx is not None:
        # 先找到加载阶段的峰值点（压力最大点）
        # 从加载开始后的数据中找最大值位置
        loaded_data = channel_0_data[load_start_idx:]
        peak_idx_in_loaded = np.argmax(loaded_data)
        peak_idx = load_start_idx + peak_idx_in_loaded  # 全局峰值索引
        
        # 从峰值之后寻找卸载开始点（连续下降）
        unload_confirm_window = 20  # 增加到20个点（300ms），避免误判
        unloading_start_idx = None
        # 从峰值后开始搜索
        for i in range(peak_idx, len(channel_0_data) - unload_confirm_window):
            # 计算当前点之后的差分（变化率）
            diffs = np.diff(channel_0_data[i:i+unload_confirm_window+1])
            if all(d < 0 for d in diffs):  # 连续下降
                unloading_start_idx = i
                break
        # 兜底逻辑
        if unloading_start_idx is None:
            unloading_start_idx = peak_idx  # 若未找到，从峰值点开始算卸载
    else:
        # 若未识别到加载，默认值
        load_start_idx = 0
        unloading_start_idx = 0
    
    print(f"识别到加载开始于第 {load_start_idx} 个采样点")
    print(f"识别到压力峰值于第 {peak_idx} 个采样点")
    print(f"识别到卸载开始于第 {unloading_start_idx} 个采样点")
    
    # 6. 提取卸载数据并补偿
    unloading_time_data = time_data[unloading_start_idx:] - time_data[unloading_start_idx]
    unloading_sensor_data = channel_0_data[unloading_start_idx:]
    
    compensator = FirstOrderLagCompensator(base_value=base_value)
    compensator.set_loaded_threshold(loaded_threshold)
    compensator.identify_parameters(unloading_time_data, unloading_sensor_data)
    
    compensated_values = []
    for i in range(len(channel_0_data)):
        current_t = time_data[i]
        current_raw = channel_0_data[i]
        
        # 当到达外部识别的卸载点时，强制进入卸载状态
        if i == unloading_start_idx:
            compensator.force_unloading_state(True)
            
        compensated = compensator.compensate(current_raw, current_t)
        compensated_values.append(compensated)
    
    # 7. 统计与可视化
    print(f"\n数据点数量: {len(channel_0_data)}")
    print(f"总采样时间: {time_data[-1]:.2f}秒")
    print(f"最大值: {channel_0_data.max():.6f}")
    print(f"最小值: {channel_0_data.min():.6f}")
    print(f"平均值: {channel_0_data.mean():.6f}")
    print(f"标准差: {channel_0_data.std():.6f}")
    
    plt.figure(figsize=(14, 7))  # 每次都固定这个形状，不能修改！！！
    
    # 完整数据对比
    plt.subplot(3, 1, 1)
    plt.plot(time_data, channel_0_data, label="原始数据", color='blue', alpha=0.6)
    plt.plot(time_data, compensated_values, label="补偿后数据", color='red', linewidth=1.5)
    plt.axhline(y=base_value, color='green', linestyle='--', label="初始基准零值")
    plt.axhline(y=loaded_threshold, color='purple', linestyle='-.', label="加载判定阈值")
    plt.axvline(x=time_data[load_start_idx], color='orange', linestyle=':', label="加载开始点")
    plt.axvline(x=time_data[peak_idx], color='black', linestyle=':', label="压力峰值点")
    plt.axvline(x=time_data[unloading_start_idx], color='brown', linestyle=':', label="卸载开始点")
    plt.title('完整数据：原始数据与补偿后数据对比')
    plt.xlabel('时间（秒）')
    plt.ylabel('传感器读数')
    plt.legend()
    plt.grid(True)
    
    # 加载阶段细节
    plt.subplot(3, 1, 2)
    # 显示加载开始到峰值的过程
    plt.plot(time_data[load_start_idx:peak_idx+100], 
             channel_0_data[load_start_idx:peak_idx+100], 
             label="原始数据（加载阶段）", color='blue', alpha=0.6)
    plt.plot(time_data[load_start_idx:peak_idx+100], 
             compensated_values[load_start_idx:peak_idx+100], 
             label="补偿后数据（加载阶段）", color='red', linewidth=1.5)
    plt.axhline(y=loaded_threshold, color='purple', linestyle='-.', label="加载判定阈值")
    plt.axvline(x=time_data[load_start_idx], color='orange', linestyle=':', label="加载开始点")
    plt.axvline(x=time_data[peak_idx], color='black', linestyle=':', label="压力峰值点")
    plt.title('加载阶段细节')
    plt.xlabel('时间（秒）')
    plt.ylabel('传感器读数')
    plt.legend()
    plt.grid(True)
    
    # 卸载阶段细节
    plt.subplot(3, 1, 3)
    plt.plot(time_data[unloading_start_idx:], channel_0_data[unloading_start_idx:], 
             label="原始数据（卸载阶段）", color='blue', alpha=0.6)
    plt.plot(time_data[unloading_start_idx:], compensated_values[unloading_start_idx:], 
             label="补偿后数据（卸载阶段）", color='red', linewidth=1.5)
    plt.axhline(y=base_value, color='green', linestyle='--', label="初始基准零值")
    plt.axvline(x=time_data[unloading_start_idx], color='brown', linestyle=':', label="卸载开始点")
    plt.title('卸载阶段补偿细节')
    plt.xlabel('时间（秒）')
    plt.ylabel('传感器读数')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()
