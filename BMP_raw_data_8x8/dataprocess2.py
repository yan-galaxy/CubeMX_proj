import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
import os

plt.rcParams["font.family"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams['axes.unicode_minus'] = False

# ---------------------- 1. 读取数据（确保双精度） ----------------------
file_path = 'raw_data_8x8_20250903_145631.csv'
script_dir = os.path.dirname(os.path.abspath(__file__))
full_file_path = os.path.join(script_dir, file_path)
data = pd.read_csv(full_file_path)

# 转为float64（双精度），提升拟合稳定性
channel_0_data = data['Channel_4'].values.astype(np.float64)  
sample_indices = np.arange(len(channel_0_data), dtype=np.float64)  # 索引也用float64
n_samples = len(channel_0_data)

# ---------------------- 2. 提取关键特征 ----------------------
peak_idx = np.argmax(channel_0_data)
peak_value = channel_0_data[peak_idx]
print(f"峰值索引: {peak_idx}, 峰值大小: {peak_value:.6f}")

# 基线：取最初100个点（双精度计算）
baseline_start = 0
baseline_end = min(100, len(channel_0_data))
baseline_range = slice(baseline_start, baseline_end)
baseline = np.mean(channel_0_data[baseline_range])
print(f"基线值（前{baseline_end - baseline_start}个点的均值）: {baseline:.6f}")

# ---------------------- 3. 核心：动态判断力撤去时刻（优化拟合逻辑） ----------------------
# 3.1 衰减段数据（双精度）
decay_start = peak_idx
decay_indices = sample_indices[decay_start:]
decay_values = channel_0_data[decay_start:]
x_decay = decay_indices - peak_idx  # 相对于峰值的偏移（x=0为峰值）

# 3.2 纯弛豫模型（指数衰减，确保物理意义）
def pure_relaxation(x, A, tau):
    # A：振幅（正），tau：时间常数（正），避免指数爆炸
    return A * np.exp(-x / tau) + baseline

# 3.3 滑动窗口拟合（关键优化）
window_size = 25  # 调整窗口大小（20-30为宜，避免数据量过少）
threshold = 0.03  # 降低残差阈值（根据数据噪声调整，避免误判）
force_removal_idx = None
# 初始化拟合参数（避免后续子图3报错）
A_window = peak_value - baseline  # 初始A=峰值差（物理合理值）
tau_window = 50.0  # 初始tau=50（可根据数据粗估，比100更灵活）

# 滑动窗口检测（从峰值后开始）
for i in range(len(x_decay) - window_size):
    x_window = x_decay[i:i+window_size]
    y_window = decay_values[i:i+window_size]
    
    try:
        # 优化1：初始值p0（A=当前窗口峰值差，tau=50，更贴近实际）
        current_peak_in_window = np.max(y_window)
        p0 = [current_peak_in_window - baseline, 50.0]
        
        # 优化2：用trf算法（鲁棒性强）+ 参数边界（A>0, tau>0，物理约束）
        params, _ = curve_fit(
            pure_relaxation,
            x_window,
            y_window,
            p0=p0,
            method='trf',  # 替换默认的LM算法，支持边界约束
            bounds=((1e-6, 1e-6), (np.inf, np.inf))  # A≥1e-6，tau≥1e-6（避免非正数）
        )
        A_window, tau_window = params  # 更新当前窗口拟合参数
        
        # 计算残差（用RMSE更敏感，避免平均绝对误差的平滑效应）
        y_fit = pure_relaxation(x_window, A_window, tau_window)
        residual = np.sqrt(np.mean((y_window - y_fit)**2))  # RMSE残差
        
        # 残差小于阈值 → 力已撤去
        if residual < threshold:
            force_removal_idx = int(decay_indices[i])  # 转为整数索引
            print(f"检测到力在索引 {force_removal_idx} 处完全撤去")
            break
    except Exception as e:
        # 打印异常（便于调试，如拟合失败原因）
        # print(f"窗口{i}拟合失败: {str(e)[:50]}")
        continue

# 未检测到则默认峰值后10个点
if force_removal_idx is None:
    force_removal_idx = peak_idx + 10
    print(f"未明确检测到力撤去，默认在索引 {force_removal_idx} 处撤去")

# ---------------------- 4. 生成力的波形 ----------------------
calibration_factor = 1.0
force_peak = calibration_factor * (peak_value - baseline)  
print(f"峰值力（相对值）: {force_peak:.6f}")

force_values = np.zeros(n_samples, dtype=np.float64)  # 双精度力数组

# 峰值前：力随信号上升
pre_peak_mask = sample_indices < peak_idx
force_values[pre_peak_mask] = calibration_factor * (channel_0_data[pre_peak_mask] - baseline)

# 峰值到力撤去：线性衰减到0
transition_mask = (sample_indices >= peak_idx) & (sample_indices < force_removal_idx)
transition_length = force_removal_idx - peak_idx
if transition_length > 0:
    force_values[transition_mask] = force_peak * (1 - np.arange(transition_length)/transition_length)

# ---------------------- 5. 可视化验证 ----------------------
plt.figure(figsize=(14, 7))  # 每次都固定这个形状，不能修改！！！

# 子图1：原始信号
plt.subplot(4, 1, 1)
plt.plot(sample_indices, channel_0_data, 'b-', label='原始传感器信号')
plt.axvline(x=peak_idx, color='r', linestyle='--', label=f'峰值（力最大）')
plt.axvline(x=force_removal_idx, color='purple', linestyle='--', label=f'力完全撤去')
plt.axhline(y=baseline, color='g', linestyle='--', label=f'基线')
plt.title('原始信号与关键节点')
plt.xlabel('样本索引')
plt.ylabel('信号值')
plt.legend(fontsize=9)
plt.grid(True)

# 子图2：残差曲线
plt.subplot(4, 1, 2)
residuals = []
for i in range(len(x_decay) - window_size):
    x_window = x_decay[i:i+window_size]
    y_window = decay_values[i:i+window_size]
    try:
        params, _ = curve_fit(
            pure_relaxation, x_window, y_window,
            p0=[np.max(y_window)-baseline, 50.0],
            method='trf', bounds=((1e-6, 1e-6), (np.inf, np.inf))
        )
        y_fit = pure_relaxation(x_window, *params)
        residuals.append(np.sqrt(np.mean((y_window - y_fit)**2)))
    except:
        residuals.append(np.nan)
plt.plot(decay_indices[:len(residuals)], residuals, 'orange', label='RMSE残差（滑动窗口）')
plt.axhline(y=threshold, color='red', linestyle='--', label=f'判断阈值={threshold}')
plt.axvline(x=force_removal_idx, color='purple', linestyle='--', label='力撤去时刻')
plt.title('滑动窗口残差')
plt.xlabel('样本索引')
plt.ylabel('RMSE残差')
plt.legend(fontsize=9)
plt.grid(True)

# 子图3：拟合对比（用最终稳定的参数）
plt.subplot(4, 1, 3)
x_fit_full = np.linspace(0, max(x_decay), 1000)
y_fit_full = pure_relaxation(x_fit_full, A_window, tau_window)
plt.plot(decay_indices, decay_values, 'b.', label='实际衰减信号', alpha=0.6)
plt.plot(peak_idx + x_fit_full, y_fit_full, 'r-', label=f'纯弛豫拟合（A={A_window:.2f}, τ={tau_window:.2f}）')
plt.axvline(x=force_removal_idx, color='purple', linestyle='--', label='力撤去时刻')
plt.title('衰减段拟合对比')
plt.xlabel('样本索引')
plt.ylabel('信号值')
plt.legend(fontsize=9)
plt.grid(True)

# 子图4：力波形
plt.subplot(4, 1, 4)
plt.plot(sample_indices, force_values, 'r-', label='力的波形')
plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
plt.axvline(x=force_removal_idx, color='purple', linestyle='--', label='力完全撤去')
plt.title('动态判断力撤去的力波形')
plt.xlabel('样本索引')
plt.ylabel('力（相对单位）')
plt.legend(fontsize=9)
plt.grid(True)

plt.tight_layout()
plt.show()