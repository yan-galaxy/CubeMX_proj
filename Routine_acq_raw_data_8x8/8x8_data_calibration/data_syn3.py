import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler
from matplotlib.widgets import CheckButtons

# 不准删我任何注释！！！！！！！一点都不准删除！！！！

# 配置用于检测第一次急剧上升时刻的传感器列
# 可选值: 'group6_p4', 'force_Fz', 或其他存在的列名
ALIGNMENT_REFERENCE_COLUMN = 'group6_p2'
# 手动调整对齐时刻的偏移量（正数表示将自制传感器向前调整，负数表示向后调整）
MANUAL_ALIGNMENT_OFFSET = 0
# # 设置中文显示
# plt.rcParams["font.family"] = ["Microsoft YaHei", "SimHei"]
# plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

try:
    # 尝试不同的编码方式读取第一个CSV文件（力传感器数据）
    force_data = pd.read_csv('../kunwei_data_20251016_205629.csv') # 
    force_data = force_data[9000:]
    # 计算前5000个点的平均值，用于消除初始漂移
    baseline_mean = force_data[:5000].mean()
    # 对所有数据进行去漂移处理
    force_data = force_data - baseline_mean
    # 尝试不同的编码方式读取第二个CSV文件（8x8传感器原始数据）
    raw_data = pd.read_csv('../self_data_8x8_20251016_205629.csv')# 
    raw_data = raw_data[10000:]
    force_data['Fz'] = -force_data['Fz']  # 反转Fz轴
    # 提取Fx, Fy, Fz数据
    fx_data = force_data['Fx']
    fy_data = force_data['Fy']
    fz_data = force_data['Fz']

    # 查找可能的group6列
    group6_columns = [col for col in raw_data.columns if 'group6' in col]
    
    # 如果找到了group6列，则提取这些数据
    if group6_columns:
        group6_data = raw_data[group6_columns]
    else:
        # 否则使用前4列作为默认数据
        group6_data = raw_data.iloc[:, :min(4, len(raw_data.columns))]

    # 确定用于对齐的列
    if ALIGNMENT_REFERENCE_COLUMN in group6_data.columns:
        alignment_ref_data = group6_data[ALIGNMENT_REFERENCE_COLUMN]
        print(f"已找到{ALIGNMENT_REFERENCE_COLUMN}数据列，将使用该列进行对齐")
    else:
        print(f"未找到{ALIGNMENT_REFERENCE_COLUMN}列，将使用group6的第一列进行对齐")
        alignment_ref_data = group6_data.iloc[:, 0]
        ALIGNMENT_REFERENCE_COLUMN = group6_data.columns[0]

    # 对力传感器数据进行统一归一化（使用所有Fx, Fy, Fz中的最大最小值）
    force_combined = force_data[['Fx', 'Fy', 'Fz']].values
    force_min = force_combined.min()
    force_max = force_combined.max()
    
    # 手动进行归一化
    normalized_force_data = (force_combined - force_min) / (force_max - force_min)
    # 单独归一化Fz用于检测上升沿
    fz_normalized = (fz_data - force_min) / (force_max - force_min)

    # 对8x8传感器数据进行统一归一化（使用group6所有数据中的最大最小值）
    raw_combined = group6_data.values
    raw_min = raw_combined.min()
    raw_max = raw_combined.max()
    
    # 手动进行归一化
    normalized_group6_data = (raw_combined - raw_min) / (raw_max - raw_min)
    
    # 确定用于对齐的列的归一化数据
    alignment_ref_normalized = (alignment_ref_data - raw_min) / (raw_max - raw_min)

    # 函数：检测信号第一次急剧上升的时刻
    def detect_first_rise(data, threshold_factor=5, baseline_length=100):
        """
        检测信号第一次急剧上升的时刻
        data: 输入数据（一维数组）
        threshold_factor: 阈值因子，用于确定检测阈值  最初文件为10
        baseline_length: 用于计算基线的初始数据长度
        """
        # 计算一阶导数（反映变化率）
        diff = np.diff(data)
        
        # 计算基线（使用初始数据段）
        baseline = diff[:baseline_length]
        baseline_mean = np.mean(baseline)
        baseline_std = np.std(baseline)
        
        # 确定阈值（基线均值加上阈值因子倍的标准差）
        threshold = baseline_mean + threshold_factor * baseline_std
        
        # 找到第一个超过阈值的点（急剧上升的起点）
        rise_indices = np.where(diff > threshold)[0]
        
        if len(rise_indices) > 0:
            # 返回第一个急剧上升的位置（+1是因为diff比原数据短1）
            return rise_indices[0] + 1
        else:
            # 如果没有找到明显的上升沿，返回0
            print("未检测到明显的急剧上升时刻，使用数据起始点对齐")
            return 0
    # 函数：检测信号第一次急剧下降的时刻
    def detect_first_fall(data, threshold_factor=30, baseline_length=1000):
        """
        检测信号第一次急剧下降的时刻
        data: 输入数据（一维数组）
        threshold_factor: 阈值因子，用于确定检测阈值  最初文件为10
        baseline_length: 用于计算基线的初始数据长度
        """
        # 计算一阶导数（反映变化率）
        diff = np.diff(data)
        
        # 计算基线（使用初始数据段）
        baseline = diff[:baseline_length]
        baseline_mean = np.mean(baseline)
        baseline_std = np.std(baseline)
        
        # 确定阈值（基线均值减去阈值因子倍的标准差，因为是下降）
        threshold = baseline_mean - threshold_factor * baseline_std
        
        # 找到第一个低于阈值的点（急剧下降的起点）
        fall_indices = np.where(diff < threshold)[0]
        
        if len(fall_indices) > 0:
            # 返回第一个急剧下降的位置（+1是因为diff比原数据短1）
            return fall_indices[0] + 1
        else:
            # 如果没有找到明显的下降沿，返回0
            print("未检测到明显的急剧下降时刻，使用数据起始点对齐")
            return 0
    
    # 检测力传感器Fz的第一次急剧上升时刻/下降时刻
    # fz_rise_index = detect_first_rise(fz_normalized)
    fz_rise_index = detect_first_fall(fz_normalized)
    print(f"Fz第一次急剧上升的时刻（采样点）: {fz_rise_index}")
    
    # 检测对齐参考列的第一次急剧上升时刻/下降时刻
    # alignment_ref_rise_index = detect_first_rise(alignment_ref_normalized)
    alignment_ref_rise_index = detect_first_fall(alignment_ref_normalized)
    print(f"{ALIGNMENT_REFERENCE_COLUMN}第一次急剧上升的时刻（采样点）: {alignment_ref_rise_index}")
    
    # 计算时间差 自制传感器 减去 坤维
    time_diff = alignment_ref_rise_index - fz_rise_index
    print(f"计算得到的时间差（采样点）: {time_diff}")
    # 应用手动调整偏移量
    time_diff += MANUAL_ALIGNMENT_OFFSET
    print(f"应用手动调整后的时差（采样点）: {time_diff}")
    # 根据时间差对齐数据
    force_len = len(normalized_force_data)
    group6_len = len(normalized_group6_data)
    
    # 对齐后的数据长度取较短的
    aligned_length = min(force_len, group6_len - abs(time_diff))
    
    # 初始化对齐后的数据
    normalized_force_aligned = None
    normalized_group6_aligned = None
    
    if time_diff >= 0:
        # 对齐参考列数据滞后，向前平移
        normalized_force_aligned = normalized_force_data[:aligned_length]
        normalized_group6_aligned = normalized_group6_data[time_diff:time_diff+aligned_length]
    else:
        # 力传感器数据滞后，向前平移
        normalized_force_aligned = normalized_force_data[-time_diff:-time_diff+aligned_length]
        normalized_group6_aligned = normalized_group6_data[:aligned_length]

    # 创建一个图表显示对齐后的归一化数据
    fig, ax = plt.subplots(figsize=(15, 7))
    
    # 存储所有的线条对象
    lines = []
    labels = []
    
    # 绘制对齐后的归一化力传感器数据
    force_labels = ['Force Fx', 'Force Fy', 'Force Fz']
    for i in range(normalized_force_aligned.shape[1]):
        line, = ax.plot(normalized_force_aligned[:, i], label=force_labels[i], linewidth=2)
        lines.append(line)
        labels.append(force_labels[i])
    
    # 绘制对齐后的归一化8x8传感器数据
    for i in range(normalized_group6_aligned.shape[1]):
        # 为对齐参考列添加特殊标记
        if group6_data.columns[i] == ALIGNMENT_REFERENCE_COLUMN:
            line, = ax.plot(normalized_group6_aligned[:, i], 
                           label=f'{group6_data.columns[i]} (Alignment Ref)', 
                           linewidth=2.5, linestyle='-', color='darkorange')
        else:
            line, = ax.plot(normalized_group6_aligned[:, i], 
                           label=f'{group6_data.columns[i]}', 
                           linewidth=2, linestyle='-')
        lines.append(line)
        labels.append(f'{group6_data.columns[i]}')
    
    # 标记对齐参考点（第一次急剧上升的时刻）
    # 计算对齐后的上升时刻位置
    if time_diff >= 0:
        # group6数据滞后，需要在对齐后的数据中向后移动time_diff个点
        fz_aligned_rise = fz_rise_index
        alignment_ref_aligned_rise = alignment_ref_rise_index - time_diff
    else:
        # 力传感器数据滞后，需要在对齐后的数据中向后移动abs(time_diff)个点
        fz_aligned_rise = fz_rise_index - (-time_diff)  # 即 fz_rise_index - abs(time_diff)
        alignment_ref_aligned_rise = alignment_ref_rise_index
    
    # 确保索引在有效范围内
    fz_aligned_rise = max(0, min(fz_aligned_rise, len(normalized_force_aligned) - 1))
    alignment_ref_aligned_rise = max(0, min(alignment_ref_aligned_rise, len(normalized_group6_aligned) - 1))
    
    # 绘制对齐参考线（两条线应该重合）
    ax.axvline(x=fz_aligned_rise, color='red', linestyle=':', linewidth=2, label='Fz Rise Point')
    ax.axvline(x=alignment_ref_aligned_rise, color='blue', linestyle='-.', linewidth=2, label=f'{ALIGNMENT_REFERENCE_COLUMN} Rise Point')
    
    # 设置图表属性
    ax.set_title(f'Aligned Normalized Sensor Data\n(Aligned using first sharp rise of Fz and {ALIGNMENT_REFERENCE_COLUMN})\n(Force data normalized together, Group6 data normalized together)')
    ax.set_xlabel('Sample Index')
    ax.set_ylabel('Normalized Value (0-1)')
    ax.grid(True)
    
    # 创建复选框区域
    # 参数说明: [left, bottom, width, height] - 所有值都是相对于图表整体尺寸的比例(0-1之间)
    # left: 左边距 - 0.02表示距离左边2%
    # bottom: 下边距 - 0.4表示距离底部40%
    # width: 宽度 - 0.15表示宽度占整个图表的15%
    # height: 高度 - 0.5表示高度占整个图表的50%
    ax_check = plt.axes([0.05, 0.7, 0.15, 0.2])
    # 设置复选框背景为半透明白色 (RGBA: 红,绿,蓝,透明度)
    ax_check.set_facecolor((1, 1, 1, 0.1))  # 最后一个值0.7表示70%不透明度，即30%透明
    labels_for_check = labels.copy()
    visibility = [True] * len(lines)
    check = CheckButtons(ax_check, labels_for_check, visibility)
    
    # 存储我们的自定义标记
    check_markers = []
    
    # 自定义复选框样式，使用对勾而不是叉
    for i, (checkbox, label) in enumerate(zip(check.lines, check.labels)):
        # 移除默认的叉号线
        for line in checkbox:
            line.set_visible(False)
        
        # 获取复选框线条的x、y数据并转换为numpy数组
        x_data = np.array(checkbox[0].get_xdata())
        y_data = np.array(checkbox[0].get_ydata())
        
        # 计算复选框的中心坐标
        x_pos = x_data.mean()  # 水平中心
        y_pos = y_data.mean()  # 垂直中心
        
        # 添加文本对勾符号（位置设为复选框中心）
        tick = ax_check.text(x_pos, y_pos, '✓', fontsize=14, 
                            verticalalignment='center', 
                            horizontalalignment='center',
                            color='green', weight='bold')
        tick.set_visible(visibility[i])
        check_markers.append(tick)
    
    # 定义复选框回调函数
    def func(label):
        index = labels_for_check.index(label)
        lines[index].set_visible(not lines[index].get_visible())
        check_markers[index].set_visible(lines[index].get_visible())
        plt.draw()
    
    check.on_clicked(func)
    
    # 添加图例
    ax.legend(loc='upper right', bbox_to_anchor=(1, 1))
    
    plt.tight_layout()
    # plt.subplots_adjust(right=0.8)  # 为复选框留出空间
    plt.show()



    # 提取原始数据并进行对齐（不使用归一化数据）
    # 首先需要获取原始力数据和原始group6数据的对齐版本
    if 'Fx' in force_data.columns and 'Fy' in force_data.columns and 'Fz' in force_data.columns:
        # 获取原始力数据
        raw_force_data = force_data[['Fx', 'Fy', 'Fz']].values
        
        # 获取原始group6数据
        raw_group6_values = group6_data.values
        
        # 基于之前计算的时间差对齐原始数据
        raw_force_aligned = None
        raw_group6_aligned = None
        
        if time_diff >= 0:
            # 对齐参考列数据滞后，向前平移
            raw_force_aligned = raw_force_data[:aligned_length]
            raw_group6_aligned = raw_group6_values[time_diff:time_diff+aligned_length]
        else:
            # 力传感器数据滞后，向前平移
            raw_force_aligned = raw_force_data[-time_diff:-time_diff+aligned_length]
            raw_group6_aligned = raw_group6_values[:aligned_length]
        
        # 保存对齐后的原始数据到CSV文件
        aligned_data_df = pd.DataFrame()
        aligned_data_df['Fx'] = raw_force_aligned[:, 0]
        aligned_data_df['Fy'] = raw_force_aligned[:, 1]
        aligned_data_df['Fz'] = raw_force_aligned[:, 2]
        
        for i, col_name in enumerate(group6_data.columns):
            aligned_data_df[col_name] = raw_group6_aligned[:, i]
        
        # aligned_data_df.to_csv('aligned_data3.csv', index=False)
        # print("\n已将对齐后的原始数据保存到 'aligned_data3.csv'")
        



except FileNotFoundError as e:
    print(f"文件未找到: {e}")
    print("请确保以下文件存在于当前目录中:")
    print("1. 20251014222347.csv")
    print("2. raw_data_8x8_20251014_222344.csv")
except KeyError as e:
    print(f"列名错误: {e}")
    print("请检查CSV文件是否包含所需的列名")
except Exception as e:
    print(f"发生错误: {e}")
    import traceback
    traceback.print_exc()