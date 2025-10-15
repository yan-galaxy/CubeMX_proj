import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler
from matplotlib.widgets import CheckButtons

# 不准删我任何注释！！！！！！！一点都不准删除！！！！

# # 设置中文显示
# plt.rcParams["font.family"] = ["Microsoft YaHei", "SimHei"]
# plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

try:
    # 尝试不同的编码方式读取第一个CSV文件（力传感器数据）
    try:
        force_data = pd.read_csv('20251014222347.csv', encoding='gbk')
    except UnicodeDecodeError:
        try:
            force_data = pd.read_csv('20251014222347.csv', encoding='utf-8')
        except UnicodeDecodeError:
            force_data = pd.read_csv('20251014222347.csv', encoding='latin1')
    
    # 尝试不同的编码方式读取第二个CSV文件（8x8传感器原始数据）
    try:
        raw_data = pd.read_csv('raw_data_8x8_20251014_222344.csv', encoding='gbk')
    except UnicodeDecodeError:
        try:
            raw_data = pd.read_csv('raw_data_8x8_20251014_222344.csv', encoding='utf-8')
        except UnicodeDecodeError:
            raw_data = pd.read_csv('raw_data_8x8_20251014_222344.csv', encoding='latin1')
    
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

    # 确保能找到group6_p4，如果找不到则使用group6的第一列
    if 'group6_p4' in group6_data.columns:
        group6_p4_data = group6_data['group6_p4']
        print("已找到group6_p4数据列，将使用该列进行对齐")
    else:
        print("未找到group6_p4列，将使用group6的第一列进行对齐")
        group6_p4_data = group6_data.iloc[:, 0]

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
    # 单独归一化group6_p4用于检测上升沿
    group6_p4_normalized = (group6_p4_data - raw_min) / (raw_max - raw_min)

    # 函数：检测信号第一次急剧上升的时刻
    def detect_first_rise(data, threshold_factor=10, baseline_length=100):
        """
        检测信号第一次急剧上升的时刻
        data: 输入数据（一维数组）
        threshold_factor: 阈值因子，用于确定检测阈值
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
    
    # 检测Fz的第一次急剧上升时刻
    fz_rise_index = detect_first_rise(fz_normalized)
    print(f"Fz第一次急剧上升的时刻（采样点）: {fz_rise_index}")
    
    # 检测group6_p4的第一次急剧上升时刻
    group6_p4_rise_index = detect_first_rise(group6_p4_normalized)
    print(f"group6_p4第一次急剧上升的时刻（采样点）: {group6_p4_rise_index}")
    
    # 计算时间差
    time_diff = group6_p4_rise_index - fz_rise_index
    print(f"计算得到的时间差（采样点）: {time_diff}")
    
    # 根据时间差对齐数据
    force_len = len(normalized_force_data)
    group6_len = len(normalized_group6_data)
    
    # 对齐后的数据长度取较短的
    aligned_length = min(force_len, group6_len - abs(time_diff))
    
    # 初始化对齐后的数据
    normalized_force_aligned = None
    normalized_group6_aligned = None
    
    if time_diff >= 0:
        # group6_p4数据滞后，向前平移
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
        # 为group6_p4添加特殊标记
        if group6_data.columns[i] == 'group6_p4' or (i == 0 and 'group6_p4' not in group6_data.columns):
            line, = ax.plot(normalized_group6_aligned[:, i], 
                           label=f'{group6_data.columns[i]}', 
                           linewidth=2.5, linestyle='-', color='darkorange')
        else:
            line, = ax.plot(normalized_group6_aligned[:, i], 
                           label=f'{group6_data.columns[i]}', 
                           linewidth=2, linestyle='-')
        lines.append(line)
        labels.append(f'{group6_data.columns[i]}')
    
    # 标记对齐参考点（第一次急剧上升的时刻）
    # 计算对齐后的上升时刻位置
    fz_aligned_rise = fz_rise_index if fz_rise_index < len(normalized_force_aligned) else len(normalized_force_aligned) - 1
    group6_p4_aligned_rise = group6_p4_rise_index - time_diff if (group6_p4_rise_index - time_diff) >= 0 and (group6_p4_rise_index - time_diff) < len(normalized_group6_aligned) else 0
    
    # 绘制对齐参考线（两条线应该重合）
    ax.axvline(x=fz_aligned_rise, color='red', linestyle=':', linewidth=2, label='Alignment Reference')
    ax.axvline(x=group6_p4_aligned_rise, color='red', linestyle='-', linewidth=1, alpha=0.5)
    
    # 设置图表属性
    ax.set_title('Aligned Normalized Sensor Data\n(Aligned using first sharp rise of Fz and group6_p4)\n(Force data normalized together, Group6 data normalized together)')
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
            # group6_p4数据滞后，向前平移
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
        
        aligned_data_df.to_csv('aligned_data.csv', index=False)
        print("\n已将对齐后的原始数据保存到 'aligned_data.csv'")
        



    # print("\n====== 开始使用原始数据进行传感器标定 ======")
    
    # # 提取原始数据并进行对齐（不使用归一化数据）
    # # 首先需要获取原始力数据和原始group6数据的对齐版本
    # if 'Fx' in force_data.columns and 'Fy' in force_data.columns and 'Fz' in force_data.columns:
    #     # 获取原始力数据
    #     raw_force_data = force_data[['Fx', 'Fy', 'Fz']].values
        
    #     # 获取原始group6数据
    #     raw_group6_values = group6_data.values
        
    #     # 基于之前计算的时间差对齐原始数据
    #     raw_force_aligned = None
    #     raw_group6_aligned = None
        
    #     if time_diff >= 0:
    #         # group6_p4数据滞后，向前平移
    #         raw_force_aligned = raw_force_data[:aligned_length]
    #         raw_group6_aligned = raw_group6_values[time_diff:time_diff+aligned_length]
    #     else:
    #         # 力传感器数据滞后，向前平移
    #         raw_force_aligned = raw_force_data[-time_diff:-time_diff+aligned_length]
    #         raw_group6_aligned = raw_group6_values[:aligned_length]
        
    #     # 确保对齐后的数据有效且有足够的传感器
    #     if raw_force_aligned is not None and raw_group6_aligned is not None and raw_group6_aligned.shape[1] >= 4:
    #         # 特征：四个压阻传感器的原始读数 (group6_p1到p4)
    #         # 目标：Fx, Fy, Fz的原始力值
    #         X_raw = raw_group6_aligned[:, :4]  # 只使用前4个传感器数据
    #         y_raw = raw_force_aligned
            
    #         # 添加多项式特征（2次多项式）
    #         def add_polynomial_features_raw(X, degree=2):
    #             """添加多项式特征，包括交互项"""
    #             n_samples, n_features = X.shape
    #             features = [X]
                
    #             # 添加高阶项
    #             for d in range(2, degree+1):
    #                 features.append(X ** d)
                
    #             # 添加交互项 (两个不同特征的乘积)
    #             for i in range(n_features):
    #                 for j in range(i+1, n_features):
    #                     features.append(X[:, i] * X[:, j])
                
    #             return np.column_stack(features)
            
    #         # 生成多项式特征
    #         X_poly_raw = add_polynomial_features_raw(X_raw, degree=2)
            
    #         # 添加偏置项（常数项）
    #         X_poly_raw = np.hstack([np.ones((X_poly_raw.shape[0], 1)), X_poly_raw])
            
    #         # 使用最小二乘法分别拟合Fx, Fy, Fz
    #         def least_squares_fit_raw(X, y):
    #             """使用最小二乘法求解线性回归系数: y = Xw"""
    #             # w = (X^T X)^(-1) X^T y
    #             # 添加微小扰动防止矩阵奇异
    #             X_T_X = X.T @ X
    #             X_T_X_reg = X_T_X + np.eye(X_T_X.shape[0]) * 1e-8
    #             w = np.linalg.inv(X_T_X_reg) @ X.T @ y
    #             return w
            
    #         # 分别拟合三个力分量
    #         coefficients_raw = {}
    #         for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    #             print(f"使用原始数据拟合 {force_component} 的标定模型...")
    #             coefficients_raw[force_component] = least_squares_fit_raw(X_poly_raw, y_raw[:, i])
            
    #         # 使用拟合的模型进行预测
    #         predictions_raw = {}
    #         for force_component in ['Fx', 'Fy', 'Fz']:
    #             predictions_raw[force_component] = X_poly_raw @ coefficients_raw[force_component]
            
    #         # 计算拟合误差
    #         def calculate_error_raw(y_true, y_pred):
    #             """计算均方误差和决定系数R²"""
    #             mse = np.mean((y_true - y_pred) ** 2)
    #             r2 = 1 - np.sum((y_true - y_pred) ** 2) / np.sum((y_true - np.mean(y_true)) ** 2)
    #             return mse, r2
            
    #         # 打印每个分量的拟合误差
    #         print("\n====== 原始数据拟合误差分析 ======")
    #         for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    #             mse, r2 = calculate_error_raw(y_raw[:, i], predictions_raw[force_component])
    #             print(f"{force_component} - 均方误差: {mse:.6f}, 决定系数R²: {r2:.6f}")
            
    #         # 可视化拟合结果
    #         fig, axes = plt.subplots(3, 1, figsize=(15, 15))
    #         fig.suptitle('原始力传感器实测值与压阻传感器预测值对比', fontsize=16)
            
    #         colors = ['blue', 'green', 'red']
    #         for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    #             axes[i].plot(y_raw[:, i], label=f'实测 {force_component}', color=colors[i], alpha=0.7)
    #             axes[i].plot(predictions_raw[force_component], label=f'预测 {force_component}', 
    #                          color=colors[i], linestyle='--')
    #             axes[i].set_title(f'{force_component} 拟合结果 (R² = {calculate_error_raw(y_raw[:, i], predictions_raw[force_component])[1]:.4f})')
    #             axes[i].set_xlabel('样本索引')
    #             axes[i].set_ylabel('力值')
    #             axes[i].grid(True)
    #             axes[i].legend()
            
    #         plt.tight_layout(rect=[0, 0, 1, 0.96])  # 为suptitle留出空间
    #         plt.show()
            
    #         # 保存原始数据的标定系数
    #         import json
    #         calibration_data_raw = {
    #             'coefficients': {k: v.tolist() for k, v in coefficients_raw.items()},
    #             'feature_degree': 2
    #         }
            
    #         with open('sensor_calibration_raw.json', 'w') as f:
    #             json.dump(calibration_data_raw, f, indent=4)
            
    #         print("\n原始数据标定完成！标定系数已保存到 'sensor_calibration_raw.json'")
            
    #         # 展示标定方程形式
    #         print("\n====== 原始数据标定方程形式 ======")
    #         features = ['常数项']
    #         sensors = ['group6_p1', 'group6_p2', 'group6_p3', 'group6_p4']
            
    #         # 添加一次项
    #         features.extend([f'{s}' for s in sensors])
    #         # 添加二次项
    #         features.extend([f'{s}²' for s in sensors])
    #         # 添加交互项
    #         for i in range(4):
    #             for j in range(i+1, 4):
    #                 features.extend([f'{sensors[i]}×{sensors[j]}'])
            
    #         print(f"力值 = {features[0]} + " + " + ".join([f"w_{i}×{features[i]}" for i in range(1, len(features))]))
    #         print("\n其中w_i为拟合得到的系数")
            
    #         # 提供一个使用标定方程进行预测的函数（基于原始数据）
    #         def predict_force_raw(sensor_readings, component='Fx'):
    #             """
    #             使用原始数据标定方程预测力值
    #             sensor_readings: 长度为4的列表，包含group6_p1到p4的原始读数
    #             component: 要预测的力分量，'Fx', 'Fy'或'Fz'
    #             """
    #             # 构建多项式特征
    #             X_predict = add_polynomial_features_raw(np.array(sensor_readings).reshape(1, -1), degree=2)
    #             X_predict = np.hstack([np.ones((1, 1)), X_predict])  # 添加偏置项
                
    #             # 预测力值（直接使用原始尺度）
    #             force_value = X_predict @ coefficients_raw[component]
                
    #             return float(force_value)
            
    #         print("\n====== 原始数据标定示例 ======")
    #         # 使用第一个样本数据进行预测示例
    #         sample_raw_readings = X_raw[0, :4]  # 前4个传感器的原始读数
            
    #         print(f"传感器原始读数示例: {sample_raw_readings}")
    #         print(f"实际Fx值: {y_raw[0, 0]:.4f}")
    #         print(f"预测Fx值: {predict_force_raw(sample_raw_readings, 'Fx'):.4f}")
    #         print(f"实际Fy值: {y_raw[0, 1]:.4f}")
    #         print(f"预测Fy值: {predict_force_raw(sample_raw_readings, 'Fy'):.4f}")
    #         print(f"实际Fz值: {y_raw[0, 2]:.4f}")
    #         print(f"预测Fz值: {predict_force_raw(sample_raw_readings, 'Fz'):.4f}")
    #     else:
    #         print("错误：没有足够的压阻传感器数据进行标定（至少需要4个）或对齐数据无效")
    # else:
    #     print("错误：力传感器数据中未找到Fx, Fy, Fz列")



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
