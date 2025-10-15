import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import json
import time

# 不准删除我任何注释！！！一个也不能删！！！！！
# 设置中文显示
plt.rcParams["font.family"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

def add_polynomial_features_raw(X, degree=2):
    """添加多项式特征，包括交互项"""
    n_samples, n_features = X.shape
    features = [X]
    
    # 添加高阶项
    for d in range(2, degree+1):
        features.append(X ** d)
    
    # 添加交互项 (两个不同特征的乘积)
    for i in range(n_features):
        for j in range(i+1, n_features):
            features.append(X[:, i] * X[:, j])
    
    return np.column_stack(features)

def least_squares_fit_raw(X, y):
    """使用最小二乘法求解线性回归系数: y = Xw"""
    # w = (X^T X)^(-1) X^T y
    # 添加微小扰动防止矩阵奇异
    X_T_X = X.T @ X
    X_T_X_reg = X_T_X + np.eye(X_T_X.shape[0]) * 1e-8
    w = np.linalg.inv(X_T_X_reg) @ X.T @ y
    return w

def calculate_error_raw(y_true, y_pred):
    """计算均方误差和决定系数R²"""
    mse = np.mean((y_true - y_pred) ** 2)
    r2 = 1 - np.sum((y_true - y_pred) ** 2) / np.sum((y_true - np.mean(y_true)) ** 2)
    return mse, r2

def predict_force_raw(sensor_readings, component='Fx', coefficients=None, degree=1):
    """
    使用原始数据标定方程预测力值
    sensor_readings: 长度为4的列表，包含group6_p1到p4的原始读数
    component: 要预测的力分量，'Fx', 'Fy'或'Fz'
    coefficients: 拟合得到的系数
    degree: 特征的多项式度数
    """
    if coefficients is None:
        raise ValueError("必须提供系数")
    
    # 构建多项式特征
    X_predict = add_polynomial_features_raw(np.array(sensor_readings).reshape(1, -1), degree=degree)
    X_predict = np.hstack([np.ones((1, 1)), X_predict])  # 添加偏置项
    
    # 预测力值（直接使用原始尺度）
    force_value = X_predict @ coefficients[component]
    
    return float(force_value)

# 读取对齐后的原始数据
try:
    aligned_data = pd.read_csv('aligned_data.csv', encoding='utf-8')
except UnicodeDecodeError:
    try:
        aligned_data = pd.read_csv('aligned_data.csv', encoding='gbk')
    except UnicodeDecodeError:
        aligned_data = pd.read_csv('aligned_data.csv', encoding='latin1')

print("已读取对齐后的原始数据")
print(f"数据形状: {aligned_data.shape}")

# 提取力传感器数据和group6数据
force_columns = ['Fx', 'Fy', 'Fz']
group6_columns = [col for col in aligned_data.columns if col.startswith('group6')]

# 确保有足够的传感器数据
if len(group6_columns) < 4:
    raise ValueError("没有足够的压阻传感器数据进行标定（至少需要4个）")

# 提取特征和目标变量
# 特征：四个压阻传感器的原始读数 (group6_p1到p4)
# 目标：Fx, Fy, Fz的原始力值
X_raw = aligned_data[group6_columns[:4]].values  # 只使用前4个传感器数据
y_raw = aligned_data[force_columns].values

# 设置多项式度数
degree = 4

# 生成多项式特征
X_poly_raw = add_polynomial_features_raw(X_raw, degree=degree)

# 添加偏置项（常数项）
X_poly_raw = np.hstack([np.ones((X_poly_raw.shape[0], 1)), X_poly_raw])

# 分别拟合三个力分量
coefficients_raw = {}
for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    print(f"使用原始数据拟合 {force_component} 的标定模型...")
    coefficients_raw[force_component] = least_squares_fit_raw(X_poly_raw, y_raw[:, i])

# 使用拟合的模型进行预测
predictions_raw = {}
total_time = 0
num_samples = X_raw.shape[0]

# 在此位置添加平均预测时间计算
print("\n====== 平均单次预测时间计算 ======")

start_time = time.perf_counter_ns()

# for force_component in ['Fx', 'Fy', 'Fz']: #矩阵计算
#     predictions_raw[force_component] = X_poly_raw @ coefficients_raw[force_component]

# 逐个样本进行预测以计算平均时间
for i in range(num_samples):
    sample_raw_readings = X_raw[i, :4]  # 前4个传感器的原始读数
    for force_component in ['Fx', 'Fy', 'Fz']:
        # 构建多项式特征
        X_predict = add_polynomial_features_raw(sample_raw_readings.reshape(1, -1), degree=degree)
        X_predict = np.hstack([np.ones((1, 1)), X_predict])  # 添加偏置项
        # 预测
        if force_component not in predictions_raw:
            predictions_raw[force_component] = []
        predictions_raw[force_component].append(float(X_predict @ coefficients_raw[force_component]))

end_time = time.perf_counter_ns()

total_prediction_time = end_time - start_time
num_samples = X_poly_raw.shape[0]
average_prediction_time = total_prediction_time / num_samples

print(f"总预测时间: {total_prediction_time} 纳秒")
print(f"样本数量: {num_samples}")
print(f"平均单次预测(Fx,Fy,Fz)时间: {average_prediction_time} 纳秒 ({average_prediction_time/1000000:.4f} 毫秒)")



# 可视化拟合结果
fig, axes = plt.subplots(3, 1, figsize=(15, 7))
fig.suptitle('原始力传感器实测值与压阻传感器预测值对比', fontsize=16)

colors = ['blue', 'green', 'red']
for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    axes[i].plot(y_raw[:, i], label=f'实测 {force_component}', color=colors[i], alpha=0.7)
    axes[i].plot(predictions_raw[force_component], label=f'预测 {force_component}', 
                 color=colors[i], linestyle='--')
    axes[i].set_title(f'{force_component} 拟合结果 (R² = {calculate_error_raw(y_raw[:, i], predictions_raw[force_component])[1]:.4f})')
    axes[i].set_xlabel('样本索引')
    axes[i].set_ylabel('力值')
    axes[i].grid(True)
    axes[i].legend()

plt.tight_layout(rect=[0, 0, 1, 0.96])  # 为suptitle留出空间
plt.show()

# 保存原始数据的标定系数
calibration_data_raw = {
    'coefficients': {k: v.tolist() for k, v in coefficients_raw.items()},
    'feature_degree': degree
}

with open('sensor_calibration_raw.json', 'w') as f:
    json.dump(calibration_data_raw, f, indent=4)

print("\n原始数据标定完成！标定系数已保存到 'sensor_calibration_raw.json'")

# 展示标定方程形式（根据degree动态生成）
print("\n====== 原始数据标定方程形式 ======")
features = ['常数项']
sensors = ['group6_p1', 'group6_p2', 'group6_p3', 'group6_p4']

# 添加一次项（1阶单项）
features.extend([f'{s}' for s in sensors])

# 添加高阶单项（2阶到degree阶）
for d in range(2, degree + 1):
    features.extend([f'{s}^{d}' for s in sensors])

# 添加交互项（两个不同特征的乘积，二次交互项）
for i in range(4):
    for j in range(i + 1, 4):
        features.append(f'{sensors[i]}×{sensors[j]}')

# 打印完整方程形式
if len(features) > 1:
    equation = f"力值 = {features[0]} + " + " + ".join([f"w_{i}×{features[i]}" for i in range(1, len(features))])
    print(equation)
print("\n其中w_i为拟合得到的系数")


print("\n====== 原始数据拟合误差分析 ======")
for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    mse, r2 = calculate_error_raw(y_raw[:, i], predictions_raw[force_component])
    rmse = np.sqrt(mse)  # 计算RMSE
    print(f"{force_component} - 均方误差: {mse:.6f}, 均方根误差: {rmse:.6f} N, 决定系数R²: {r2:.6f}")
print(f"平均单次预测(Fx,Fy,Fz)时间: {average_prediction_time} 纳秒 ({average_prediction_time/1000000:.4f} 毫秒)")



# 以下为新增的神经网络MLP预测部分
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, r2_score

# 数据标准化（对神经网络很重要）
scaler_X = StandardScaler()
scaler_y = StandardScaler()

# 使用原始特征（不添加多项式特征）
X_mlp = scaler_X.fit_transform(X_raw)
y_mlp = scaler_y.fit_transform(y_raw)

# 创建并训练MLP模型
print("\n====== 训练MLP神经网络模型 ======")
mlp_models = {}
for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    print(f"训练 {force_component} 的MLP模型...")
    # 定义MLP模型，可根据需要调整参数
    mlp = MLPRegressor(
        hidden_layer_sizes=(20, 20, 20),  # 两层隐藏层，分别有100和50个神经元
        activation='relu',             # 激活函数
        solver='adam',                 # 优化器
        max_iter=5,                  # 最大迭代次数
        random_state=42,               # 随机种子，保证结果可复现
        verbose=True                  # 打印训练过程
    )
    
    # 训练模型
    mlp.fit(X_mlp, y_mlp[:, i])
    mlp_models[force_component] = mlp

# 使用MLP模型进行预测并计算时间
print("\n====== MLP模型预测与时间计算 ======")
predictions_mlp = {}

start_time_mlp = time.perf_counter_ns()

# 逐个样本进行预测以计算平均时间
for i in range(num_samples):
    sample_raw_readings = X_raw[i, :4].reshape(1, -1)  # 前4个传感器的原始读数
    sample_scaled = scaler_X.transform(sample_raw_readings)  # 标准化
    
    for force_component in ['Fx', 'Fy', 'Fz']:
        # 预测（得到标准化后的结果）
        pred_scaled = mlp_models[force_component].predict(sample_scaled)
        # 反标准化，得到原始尺度的预测值
        # 修复：创建一个包含所有三个分量的数组，但只修改当前分量的值
        temp_y = np.zeros((1, 3))
        component_index = ['Fx', 'Fy', 'Fz'].index(force_component)
        temp_y[0, component_index] = pred_scaled[0]
        pred_original = scaler_y.inverse_transform(temp_y)[0, component_index]
        
        if force_component not in predictions_mlp:
            predictions_mlp[force_component] = []
        predictions_mlp[force_component].append(float(pred_original))

end_time_mlp = time.perf_counter_ns()

total_prediction_time_mlp = end_time_mlp - start_time_mlp
average_prediction_time_mlp = total_prediction_time_mlp / num_samples

print(f"MLP总预测时间: {total_prediction_time_mlp} 纳秒")
print(f"样本数量: {num_samples}")
print(f"MLP平均单次预测(Fx,Fy,Fz)时间: {average_prediction_time_mlp} 纳秒 ({average_prediction_time_mlp/1000000:.4f} 毫秒)")


# 可视化最小二乘法与MLP的拟合结果对比
fig, axes = plt.subplots(3, 1, figsize=(15, 10))
fig.suptitle('最小二乘法与MLP神经网络预测结果对比', fontsize=16)

colors = ['blue', 'green', 'red']
for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    # 实测值
    axes[i].plot(y_raw[:, i], label=f'实测 {force_component}', color=colors[i], alpha=0.5)
    # 最小二乘法预测值
    axes[i].plot(predictions_raw[force_component], label=f'最小二乘法预测 {force_component}', 
                 color=colors[i], linestyle='--')
    # MLP预测值
    axes[i].plot(predictions_mlp[force_component], label=f'MLP预测 {force_component}', 
                 color='purple', linestyle='-.')
    
    # 计算两种方法的R²
    ls_r2 = calculate_error_raw(y_raw[:, i], predictions_raw[force_component])[1]
    mlp_r2 = r2_score(y_raw[:, i], predictions_mlp[force_component])
    
    axes[i].set_title(f'{force_component} 预测对比 (最小二乘R² = {ls_r2:.4f}, MLP R² = {mlp_r2:.4f})')
    axes[i].set_xlabel('样本索引')
    axes[i].set_ylabel('力值')
    axes[i].grid(True)
    axes[i].legend()

plt.tight_layout(rect=[0, 0, 1, 0.96])  # 为suptitle留出空间
plt.show()

# 误差分析对比
print("\n====== 两种方法的误差对比 ======")
for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    # 最小二乘法误差
    ls_mse, ls_r2 = calculate_error_raw(y_raw[:, i], predictions_raw[force_component])
    ls_rmse = np.sqrt(ls_mse)
    
    # MLP误差
    mlp_mse = mean_squared_error(y_raw[:, i], predictions_mlp[force_component])
    mlp_rmse = np.sqrt(mlp_mse)
    mlp_r2 = r2_score(y_raw[:, i], predictions_mlp[force_component])
    
    print(f"\n{force_component} 误差对比:")
    print(f"  最小二乘法 - 均方误差: {ls_mse:.6f}, 均方根误差: {ls_rmse:.6f} N, 决定系数R²: {ls_r2:.6f}")
    print(f"  MLP神经网络 - 均方误差: {mlp_mse:.6f}, 均方根误差: {mlp_rmse:.6f} N, 决定系数R²: {mlp_r2:.6f}")

# 预测时间对比
print("\n====== 两种方法的预测时间对比 ======")
print(f"最小二乘法平均单次预测时间: {average_prediction_time} 纳秒 ({average_prediction_time/1000000:.4f} 毫秒)")
print(f"MLP神经网络平均单次预测时间: {average_prediction_time_mlp} 纳秒 ({average_prediction_time_mlp/1000000:.4f} 毫秒)")
print(f"MLP相对最小二乘法的时间倍数: {average_prediction_time_mlp / average_prediction_time:.2f}x")
