import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import json
import time
import os
import pickle
import logging
from datetime import datetime
# 新增 PyTorch 依赖
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, r2_score

# 不准删除我任何注释！！！一个也不能删！！！！！
# 设置中文显示
plt.rcParams["font.family"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# 创建带时间戳的模型保存目录和日志配置
TIMESTAMP = datetime.now().strftime('%Y%m%d_%H%M%S')
MODEL_SAVE_DIR = f"mlp_models_torch_{TIMESTAMP}"
# ===================== 训练日志配置 =====================
def setup_logging():
    """配置日志：同时输出到控制台和文件，记录训练关键信息"""
    # 创建模型保存目录（包含时间戳）
    os.makedirs(MODEL_SAVE_DIR, exist_ok=True)
    
    # 日志文件保存在模型目录内
    log_filename = f"{MODEL_SAVE_DIR}/training_log.log"
    
    # 配置日志格式
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_filename, encoding='utf-8'),  # 写入文件
            logging.StreamHandler()  # 输出到控制台
        ]
    )
    
    logger = logging.getLogger(__name__)
    logger.info("日志系统初始化完成，日志文件保存路径：%s", log_filename)
    logger.info("模型文件将保存在目录：%s", MODEL_SAVE_DIR)
    return logger

# 初始化日志器
logger = setup_logging()


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
aligned_data = pd.read_csv('aligned_data3.csv')

logger.info("已读取对齐后的原始数据")
# print("已读取对齐后的原始数据")
logger.info(f"数据形状: {aligned_data.shape}")
# print(f"数据形状: {aligned_data.shape}")

# 提取力传感器数据和group6数据
force_columns = ['Fx', 'Fy', 'Fz']
group6_columns = [col for col in aligned_data.columns if col.startswith('group6')]

# 确保有足够的传感器数据
if len(group6_columns) < 4:
    logger.error("没有足够的压阻传感器数据进行标定（至少需要4个）")
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
    logger.info(f"使用原始数据拟合 {force_component} 的标定模型...")
    # print(f"使用原始数据拟合 {force_component} 的标定模型...")
    coefficients_raw[force_component] = least_squares_fit_raw(X_poly_raw, y_raw[:, i])

# 使用拟合的模型进行预测
predictions_raw = {}
total_time = 0
num_samples = X_raw.shape[0]

# 在此位置添加平均预测时间计算
logger.info("\n====== 平均单次预测时间计算 ======")
# print("\n====== 平均单次预测时间计算 ======")

start_time = time.perf_counter_ns()

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

logger.info(f"总预测时间: {total_prediction_time} 纳秒")
logger.info(f"样本数量: {num_samples}")
logger.info(f"平均单次预测(Fx,Fy,Fz)时间: {average_prediction_time} 纳秒 ({average_prediction_time/1000000:.4f} 毫秒)")
# print(f"总预测时间: {total_prediction_time} 纳秒")
# print(f"样本数量: {num_samples}")
# print(f"平均单次预测(Fx,Fy,Fz)时间: {average_prediction_time} 纳秒 ({average_prediction_time/1000000:.4f} 毫秒)")




# # 可视化拟合结果
# fig, axes = plt.subplots(3, 1, figsize=(15, 7))
# fig.suptitle('原始力传感器实测值与压阻传感器预测值对比', fontsize=16)

# colors = ['blue', 'green', 'red']
# for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
#     axes[i].plot(y_raw[:, i], label=f'实测 {force_component}', color=colors[i], alpha=0.7)
#     axes[i].plot(predictions_raw[force_component], label=f'预测 {force_component}', 
#                  color=colors[i], linestyle='--')
#     r2 = calculate_error_raw(y_raw[:, i], predictions_raw[force_component])[1]
#     axes[i].set_title(f'{force_component} 拟合结果 (R² = {r2:.4f})')
#     axes[i].set_xlabel('样本索引')
#     axes[i].set_ylabel('力值')
#     axes[i].grid(True)
#     axes[i].legend()
#     logger.info(f"{force_component} 最小二乘拟合 R²: {r2:.6f}")

# plt.tight_layout(rect=[0, 0, 1, 0.96])  # 为suptitle留出空间
# plt.show()

# 保存原始数据的标定系数
calibration_data_raw = {
    'coefficients': {k: v.tolist() for k, v in coefficients_raw.items()},
    'feature_degree': degree
}

with open('sensor_calibration_raw.json', 'w') as f:
    json.dump(calibration_data_raw, f, indent=4)

logger.info("\n原始数据标定完成！标定系数已保存到 'sensor_calibration_raw.json'")
# print("\n原始数据标定完成！标定系数已保存到 'sensor_calibration_raw.json'")

# 展示标定方程形式（根据degree动态生成）
logger.info("\n====== 原始数据标定方程形式 ======")
# print("\n====== 原始数据标定方程形式 ======")
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
    logger.info(equation)
    # print(equation)
logger.info("\n其中w_i为拟合得到的系数")
# print("\n其中w_i为拟合得到的系数")

# 原始数据拟合误差分析
logger.info("\n====== 原始数据拟合误差分析 ======")
# print("\n====== 原始数据拟合误差分析 ======")
for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    mse, r2 = calculate_error_raw(y_raw[:, i], predictions_raw[force_component])
    rmse = np.sqrt(mse)  # 计算RMSE
    logger.info(f"{force_component} - 均方误差: {mse:.6f}, 均方根误差: {rmse:.6f} N, 决定系数R²: {r2:.6f}")
    # print(f"{force_component} - 均方误差: {mse:.6f}, 均方根误差: {rmse:.6f} N, 决定系数R²: {r2:.6f}")
logger.info(f"平均单次预测(Fx,Fy,Fz)时间: {average_prediction_time} 纳秒 ({average_prediction_time/1000000:.4f} 毫秒)")
# print(f"平均单次预测(Fx,Fy,Fz)时间: {average_prediction_time} 纳秒 ({average_prediction_time/1000000:.4f} 毫秒)")


# ===================== 替换为 PyTorch 实现的 MLP 部分（新增模型保存/加载） =====================
# 1. 数据预处理（保持与原逻辑一致的标准化）
scaler_X = StandardScaler()
scaler_y = StandardScaler()

# 标准化特征（4个传感器原始数据）和目标（3个力分量）
X_mlp = scaler_X.fit_transform(X_raw)  # (n_samples, 4)
y_mlp = scaler_y.fit_transform(y_raw)  # (n_samples, 3)

# 转换为 PyTorch 张量（CPU/GPU 自动适配）
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
logger.info(f"\n====== PyTorch MLP 训练（使用 {device} 设备）======")
# print(f"\n====== PyTorch MLP 训练（使用 {device} 设备）======")


# 2. 自定义 Dataset 类（适配 DataLoader 批量加载）
class ForceDataset(Dataset):
    def __init__(self, X_tensor, y_tensor):
        self.X = X_tensor
        self.y = y_tensor

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return self.X[idx], self.y[idx]


# 3. MLP 模型定义（结构：输入4 → 40 → 30 → 20 → 输出）
class MLPRegressorTorch(nn.Module):
    def __init__(self, input_dim=4, hidden_sizes=(40, 20, 10), output_dim=1):
        super(MLPRegressorTorch, self).__init__()
        # 构建网络层（ReLU 激活，与原逻辑一致）
        layers = []
        in_size = input_dim
        for out_size in hidden_sizes:
            layers.append(nn.Linear(in_size, out_size))
            layers.append(nn.ReLU())
            in_size = out_size
        # 输出层（无激活，回归任务）
        layers.append(nn.Linear(in_size, output_dim))
        self.model = nn.Sequential(*layers)

    def forward(self, x):
        return self.model(x)


# ===================== 新增：模型保存函数 =====================
def save_mlp_models(models, scaler_X, scaler_y, save_dir=MODEL_SAVE_DIR):
    """
    保存 PyTorch MLP 模型和标准化器
    models: 字典，key=力分量（Fx/Fy/Fz），value=MLP模型
    scaler_X: 特征标准化器（StandardScaler）
    scaler_y: 目标标准化器（StandardScaler）
    save_dir: 模型保存目录
    """
    # 创建保存目录（若不存在）
    os.makedirs(save_dir, exist_ok=True)
    logger.info(f"开始保存模型，保存目录：{save_dir}")
    
    # 1. 保存每个力分量的模型参数（状态字典，轻量化）
    for component, model in models.items():
        model_path = os.path.join(save_dir, f'{component.lower()}_mlp_model.pth')
        torch.save(model.state_dict(), model_path)
        logger.info(f"{component} 模型保存路径：{model_path}")
    
    # 2. 保存标准化器参数（使用pickle序列化）
    scaler_X_path = os.path.join(save_dir, 'scaler_X.pkl')
    scaler_y_path = os.path.join(save_dir, 'scaler_y.pkl')
    with open(scaler_X_path, 'wb') as f:
        pickle.dump(scaler_X, f)
    with open(scaler_y_path, 'wb') as f:
        pickle.dump(scaler_y, f)
    logger.info(f"特征标准化器保存路径：{scaler_X_path}")
    logger.info(f"目标标准化器保存路径：{scaler_y_path}")
    
    # 3. 保存模型结构参数（方便加载时复现结构）
    model_config = {
        'input_dim': 4,
        'hidden_sizes': (5, 5, 5),
        'output_dim': 1
    }
    config_path = os.path.join(save_dir, 'model_config.json')
    with open(config_path, 'w') as f:
        json.dump(model_config, f, indent=4)
    logger.info(f"模型结构配置保存路径：{config_path}")
    print(f"\n所有模型文件已保存到目录：{save_dir}")


# ===================== 新增：模型加载函数 =====================
def load_mlp_models(load_dir=MODEL_SAVE_DIR, device=device):
    """
    加载 PyTorch MLP 模型和标准化器
    load_dir: 模型加载目录
    device: 加载后模型运行的设备（CPU/GPU）
    return: 字典（models=模型字典, scaler_X=特征标准化器, scaler_y=目标标准化器）
    """
    # 检查目录是否存在
    if not os.path.exists(load_dir):
        logger.error(f"模型加载目录不存在：{load_dir}")
        raise FileNotFoundError(f"模型加载目录不存在：{load_dir}")
    
    logger.info(f"开始加载模型，加载目录：{load_dir}")
    
    # 1. 加载模型结构配置
    config_path = os.path.join(load_dir, 'model_config.json')
    with open(config_path, 'r') as f:
        model_config = json.load(f)
    logger.info(f"加载模型结构配置：{model_config}")
    
    # 2. 加载每个力分量的模型
    models = {}
    components = ['Fx', 'Fy', 'Fz']
    for component in components:
        model_path = os.path.join(load_dir, f'{component.lower()}_mlp_model.pth')
        # 初始化模型
        model = MLPRegressorTorch(
            input_dim=model_config['input_dim'],
            hidden_sizes=model_config['hidden_sizes'],
            output_dim=model_config['output_dim']
        ).to(device)
        # 加载模型参数
        model.load_state_dict(torch.load(model_path, map_location=device))
        # 设置为评估模式（禁用梯度计算，关闭dropout等）
        model.eval()
        models[component] = model
        logger.info(f"加载 {component} 模型完成，路径：{model_path}")
    
    # 3. 加载标准化器
    scaler_X_path = os.path.join(load_dir, 'scaler_X.pkl')
    scaler_y_path = os.path.join(load_dir, 'scaler_y.pkl')
    with open(scaler_X_path, 'rb') as f:
        scaler_X = pickle.load(f)
    with open(scaler_y_path, 'rb') as f:
        scaler_y = pickle.load(f)
    logger.info(f"加载特征标准化器完成，路径：{scaler_X_path}")
    logger.info(f"加载目标标准化器完成，路径：{scaler_y_path}")
    
    print(f"\n所有模型已从目录 {load_dir} 加载完成")
    return {'models': models, 'scaler_X': scaler_X, 'scaler_y': scaler_y}


# 4. 模型训练函数（新增日志记录）
def train_mlp_torch(component_idx, component_name, X_tensor, y_tensor, epochs=10, batch_size=32, lr=1e-3):
    """
    训练单个力分量的 MLP 模型（新增训练日志）
    component_idx: 力分量索引（0=Fx, 1=Fy, 2=Fz）
    component_name: 力分量名称（用于打印日志）
    """
    # 记录训练开始时间
    train_start_time = time.time()
    logger.info(f"{component_name} 模型训练开始，epochs={epochs}, batch_size={batch_size}, lr={lr}")
    
    # 构建 Dataset 和 DataLoader
    dataset = ForceDataset(X_tensor, y_tensor[:, component_idx].unsqueeze(1))  # y 维度：(n_samples, 1)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)
    logger.info(f"{component_name} 训练数据集大小：{len(dataset)}，批次数量：{len(dataloader)}")

    # 初始化模型、损失函数、优化器（匹配原 sklearn 的 Adam + MSE）
    model = MLPRegressorTorch().to(device)
    criterion = nn.MSELoss()  # 均方误差损失
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)

    # 训练循环
    model.train()
    for epoch in range(epochs):
        epoch_start_time = time.time()
        total_loss = 0.0
        for batch_idx, (batch_X, batch_y) in enumerate(dataloader):
            # 数据移至设备
            batch_X, batch_y = batch_X.to(device), batch_y.to(device)
            
            # 前向传播
            outputs = model(batch_X)
            loss = criterion(outputs, batch_y)
            
            # 反向传播 + 优化
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            total_loss += loss.item() * batch_X.size(0)  # 累计批次损失
        
        # 计算epoch平均损失和耗时
        avg_loss = total_loss / len(dataset)
        epoch_time = time.time() - epoch_start_time
        
        # 打印并记录日志（每3轮或首轮）
        if (epoch + 1) % 5 == 0 or epoch == 0:
            log_msg = f"  {component_name} 训练轮次 {epoch+1:3d}/{epochs} | 平均MSE损失: {avg_loss:.6f} | 耗时: {epoch_time:.2f} 秒"
            logger.info(log_msg)
            # print(log_msg)
    
    # 记录训练总耗时
    total_train_time = time.time() - train_start_time
    logger.info(f"{component_name} 模型训练完成，总耗时：{total_train_time:.2f} 秒，最终平均MSE损失：{avg_loss:.6f}")
    # print(f"  {component_name} 模型训练完成，总耗时：{total_train_time:.2f} 秒")
    
    return model


# 5. 训练3个力分量的 MLP 模型
mlp_models_torch = {}  # 存储每个力分量的模型
# 转换 numpy 数组为张量（float32 适配 PyTorch 默认精度）
X_tensor = torch.tensor(X_mlp, dtype=torch.float32).to(device)
y_tensor = torch.tensor(y_mlp, dtype=torch.float32).to(device)

# 记录整体训练开始时间
overall_train_start = time.time()
for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    logger.info(f"\n====== 开始训练 {force_component} 的 PyTorch MLP 模型 ======")
    print(f"\n训练 {force_component} 的 PyTorch MLP 模型...")
    mlp_models_torch[force_component] = train_mlp_torch(
        component_idx=i,
        component_name=force_component,
        X_tensor=X_tensor,
        y_tensor=y_tensor,
        epochs=300,  # 匹配原 max_iter=300
        batch_size=512,
        lr=1e-3
    )
# 记录整体训练总耗时
overall_train_time = time.time() - overall_train_start
logger.info(f"\n所有 MLP 模型训练完成，总耗时：{overall_train_time:.2f} 秒")
# print(f"\n所有 MLP 模型训练完成，总耗时：{overall_train_time:.2f} 秒")


# ===================== 新增：调用模型保存函数 =====================
save_mlp_models(
    models=mlp_models_torch,
    scaler_X=scaler_X,
    scaler_y=scaler_y,
    save_dir='mlp_models_torch'
)


# 6. PyTorch MLP 预测与时间计算（批量预测，与原逻辑一致）
logger.info("\n====== PyTorch MLP 预测与时间计算 ======")
# print("\n====== PyTorch MLP 预测与时间计算 ======")
predictions_mlp = {}  # 存储预测结果（与原格式一致：list）

# 预测计时（批量处理，同原 sklearn 矩阵运算逻辑）
start_time_mlp = time.perf_counter_ns()

# 切换模型到评估模式（关闭 dropout/批量归一化，仅前向传播）
for force_component in ['Fx', 'Fy', 'Fz']:
    mlp_models_torch[force_component].eval()

# 禁用梯度计算（加速预测，避免内存占用）
with torch.no_grad():
    # 批量预测所有样本
    X_scaled_tensor = torch.tensor(scaler_X.transform(X_raw), dtype=torch.float32).to(device)
    
    for force_component in ['Fx', 'Fy', 'Fz']:
        # 模型预测（标准化后的结果）
        pred_scaled_tensor = mlp_models_torch[force_component](X_scaled_tensor)
        pred_scaled = pred_scaled_tensor.cpu().numpy().squeeze()  # 转 numpy 并压缩维度
        
        # 反标准化（还原为原始力值尺度，与原逻辑一致）
        component_idx = ['Fx', 'Fy', 'Fz'].index(force_component)
        temp_y = np.zeros((len(pred_scaled), 3))
        temp_y[:, component_idx] = pred_scaled
        pred_original = scaler_y.inverse_transform(temp_y)[:, component_idx]
        
        # 转换为 list 格式（保持与原 predictions_mlp 一致）
        predictions_mlp[force_component] = pred_original.tolist()
        # 记录预测结果统计信息
        logger.info(f"{force_component} 预测完成，样本数量：{len(pred_original)}，预测值范围：[{np.min(pred_original):.2f}, {np.max(pred_original):.2f}]")

end_time_mlp = time.perf_counter_ns()

# 计算预测时间（与原逻辑一致）
total_prediction_time_mlp = end_time_mlp - start_time_mlp
average_prediction_time_mlp = total_prediction_time_mlp / num_samples

logger.info(f"PyTorch MLP 总预测时间: {total_prediction_time_mlp} 纳秒")
logger.info(f"样本数量: {num_samples}")
logger.info(f"PyTorch MLP 平均单次预测(Fx,Fy,Fz)时间: {average_prediction_time_mlp} 纳秒 ({average_prediction_time_mlp/1000000:.4f} 毫秒)")
# print(f"PyTorch MLP 总预测时间: {total_prediction_time_mlp} 纳秒")
# print(f"样本数量: {num_samples}")
# print(f"PyTorch MLP 平均单次预测(Fx,Fy,Fz)时间: {average_prediction_time_mlp} 纳秒 ({average_prediction_time_mlp/1000000:.4f} 毫秒)")


# ===================== 保留原逻辑的可视化与误差对比（新增日志记录） =====================
# 可视化最小二乘法与 PyTorch MLP 的拟合结果对比
fig, axes = plt.subplots(3, 1, figsize=(15, 9))
fig.suptitle('最小二乘法与 PyTorch MLP 神经网络预测结果对比', fontsize=16)

colors = ['blue', 'green', 'red']
for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    # 实测值
    axes[i].plot(y_raw[:, i], label=f'实测 {force_component}', color=colors[i], alpha=1.0)
    # 最小二乘法预测值
    axes[i].plot(predictions_raw[force_component], label=f'最小二乘法预测 {force_component}', 
                 color=colors[i], linestyle='--', alpha=0.5)
    # PyTorch MLP 预测值
    axes[i].plot(predictions_mlp[force_component], label=f'PyTorch MLP 预测 {force_component}', 
                 color='purple', linestyle='-.', alpha=0.7)
    
    # 计算两种方法的 R²（保持原逻辑）
    ls_r2 = calculate_error_raw(y_raw[:, i], predictions_raw[force_component])[1]
    mlp_r2 = r2_score(y_raw[:, i], predictions_mlp[force_component])
    # 记录 R² 对比日志
    logger.info(f"{force_component} 方法对比 - 最小二乘R²: {ls_r2:.6f}, PyTorch MLP R²: {mlp_r2:.6f}")
    
    axes[i].set_title(f'{force_component} 预测对比 (最小二乘R² = {ls_r2:.4f}, PyTorch MLP R² = {mlp_r2:.4f})')
    axes[i].set_xlabel('样本索引')
    axes[i].set_ylabel('力值')
    axes[i].grid(True)
    axes[i].legend()

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()

# 误差分析对比（保持原逻辑，新增日志）
logger.info("\n====== 两种方法的误差对比 ======")
# print("\n====== 两种方法的误差对比 ======")
for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    # 最小二乘法误差
    ls_mse, ls_r2 = calculate_error_raw(y_raw[:, i], predictions_raw[force_component])
    ls_rmse = np.sqrt(ls_mse)
    
    # PyTorch MLP 误差
    mlp_mse = mean_squared_error(y_raw[:, i], predictions_mlp[force_component])
    mlp_rmse = np.sqrt(mlp_mse)
    mlp_r2 = r2_score(y_raw[:, i], predictions_mlp[force_component])
    
    # 记录误差日志
    logger.info(f"\n{force_component} 误差对比:")
    logger.info(f"  最小二乘法 - 均方误差: {ls_mse:.6f}, 均方根误差: {ls_rmse:.6f} N, 决定系数R²: {ls_r2:.6f}")
    logger.info(f"  PyTorch MLP - 均方误差: {mlp_mse:.6f}, 均方根误差: {mlp_rmse:.6f} N, 决定系数R²: {mlp_r2:.6f}")
    # # 打印误差信息
    # print(f"\n{force_component} 误差对比:")
    # print(f"  最小二乘法 - 均方误差: {ls_mse:.6f}, 均方根误差: {ls_rmse:.6f} N, 决定系数R²: {ls_r2:.6f}")
    # print(f"  PyTorch MLP - 均方误差: {mlp_mse:.6f}, 均方根误差: {mlp_rmse:.6f} N, 决定系数R²: {mlp_r2:.6f}")

# 预测时间对比（保持原逻辑，新增日志）
logger.info("\n====== 两种方法的预测时间对比 ======")
# print("\n====== 两种方法的预测时间对比 ======")
time_ratio = average_prediction_time_mlp / average_prediction_time
logger.info(f"最小二乘法平均单次预测时间: {average_prediction_time} 纳秒 ({average_prediction_time/1000000:.4f} 毫秒)")
logger.info(f"PyTorch MLP 平均单次预测时间: {average_prediction_time_mlp} 纳秒 ({average_prediction_time_mlp/1000000:.4f} 毫秒)")
logger.info(f"PyTorch MLP 相对最小二乘法的时间倍数: {time_ratio:.2f}x")
# print(f"最小二乘法平均单次预测时间: {average_prediction_time} 纳秒 ({average_prediction_time/1000000:.4f} 毫秒)")
# print(f"PyTorch MLP 平均单次预测时间: {average_prediction_time_mlp} 纳秒 ({average_prediction_time_mlp/1000000:.4f} 毫秒)")
# print(f"PyTorch MLP 相对最小二乘法的时间倍数: {time_ratio:.2f}x")


# 绘制真实值vs预测值散点图（更新为 PyTorch MLP 标签，新增日志）
logger.info("\n====== 绘制真实值与预测值散点对比图 ======")
# print("\n====== 绘制真实值与预测值散点对比图 ======")
fig, axes = plt.subplots(2, 3, figsize=(15, 9))
fig.suptitle('真实值 vs 预测值 散点对比（最小二乘法·上排 | PyTorch MLP·下排）', fontsize=18)

# 样式配置（保持原逻辑）
ls_color = 'steelblue'    
mlp_color = 'darkorchid'  
diag_color = 'crimson'    
point_size = 2           
alpha = 0.2              

# 第一排：最小二乘法散点图
for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    ax = axes[0, i]
    y_true = y_raw[:, i]
    y_pred_ls = predictions_raw[force_component]
    ls_r2 = calculate_error_raw(y_true, y_pred_ls)[1]
    
    ax.scatter(y_true, y_pred_ls, color=ls_color, s=point_size, alpha=alpha, label=f'最小二乘法')
    # 完美预测对角线
    min_val = min(min(y_true), min(y_pred_ls))
    max_val = max(max(y_true), max(y_pred_ls))
    ax.plot([min_val, max_val], [min_val, max_val], color=diag_color, linestyle='--', linewidth=2, label='完美预测 (y=x)')
    
    ax.set_title(f'最小二乘法 - {force_component}\nR² = {ls_r2:.4f}', fontsize=14)
    ax.set_xlabel('真实力值 (N)', fontsize=12)
    ax.set_ylabel('预测力值 (N)', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)
    ax.axis('equal')

# 第二排：PyTorch MLP 散点图
for i, force_component in enumerate(['Fx', 'Fy', 'Fz']):
    ax = axes[1, i]
    y_true = y_raw[:, i]
    y_pred_mlp = predictions_mlp[force_component]
    mlp_r2 = r2_score(y_true, y_pred_mlp)
    
    ax.scatter(y_true, y_pred_mlp, color=mlp_color, s=point_size, alpha=alpha, label=f'PyTorch MLP')
    # 完美预测对角线
    min_val = min(min(y_true), min(y_pred_mlp))
    max_val = max(max(y_true), max(y_pred_mlp))
    ax.plot([min_val, max_val], [min_val, max_val], color=diag_color, linestyle='--', linewidth=2, label='完美预测 (y=x)')
    
    ax.set_title(f'PyTorch MLP - {force_component}\nR² = {mlp_r2:.4f}', fontsize=14)
    ax.set_xlabel('真实力值 (N)', fontsize=12)
    ax.set_ylabel('预测力值 (N)', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)
    ax.axis('equal')

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.show()
logger.info("散点对比图绘制完成！")
# print("散点对比图绘制完成！")


# ===================== 新增：模型加载示例（注释说明如何使用） =====================
# logger.info("\n====== 模型加载使用示例 ======")
# print("\n====== 模型加载使用示例（取消注释即可运行） ======")
# print("""
# # 1. 加载已保存的模型和标准化器
# loaded_data = load_mlp_models(load_dir='mlp_models_torch')
# loaded_models = loaded_data['models']
# loaded_scaler_X = loaded_data['scaler_X']
# loaded_scaler_y = loaded_data['scaler_y']

# # 2. 用加载的模型进行预测（示例：取第一个样本）
# sample_reading = X_raw[0:1]  # 单个样本（形状：(1,4)）
# sample_scaled = loaded_scaler_X.transform(sample_reading)
# sample_tensor = torch.tensor(sample_scaled, dtype=torch.float32).to(device)

# # 3. 预测每个力分量
# with torch.no_grad():
#     fx_pred = loaded_models['Fx'](sample_tensor).cpu().numpy().squeeze()
#     fy_pred = loaded_models['Fy'](sample_tensor).cpu().numpy().squeeze()
#     fz_pred = loaded_models['Fz'](sample_tensor).cpu().numpy().squeeze()

# # 4. 反标准化得到原始力值
# temp_fx = np.zeros((1,3))
# temp_fx[0,0] = fx_pred
# fx_original = loaded_scaler_y.inverse_transform(temp_fx)[0,0]

# temp_fy = np.zeros((1,3))
# temp_fy[0,1] = fy_pred
# fy_original = loaded_scaler_y.inverse_transform(temp_fy)[0,1]

# temp_fz = np.zeros((1,3))
# temp_fz[0,2] = fz_pred
# fz_original = loaded_scaler_y.inverse_transform(temp_fz)[0,2]

# print(f"加载模型预测结果 - Fx: {fx_original:.2f} N, Fy: {fy_original:.2f} N, Fz: {fz_original:.2f} N")
# """)