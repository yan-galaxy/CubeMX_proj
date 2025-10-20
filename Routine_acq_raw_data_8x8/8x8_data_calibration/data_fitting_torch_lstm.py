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
# 添加GUI相关库
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# 不准删除我任何注释！！！一个也不能删！！！！！
# 设置中文显示
plt.rcParams["font.family"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# ===================== 训练模式超参数宏定义 =====================
# 1. 基础核心参数
LSTM_WINDOW_SIZE_TRAIN = 50  # 采样率1000Hz，50ms对应50个采样点，覆盖滞后时间
POLY_DEGREE_TRAIN = 4        # 最小二乘多项式度数
MODEL_SAVE_DIR = ""          # 模型保存根目录（训练时自动加时间戳，推理时由用户选择）
logger = logging.getLogger(__name__)  # 全局日志实例

# 2. MLP训练超参数宏定义
MLP_HIDDEN_SIZES = (10, 10)  # MLP各隐藏层神经元数
MLP_EPOCHS = 300                   # MLP训练轮次
MLP_LR = 2e-3                     # MLP学习率
MLP_BATCH_SIZE = 2048              # MLP训练批次大小

# 3. LSTM训练超参数宏定义
LSTM_INPUT_SIZE_TRAIN = 4    # LSTM输入特征数（传感器数量）
LSTM_HIDDEN_SIZE_TRAIN = 16  # LSTM隐藏层维度
LSTM_NUM_LAYERS_TRAIN = 1    # LSTM层数
LSTM_EPOCHS_TRAIN = 200       # LSTM训练轮次
LSTM_LR_TRAIN = 2e-4         # LSTM学习率
LSTM_BATCH_SIZE_TRAIN = 2048  # LSTM训练批次大小


# ===================== 公共函数：GUI选择 =====================
class ModelSelectionApp:
    def __init__(self, root):
        self.root = root
        self.root.title("模型选择界面")
        self.root.geometry("500x300")
        
        # 将窗口居中显示
        self.center_window(500, 150)

        self.mode = None
        self.csv_file_path = None
        self.model_folder_path = None
        self.inference_data_path = None
        
        self.create_widgets()

    def center_window(self, width, height):
        # 获取屏幕尺寸
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        
        # 计算居中位置
        x = (screen_width - width) // 2
        y = (screen_height - height) // 2
        
        # 设置窗口大小和位置
        self.root.geometry(f'{width}x{height}+{x}+{y}')
    def create_widgets(self):
        # 主框架
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 标题
        title_label = ttk.Label(main_frame, text="请选择操作模式:", font=("Arial", 14, "bold"))
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))
        
        # 训练模式按钮
        train_button = ttk.Button(main_frame, text="训练模型", command=self.select_train_mode)
        train_button.grid(row=1, column=0, padx=10, pady=10, sticky=(tk.W, tk.E))
        
        # 推理模式按钮
        inference_button = ttk.Button(main_frame, text="推理模型", command=self.select_inference_mode)
        inference_button.grid(row=1, column=1, padx=10, pady=10, sticky=(tk.W, tk.E))
        
        # 状态标签
        self.status_label = ttk.Label(main_frame, text="", foreground="blue")
        self.status_label.grid(row=2, column=0, columnspan=2, pady=10)
        
        # 配置网格权重
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
    
    def select_train_mode(self):
        self.mode = "train"
        self.status_label.config(text="已选择: 训练新模型")
        # 选择训练CSV文件
        self.select_train_file()
    
    def select_inference_mode(self):
        self.mode = "inference"
        self.status_label.config(text="已选择: 使用已有模型进行推理")
        # 选择模型文件夹和推理数据
        self.select_inference_files()
    
    def select_train_file(self):
        file_path = filedialog.askopenfilename(
            title="选择训练数据CSV文件",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if file_path:
            self.csv_file_path = file_path
            self.root.quit()
            self.root.destroy()  # 销毁窗口
        else:
            # 用户取消了文件选择对话框，重置状态
            self.mode = None
            self.status_label.config(text="")
    
    def select_inference_files(self):
        # 先选模型文件夹，再选推理数据
        folder_path = filedialog.askdirectory(title="选择模型保存的文件夹")
        if folder_path:
            self.model_folder_path = folder_path
            file_path = filedialog.askopenfilename(
                title="选择推理数据CSV文件",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
            )
            if file_path:
                self.inference_data_path = file_path
                self.root.quit()
                self.root.destroy()  # 销毁窗口
            else:
                # 用户取消了第二个对话框，重置状态
                self.mode = None
                self.model_folder_path = None
                self.status_label.config(text="")
        else:
            # 用户取消了第一个对话框，重置状态
            self.mode = None
            self.status_label.config(text="")
            

def run_gui_selection():
    """公共函数：运行GUI选择模式与文件路径"""
    root = tk.Tk()
    app = ModelSelectionApp(root)
    
    # 处理窗口关闭事件
    def on_closing():
        app.mode = None  # 设置mode为None表示用户取消操作
        root.quit()  # 退出主循环
        root.destroy()  # 销毁窗口
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
    
    # 检查是否选择了模式，如果没有则返回None值
    if hasattr(app, 'mode') and app.mode:
        return app.mode, app.csv_file_path, app.model_folder_path, app.inference_data_path
    else:
        # 用户关闭了窗口而没有选择模式
        return None, None, None, None
    
# ===================== 公共函数：日志配置 =====================
def setup_logging(mode, model_folder_path=None):
    """公共函数：根据模式配置日志（训练带文件输出，推理仅控制台）"""
    global MODEL_SAVE_DIR
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    if mode == "train":
        # 训练模式：创建带时间戳的目录，日志同时输出到文件
        TIMESTAMP = datetime.now().strftime('%Y%m%d_%H%M%S')
        MODEL_SAVE_DIR = f"force_models_{TIMESTAMP}"
        os.makedirs(MODEL_SAVE_DIR, exist_ok=True)
        
        log_filename = f"{MODEL_SAVE_DIR}/training_log.log"
        file_handler = logging.FileHandler(log_filename, encoding='utf-8')
        file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        logger.addHandler(file_handler)
        
        logger.info("日志系统初始化完成（训练模式），日志文件：%s", log_filename)
        logger.info("模型保存目录：%s", MODEL_SAVE_DIR)
    else:
        # 推理模式：日志仅输出到控制台，使用用户选择的模型目录
        MODEL_SAVE_DIR = model_folder_path
        logger.info("日志系统初始化完成（推理模式），模型读取目录：%s", MODEL_SAVE_DIR)
    
    return logger


# ===================== 公共函数：数据处理（训练/推理共用，推理参数从文件来） =====================
def load_and_preprocess_data(mode, file_path, model_folder_path=None):
    """公共函数：加载数据并提取特征/目标（训练用宏定义，推理用模型文件参数）"""
    # 读取CSV数据
    aligned_data = pd.read_csv(file_path)
    logger.info(f"已读取{mode}数据文件：{file_path}，数据形状：{aligned_data.shape}")
    
    # 提取特征（前4个group6传感器）和目标（力分量）
    force_columns = ['Fx', 'Fy', 'Fz']
    group6_columns = [col for col in aligned_data.columns if col.startswith('group6')]
    
    if len(group6_columns) < 4:
        logger.error("没有足够的压阻传感器数据（至少需要4个）")
        raise ValueError("没有足够的压阻传感器数据（至少需要4个）")
    
    X_raw = aligned_data[group6_columns[:4]].values  # 原始特征 (n_samples, 4)
    y_raw = aligned_data[force_columns].values       # 原始目标 (n_samples, 3)
    
    # 时序窗口参数：训练用宏定义，推理从LSTM配置文件读取
    if mode == "train":
        window_size = LSTM_WINDOW_SIZE_TRAIN
    else:
        # 推理模式：从LSTM配置文件读取时序窗口（不读代码宏定义）
        lstm_config_path = os.path.join(model_folder_path, 'lstm_models', 'lstm_config.json')
        with open(lstm_config_path, 'r') as f:
            lstm_config = json.load(f)
        window_size = lstm_config['seq_len']
        logger.info(f"推理模式：从LSTM配置文件读取时序窗口={window_size}ms")
    
    # 构建LSTM时序数据（用对应模式的窗口大小）
    X_seq, y_seq = create_sequence_data(X_raw, y_raw, window_size=window_size)
    logger.info(f"LSTM时序数据构建完成：序列特征{X_seq.shape}，序列目标{y_seq.shape}（窗口大小{window_size}）")
    
    # 推理模式：加载训练好的标准化器+最小二乘多项式度数；训练模式：返回空
    scaler_X, scaler_y, poly_degree = None, None, None
    if mode == "inference":
        # 加载标准化器
        scaler_X_path = os.path.join(model_folder_path, 'mlp_models', 'scaler_X.pkl')
        scaler_y_path = os.path.join(model_folder_path, 'mlp_models', 'scaler_y.pkl')
        with open(scaler_X_path, 'rb') as f:
            scaler_X = pickle.load(f)
        with open(scaler_y_path, 'rb') as f:
            scaler_y = pickle.load(f)
        # 加载最小二乘多项式度数（不读代码宏定义）
        calib_path = os.path.join(model_folder_path, 'sensor_calibration_raw.json')
        with open(calib_path, 'r') as f:
            calib_data = json.load(f)
        poly_degree = calib_data['feature_degree']
        logger.info(f"推理模式：从校准文件读取多项式度数={poly_degree}")
        logger.info("推理模式：成功加载标准化器")
    
    return X_raw, y_raw, X_seq, y_seq, scaler_X, scaler_y, poly_degree, window_size

def create_sequence_data(X, y, window_size):
    """公共函数：构建LSTM时序序列（窗口大小由外部传入，训练用宏定义，推理用文件参数）"""
    n_samples = X.shape[0]
    n_seq = n_samples - window_size + 1
    
    X_seq = np.zeros((n_seq, window_size, X.shape[1]))
    y_seq = np.zeros((n_seq, y.shape[1]))
    
    for i in range(n_seq):
        X_seq[i] = X[i:i+window_size, :]
        y_seq[i] = y[i+window_size-1, :]
    
    return X_seq, y_seq

def add_polynomial_features_raw(X, degree):
    """公共函数：添加多项式特征（度数由外部传入，训练用宏定义，推理用文件参数）"""
    n_samples, n_features = X.shape
    features = [X]
    
    # 高阶项
    for d in range(2, degree+1):
        features.append(X ** d)
    
    # 交互项
    for i in range(n_features):
        for j in range(i+1, n_features):
            features.append(X[:, i] * X[:, j])
    
    return np.column_stack(features)


# ===================== 公共函数：模型类定义（训练/推理共用） =====================
# MLP模型类（训练/推理共用）
class MLPRegressorTorch(nn.Module):
    def __init__(self, input_dim=4, hidden_sizes=(40, 20, 10), output_dim=1):
        super(MLPRegressorTorch, self).__init__()
        layers = []
        in_size = input_dim
        for out_size in hidden_sizes:
            layers.append(nn.Linear(in_size, out_size))
            layers.append(nn.ReLU())
            in_size = out_size
        layers.append(nn.Linear(in_size, output_dim))
        self.model = nn.Sequential(*layers)

    def forward(self, x):
        return self.model(x)

# LSTM模型类（训练/推理共用）
class LSTMRegressorTorch(nn.Module):
    def __init__(self, input_size=4, hidden_size=64, num_layers=2, seq_len=50, output_dim=1):
        super(LSTMRegressorTorch, self).__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        
        self.lstm = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            bidirectional=False
        )
        self.fc = nn.Linear(hidden_size, output_dim)

    def forward(self, x):
        # 初始化隐藏状态和细胞状态，形状：(num_layers, batch_size, hidden_size)
        h0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size, device=x.device)
        c0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size, device=x.device)
        
        outputs, (hn, cn) = self.lstm(x, (h0, c0))
        last_seq_output = outputs[:, -1, :]  # 取每个序列的最后一个时间步输出
        pred = self.fc(last_seq_output)
        return pred

# Dataset类（训练/推理共用）
class ForceDataset(Dataset):
    def __init__(self, X_tensor, y_tensor):
        self.X = X_tensor
        self.y = y_tensor

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return self.X[idx], self.y[idx]

class LSTMForceDataset(Dataset):
    def __init__(self, X_seq_tensor, y_seq_tensor=None):
        self.X_seq = X_seq_tensor
        self.y_seq = y_seq_tensor  # 允许y_seq为None（仅预测时用）

    def __len__(self):
        return len(self.X_seq)

    def __getitem__(self, idx):
        if self.y_seq is not None:
            return self.X_seq[idx], self.y_seq[idx]
        return self.X_seq[idx]  # 仅预测时返回输入


# ===================== 模型保存与加载（训练/推理对应） =====================
def save_models(mlp_models, lstm_models, scaler_X, scaler_y):
    """训练后保存所有模型（保存训练用宏定义参数，供推理读取）"""
    # 保存MLP模型
    mlp_save_dir = os.path.join(MODEL_SAVE_DIR, 'mlp_models')
    os.makedirs(mlp_save_dir, exist_ok=True)
    for component, model in mlp_models.items():
        torch.save(model.state_dict(), os.path.join(mlp_save_dir, f'{component.lower()}_mlp.pth'))
    # 保存MLP配置（训练用宏定义参数）
    mlp_config = {
        'input_dim': LSTM_INPUT_SIZE_TRAIN,
        'hidden_sizes': MLP_HIDDEN_SIZES,
        'output_dim': 1
    }
    with open(os.path.join(mlp_save_dir, 'mlp_config.json'), 'w') as f:
        json.dump(mlp_config, f, indent=4)
    # 保存MLP标准化器
    with open(os.path.join(mlp_save_dir, 'scaler_X.pkl'), 'wb') as f:
        pickle.dump(scaler_X, f)
    with open(os.path.join(mlp_save_dir, 'scaler_y.pkl'), 'wb') as f:
        pickle.dump(scaler_y, f)
    logger.info("MLP模型保存完成：%s", mlp_save_dir)

    # 保存LSTM模型
    lstm_save_dir = os.path.join(MODEL_SAVE_DIR, 'lstm_models')
    os.makedirs(lstm_save_dir, exist_ok=True)
    for component, model in lstm_models.items():
        torch.save(model.state_dict(), os.path.join(lstm_save_dir, f'{component.lower()}_lstm.pth'))
    # 保存LSTM配置（训练用宏定义参数，含时序窗口）
    lstm_config = {
        'input_size': LSTM_INPUT_SIZE_TRAIN,
        'hidden_size': LSTM_HIDDEN_SIZE_TRAIN,
        'num_layers': LSTM_NUM_LAYERS_TRAIN,
        'seq_len': LSTM_WINDOW_SIZE_TRAIN,  # 保存训练时的窗口大小，供推理用
        'output_dim': 1
    }
    with open(os.path.join(lstm_save_dir, 'lstm_config.json'), 'w') as f:
        json.dump(lstm_config, f, indent=4)
    logger.info("LSTM模型保存完成：%s", lstm_save_dir)

    # 保存最小二乘系数+多项式度数（训练用宏定义）
    calibration_data = {
        'coefficients': coefficients_raw,
        'feature_degree': POLY_DEGREE_TRAIN  # 保存训练时的多项式度数，供推理用
    }
    with open(f'{MODEL_SAVE_DIR}/sensor_calibration_raw.json', 'w') as f:
        json.dump({k: {kk: vv.tolist() for kk, vv in v.items()} if isinstance(v, dict) else v for k, v in calibration_data.items()}, f, indent=4)
    logger.info("最小二乘系数保存完成：sensor_calibration_raw.json")

def load_models(mode, model_folder_path):
    """推理前加载所有模型（所有参数从文件读取，不读代码宏定义）"""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    # 加载MLP模型（参数从mlp_config.json读取）
    mlp_models = {}
    mlp_config_path = os.path.join(model_folder_path, 'mlp_models', 'mlp_config.json')
    with open(mlp_config_path, 'r') as f:
        mlp_config = json.load(f)  # 推理用文件参数，不读宏定义
    for component in ['Fx', 'Fy', 'Fz']:
        model_path = os.path.join(model_folder_path, 'mlp_models', f'{component.lower()}_mlp.pth')
        model = MLPRegressorTorch(**mlp_config).to(device)  # 用文件参数初始化
        model.load_state_dict(torch.load(model_path, map_location=device))
        model.eval()
        mlp_models[component] = model
    logger.info("MLP模型加载完成")

    # 加载LSTM模型（参数从lstm_config.json读取，含时序窗口）
    lstm_models = {}
    lstm_config_path = os.path.join(model_folder_path, 'lstm_models', 'lstm_config.json')
    with open(lstm_config_path, 'r') as f:
        lstm_config = json.load(f)  # 推理用文件参数，不读宏定义
    for component in ['Fx', 'Fy', 'Fz']:
        model_path = os.path.join(model_folder_path, 'lstm_models', f'{component.lower()}_lstm.pth')
        model = LSTMRegressorTorch(**lstm_config).to(device)  # 用文件参数初始化
        model.load_state_dict(torch.load(model_path, map_location=device))
        model.eval()
        lstm_models[component] = model
    logger.info("LSTM模型加载完成")

    # 加载最小二乘系数+多项式度数（从校准文件读取）
    calib_path = os.path.join(model_folder_path, 'sensor_calibration_raw.json')
    with open(calib_path, 'r') as f:
        calib_data = json.load(f)
    coefficients = {k: np.array(v) for k, v in calib_data['coefficients'].items()}
    poly_degree = calib_data['feature_degree']  # 推理用文件度数，不读宏定义
    logger.info("最小二乘系数+多项式度数加载完成")

    return mlp_models, lstm_models, coefficients, poly_degree, lstm_config['seq_len'], device


# ===================== 公共函数：预测与误差计算（训练/推理共用） =====================
def predict_lstm_batch(model, X_seq_tensor, batch_size, device):
    """LSTM分批次预测工具函数：避免全量输入导致内存溢出"""
    model.eval()
    y_pred_scaled = []
    # 创建仅含输入的Dataset（无需标签）
    dataset = LSTMForceDataset(X_seq_tensor)
    # 用指定批次大小创建DataLoader（shuffle=False保持顺序）
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=False)
    
    with torch.no_grad():
        for batch_X in dataloader:
            batch_X = batch_X.to(device)
            batch_pred = model(batch_X).cpu().numpy().squeeze()
            y_pred_scaled.extend(batch_pred.tolist())  # 收集批次预测结果
    
    return np.array(y_pred_scaled)

def predict_all_methods(mode, X_raw, X_seq, scaler_X, scaler_y, mlp_models, lstm_models, coefficients, poly_degree, window_size, device):
    """公共函数：三种方法预测（LSTM改用分批次，避免内存溢出）"""
    num_samples_raw = X_raw.shape[0]
    num_samples_seq = X_seq.shape[0] if X_seq is not None else 0
    predictions = {'最小二乘': {}, 'MLP': {}, 'LSTM': {}}

    # 1. 最小二乘预测（多项式度数由外部传入）
    start_time_ls = time.perf_counter_ns()
    for component in ['Fx', 'Fy', 'Fz']:
        preds = []
        for i in range(num_samples_raw):
            X_poly = add_polynomial_features_raw(X_raw[i:i+1, :], degree=poly_degree)
            X_poly = np.hstack([np.ones((1,1)), X_poly])
            preds.append(float(X_poly @ coefficients[component]))
        predictions['最小二乘'][component] = preds
    end_time_ls = time.perf_counter_ns()
    avg_time_ls = (end_time_ls - start_time_ls) / num_samples_raw
    logger.info(f"最小二乘预测完成：平均时间{avg_time_ls/1000000:.4f}ms（多项式度数{poly_degree}）")

    # 2. MLP预测（无额外参数，依赖模型本身）
    start_time_mlp = time.perf_counter_ns()
    X_mlp_scaled = scaler_X.transform(X_raw)
    X_mlp_tensor = torch.tensor(X_mlp_scaled, dtype=torch.float32).to(device)
    
    with torch.no_grad():
        for component in ['Fx', 'Fy', 'Fz']:
            # MLP分批次预测（若X_mlp_tensor过大，可参考LSTM用DataLoader）
            pred_scaled = mlp_models[component](X_mlp_tensor).cpu().numpy().squeeze()
            # 反标准化
            idx = ['Fx', 'Fy', 'Fz'].index(component)
            temp_y = np.zeros((len(pred_scaled), 3))
            temp_y[:, idx] = pred_scaled
            pred_original = scaler_y.inverse_transform(temp_y)[:, idx]
            predictions['MLP'][component] = pred_original.tolist()
    
    end_time_mlp = time.perf_counter_ns()
    avg_time_mlp = (end_time_mlp - start_time_mlp) / num_samples_raw
    logger.info(f"MLP预测完成：平均时间{avg_time_mlp/1000000:.4f}ms")

    # 3. LSTM预测（改用分批次，避免内存溢出）
    start_time_lstm = time.perf_counter_ns()
    # 前window_size-1个样本用最小二乘填充
    for component in ['Fx', 'Fy', 'Fz']:
        predictions['LSTM'][component] = predictions['最小二乘'][component][:window_size-1].copy()
    
    # LSTM序列数据标准化
    X_seq_scaled = scaler_X.transform(X_seq.reshape(-1, X_seq.shape[-1])).reshape(X_seq.shape)
    X_seq_tensor = torch.tensor(X_seq_scaled, dtype=torch.float32)
    
    # 确定LSTM批次大小（训练模式用训练批次，推理模式用相同逻辑）
    lstm_batch_size = LSTM_BATCH_SIZE_TRAIN if mode == "train" else 128  # 推理默认128，可调整
    logger.info(f"LSTM分批次预测：批次大小={lstm_batch_size}，总序列数={num_samples_seq}")
    
    with torch.no_grad():
        # 分批次预测LSTM序列
        for component in ['Fx', 'Fy', 'Fz']:
            # 调用分批次预测工具函数
            y_pred_scaled = predict_lstm_batch(
                model=lstm_models[component],
                X_seq_tensor=X_seq_tensor,
                batch_size=lstm_batch_size,
                device=device
            )
            # 反标准化到原始力值尺度
            idx = ['Fx', 'Fy', 'Fz'].index(component)
            temp_y = np.zeros((len(y_pred_scaled), 3))
            temp_y[:, idx] = y_pred_scaled
            y_pred_original = scaler_y.inverse_transform(temp_y)[:, idx]
            # 拼接填充部分和预测部分
            predictions['LSTM'][component].extend(y_pred_original.tolist())
    
    # 清理GPU缓存，减少内存占用
    torch.cuda.empty_cache()
    end_time_lstm = time.perf_counter_ns()
    avg_time_lstm = (end_time_lstm - start_time_lstm) / num_samples_seq
    logger.info(f"LSTM预测完成：平均时间{avg_time_lstm/1000000:.4f}ms（时序窗口{window_size}，批次{lstm_batch_size}）")

    # 整理时间统计
    time_metrics = {
        '最小二乘': avg_time_ls,
        'MLP': avg_time_mlp,
        'LSTM': avg_time_lstm
    }
    return predictions, time_metrics

def calculate_error(y_true, predictions):
    """公共函数：计算三种方法的误差（训练/推理共用）"""
    error_metrics = {}
    for method, preds in predictions.items():
        error_metrics[method] = {}
        for i, component in enumerate(['Fx', 'Fy', 'Fz']):
            y_pred = np.array(preds[component])[:len(y_true)]
            mse = mean_squared_error(y_true[:, i], y_pred)
            rmse = np.sqrt(mse)
            r2 = r2_score(y_true[:, i], y_pred)
            error_metrics[method][component] = {'mse': mse, 'rmse': rmse, 'r2': r2}
            logger.info(f"\n{component}-{method}：MSE={mse:.6f} (N²)，RMSE={rmse:.6f} N，R²={r2:.6f}")
    return error_metrics

def calculate_single_model_error(y_true, y_pred, model_name, component):
    """公共函数：计算单个模型的误差（用于训练过程中单独输出）"""
    mse = mean_squared_error(y_true, y_pred)
    rmse = np.sqrt(mse)
    r2 = r2_score(y_true, y_pred)
    logger.info(f"{component}-{model_name} 训练误差：MSE={mse:.6f} (N²)，RMSE={rmse:.6f} N，R²={r2:.6f}")
    return {'mse': mse, 'rmse': rmse, 'r2': r2}


# ===================== 公共函数：可视化（训练/推理共用） =====================
def plot_results(y_true, predictions, error_metrics, mode, window_size):
    """公共函数：绘制时序对比图与散点图（窗口大小用于标注，训练宏定义/推理文件）"""
    num_samples_raw = y_true.shape[0]
    colors = {'实测': 'r', '最小二乘': 'g', 'MLP': 'c', 'LSTM': 'm'}
    linestyles = {'实测': '--', '最小二乘': '-', 'MLP': '-', 'LSTM': '-'}

    # 1. 时序对比图（标题补充MSE/RMSE+窗口大小）
    fig, axes = plt.subplots(3, 1, figsize=(15, 8)) # 不准改figsize大小！！！！！figsize就固定了！！！！
    fig.suptitle(f'传感器力值预测对比（{mode}模式，LSTM窗口{window_size}ms）', fontsize=18, y=0.98)
    
    for i, component in enumerate(['Fx', 'Fy', 'Fz']):
        # 绘制实测值
        axes[i].plot(range(num_samples_raw), y_true[:, i], 
                     label='实测', color=colors['实测'], linestyle=linestyles['实测'], linewidth=2)
        # 绘制各方法预测值
        for method, preds in predictions.items():
            y_pred = np.array(preds[component])[:num_samples_raw]
            axes[i].plot(range(num_samples_raw), y_pred,
                         label=method, color=colors[method], linewidth=1.5, alpha=0.8)
        
        # 提取当前分量的所有误差指标
        ls_mse = error_metrics['最小二乘'][component]['mse']
        ls_rmse = error_metrics['最小二乘'][component]['rmse']
        ls_r2 = error_metrics['最小二乘'][component]['r2']
        
        mlp_mse = error_metrics['MLP'][component]['mse']
        mlp_rmse = error_metrics['MLP'][component]['rmse']
        mlp_r2 = error_metrics['MLP'][component]['r2']
        
        lstm_mse = error_metrics['LSTM'][component]['mse']
        lstm_rmse = error_metrics['LSTM'][component]['rmse']
        lstm_r2 = error_metrics['LSTM'][component]['r2']
        
        # 标题显示MSE/RMSE/R²（分行避免拥挤）
        axes[i].set_title(
            f'{component} 预测对比\n'
            f'最小二乘：MSE={ls_mse:.4f} | RMSE={ls_rmse:.4f}N | R²={ls_r2:.4f} | '
            f'MLP：MSE={mlp_mse:.4f} | RMSE={mlp_rmse:.4f}N | R²={mlp_r2:.4f} | '
            f'LSTM：MSE={lstm_mse:.4f} | RMSE={lstm_rmse:.4f}N | R²={lstm_r2:.4f}',
            fontsize=12, pad=10
        )
        axes[i].set_xlabel('样本索引（采样率1000Hz，单位：ms）', fontsize=11)
        axes[i].set_ylabel('力值 (N)', fontsize=11)
        axes[i].grid(True, alpha=0.3)
        axes[i].legend(fontsize=9, loc='upper right')
        # 时间轴标注（毫秒）
        axes[i].set_xticks(range(0, num_samples_raw+1, max(1, num_samples_raw//20)))
        axes[i].set_xticklabels([f'{t}' for t in range(0, num_samples_raw+1, max(1, num_samples_raw//20))])

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    if mode == "train":
        plt.savefig(f'{MODEL_SAVE_DIR}/force_time_contrast.png', dpi=300, bbox_inches='tight')
        logger.info("时序对比图保存完成：force_time_contrast.png")
    plt.show()

    # 2. 散点对比图（标题补充MSE/RMSE）
    fig, axes = plt.subplots(3, 3, figsize=(15, 8)) # 不准改figsize大小！！！！！figsize就固定了！！！！
    fig.suptitle(f'真实值vs预测值散点对比（{mode}模式，LSTM窗口{window_size}ms）', fontsize=18, y=0.98)
    method_rows = {'最小二乘': 0, 'MLP': 1, 'LSTM': 2}
    
    for method, row_idx in method_rows.items():
        for col_idx, component in enumerate(['Fx', 'Fy', 'Fz']):
            ax = axes[row_idx, col_idx]
            y_pred = np.array(predictions[method][component])[:num_samples_raw]
            # 提取当前方法-分量的误差
            mse = error_metrics[method][component]['mse']
            rmse = error_metrics[method][component]['rmse']
            r2 = error_metrics[method][component]['r2']
            
            # 散点与对角线
            ax.scatter(y_true[:, col_idx], y_pred, color=colors[method], s=1, alpha=0.1)
            min_val = min(y_true[:, col_idx].min(), y_pred.min())
            max_val = max(y_true[:, col_idx].max(), y_pred.max())
            ax.plot([min_val, max_val], [min_val, max_val], 'k--', linewidth=2, label='完美预测(y=x)')
            
            # 标题显示MSE/RMSE/R²
            ax.set_title(
                f'{method}-{component}\n'
                f'MSE={mse:.4f} (N²) | RMSE={rmse:.4f} N | R²={r2:.4f}',
                fontsize=11
            )
            ax.set_xlabel('真实力值 (N)', fontsize=9)
            ax.set_ylabel('预测力值 (N)', fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)
            ax.axis('equal')
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    if mode == "train":
        plt.savefig(f'{MODEL_SAVE_DIR}/force_scatter_contrast.png', dpi=300, bbox_inches='tight')
        logger.info("散点对比图保存完成：force_scatter_contrast.png")
    plt.show()


# ===================== 训练模式核心逻辑 =====================
def run_train_mode(csv_file_path):
    """训练模式：完全使用开头宏定义参数，LSTM预测改用分批次，避免内存溢出"""
    global coefficients_raw
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logger.info(f"\n====== 开始训练模式（使用{device}设备）======")
    # 日志输出训练超参数（全部来自开头宏定义）
    logger.info("=== 训练超参数（全部来自全局宏定义） ===")
    logger.info(f"MLP：隐藏层神经元{MLP_HIDDEN_SIZES} | 轮次{MLP_EPOCHS} | 学习率{MLP_LR} | 批次{MLP_BATCH_SIZE}")
    logger.info(f"LSTM：输入维度{LSTM_INPUT_SIZE_TRAIN} | 隐藏维度{LSTM_HIDDEN_SIZE_TRAIN} | 层数{LSTM_NUM_LAYERS_TRAIN} | 轮次{LSTM_EPOCHS_TRAIN} | 学习率{LSTM_LR_TRAIN} | 批次{LSTM_BATCH_SIZE_TRAIN} | 窗口{LSTM_WINDOW_SIZE_TRAIN}ms")
    logger.info(f"最小二乘：多项式度数{POLY_DEGREE_TRAIN}")

    # 1. 数据加载与预处理（训练用宏定义参数）
    X_raw, y_raw, X_seq, y_seq, _, _, _, _ = load_and_preprocess_data(
        mode="train",
        file_path=csv_file_path,
        model_folder_path=None  # 训练模式不用模型文件夹
    )
    # 训练模式：初始化并拟合标准化器
    scaler_X = StandardScaler()
    scaler_y = StandardScaler()
    X_mlp_scaled = scaler_X.fit_transform(X_raw)
    y_mlp_scaled = scaler_y.fit_transform(y_raw)
    # 用训练宏定义的窗口大小处理时序数据
    X_seq_scaled = scaler_X.transform(X_seq.reshape(-1, LSTM_INPUT_SIZE_TRAIN)).reshape(-1, LSTM_WINDOW_SIZE_TRAIN, LSTM_INPUT_SIZE_TRAIN)
    y_seq_scaled = scaler_y.transform(y_seq)
    X_seq_tensor = torch.tensor(X_seq_scaled, dtype=torch.float32).to(device)
    y_seq_tensor = torch.tensor(y_seq_scaled, dtype=torch.float32).to(device)

    # 2. 最小二乘模型训练（用训练宏定义的多项式度数）
    logger.info("\n====== 训练最小二乘模型 ======")
    logger.info(f"最小二乘多项式特征配置：度数={POLY_DEGREE_TRAIN}")
    X_poly = add_polynomial_features_raw(X_raw, degree=POLY_DEGREE_TRAIN)
    X_poly = np.hstack([np.ones((X_poly.shape[0], 1)), X_poly])
    coefficients_raw = {}
    # 训练+预测+误差计算
    for i, component in enumerate(['Fx', 'Fy', 'Fz']):
        # 带正则化的最小二乘拟合
        X_T_X = X_poly.T @ X_poly
        X_T_X_reg = X_T_X + np.eye(X_T_X.shape[0]) * 1e-8
        coefficients_raw[component] = np.linalg.inv(X_T_X_reg) @ X_poly.T @ y_raw[:, i]
        # 训练数据预测
        y_pred = X_poly @ coefficients_raw[component]
        # 计算并日志输出误差
        calculate_single_model_error(y_raw[:, i], y_pred, "最小二乘", component)
    logger.info("最小二乘模型训练完成")

    # 3. MLP模型训练（完全用训练宏定义参数）
    logger.info("\n====== 训练MLP模型 ======")
    mlp_num_layers = len(MLP_HIDDEN_SIZES)  # 由宏定义推导
    # 记录MLP详细参数到日志（来自宏定义）
    logger.info(f"MLP模型配置：")
    logger.info(f"  输入层维度：{LSTM_INPUT_SIZE_TRAIN}（传感器数量）")
    logger.info(f"  隐藏层数量：{mlp_num_layers}层")
    logger.info(f"  各层神经元数：{MLP_HIDDEN_SIZES}")
    logger.info(f"  输出层维度：1（单个力分量）")
    logger.info(f"  训练参数：epochs={MLP_EPOCHS}，batch_size={MLP_BATCH_SIZE}，学习率={MLP_LR}")
    
    mlp_models = {}
    X_mlp_tensor = torch.tensor(X_mlp_scaled, dtype=torch.float32).to(device)
    y_mlp_tensor = torch.tensor(y_mlp_scaled, dtype=torch.float32).to(device)
    
    for i, component in enumerate(['Fx', 'Fy', 'Fz']):
        dataset = ForceDataset(X_mlp_tensor, y_mlp_tensor[:, i].unsqueeze(1))
        dataloader = DataLoader(dataset, batch_size=MLP_BATCH_SIZE, shuffle=True)  # 宏定义批次
        
        model = MLPRegressorTorch(
            input_dim=LSTM_INPUT_SIZE_TRAIN,
            hidden_sizes=MLP_HIDDEN_SIZES
        ).to(device)  # 宏定义参数初始化
        criterion = nn.MSELoss()
        optimizer = torch.optim.Adam(model.parameters(), lr=MLP_LR)  # 宏定义学习率
        
        # 训练循环（宏定义轮次）
        model.train()
        for epoch in range(MLP_EPOCHS):
            total_loss = 0.0
            for batch_X, batch_y in dataloader:
                outputs = model(batch_X)
                loss = criterion(outputs, batch_y)
                
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                
                total_loss += loss.item() * batch_X.size(0)
            
            avg_loss = total_loss / len(dataset)
            # 每10轮输出训练损失（标准化后）
            if (epoch + 1) % 10 == 0:
                logger.info(f"  {component} MLP：轮次{epoch+1}/{MLP_EPOCHS}，标准化MSE={avg_loss:.6f}")
        
        # MLP训练后误差计算（若内存不足，可参考LSTM改用分批次）
        model.eval()
        with torch.no_grad():
            y_pred_scaled = model(X_mlp_tensor).cpu().numpy().squeeze()
            # 反标准化到原始力值尺度
            temp_y = np.zeros((len(y_pred_scaled), 3))
            temp_y[:, i] = y_pred_scaled
            y_pred_original = scaler_y.inverse_transform(temp_y)[:, i]
        # 计算并日志输出误差
        calculate_single_model_error(y_raw[:, i], y_pred_original, "MLP", component)
        mlp_models[component] = model
    logger.info("MLP模型训练完成")

    # 4. LSTM模型训练（完全用训练宏定义参数，误差计算改用分批次）
    logger.info("\n====== 训练LSTM模型 ======")
    # 记录LSTM详细参数到日志（来自宏定义）
    logger.info(f"LSTM模型配置：")
    logger.info(f"  时序窗口长度：{LSTM_WINDOW_SIZE_TRAIN}（对应{LSTM_WINDOW_SIZE_TRAIN}ms，采样率1000Hz）")
    logger.info(f"  输入层维度：{LSTM_INPUT_SIZE_TRAIN}（传感器数量）")
    logger.info(f"  LSTM层数：{LSTM_NUM_LAYERS_TRAIN}层")
    logger.info(f"  隐藏层维度：{LSTM_HIDDEN_SIZE_TRAIN}")
    logger.info(f"  输出层维度：1（单个力分量）")
    logger.info(f"  训练参数：epochs={LSTM_EPOCHS_TRAIN}，batch_size={LSTM_BATCH_SIZE_TRAIN}，学习率={LSTM_LR_TRAIN}")
    
    lstm_models = {}
    
    for i, component in enumerate(['Fx', 'Fy', 'Fz']):
        dataset = LSTMForceDataset(X_seq_tensor, y_seq_tensor[:, i].unsqueeze(1))
        dataloader = DataLoader(dataset, batch_size=LSTM_BATCH_SIZE_TRAIN, shuffle=True)  # 宏定义批次
        
        model = LSTMRegressorTorch(
            input_size=LSTM_INPUT_SIZE_TRAIN,
            hidden_size=LSTM_HIDDEN_SIZE_TRAIN,
            num_layers=LSTM_NUM_LAYERS_TRAIN,
            seq_len=LSTM_WINDOW_SIZE_TRAIN
        ).to(device)  # 宏定义参数初始化
        criterion = nn.MSELoss()
        optimizer = torch.optim.Adam(model.parameters(), lr=LSTM_LR_TRAIN)  # 宏定义学习率
        
        # 训练循环（宏定义轮次）
        model.train()
        for epoch in range(LSTM_EPOCHS_TRAIN):
            total_loss = 0.0
            for batch_X, batch_y in dataloader:
                outputs = model(batch_X)
                loss = criterion(outputs, batch_y)
                
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                
                total_loss += loss.item() * batch_X.size(0)
            
            avg_loss = total_loss / len(dataset)
            # 每10轮输出训练损失（标准化后）
            if (epoch + 1) % 10 == 0:
                logger.info(f"  {component} LSTM：轮次{epoch+1}/{LSTM_EPOCHS_TRAIN}，标准化MSE={avg_loss:.6f}")
        
        # LSTM训练后误差计算：改用分批次预测（核心修改！避免内存溢出）
        logger.info(f"  {component} LSTM：开始分批次计算训练误差")
        y_pred_scaled = predict_lstm_batch(
            model=model,
            X_seq_tensor=X_seq_tensor,
            batch_size=LSTM_BATCH_SIZE_TRAIN,
            device=device
        )
        # 反标准化到原始力值尺度
        temp_y = np.zeros((len(y_pred_scaled), 3))
        temp_y[:, i] = y_pred_scaled
        y_pred_original = scaler_y.inverse_transform(temp_y)[:, i]
        # LSTM真实值：原始数据的[window-1:]部分（宏定义窗口）
        y_true_lstm = y_raw[LSTM_WINDOW_SIZE_TRAIN-1:, i]
        # 计算并日志输出误差
        calculate_single_model_error(y_true_lstm, y_pred_original, "LSTM", component)
        lstm_models[component] = model
        # 清理该组件的GPU缓存，避免累积
        torch.cuda.empty_cache()
    logger.info("LSTM模型训练完成")

    # 5. 模型保存（保存宏定义参数，供推理读取）
    save_models(mlp_models, lstm_models, scaler_X, scaler_y)

    # 6. 预测与评估（用训练宏定义参数，LSTM分批次）
    logger.info("\n====== 训练模式：全局预测与误差对比 ======")
    predictions, time_metrics = predict_all_methods(
        mode="train",
        X_raw=X_raw,
        X_seq=X_seq,
        scaler_X=scaler_X,
        scaler_y=scaler_y,
        mlp_models=mlp_models,
        lstm_models=lstm_models,
        coefficients=coefficients_raw,
        poly_degree=POLY_DEGREE_TRAIN,  # 宏定义度数
        window_size=LSTM_WINDOW_SIZE_TRAIN,  # 宏定义窗口
        device=device
    )

    # 7. 误差计算与时间对比（全局日志）
    error_metrics = calculate_error(y_raw, predictions)
    logger.info("\n====== 预测时间对比 ======")
    for method, avg_time in time_metrics.items():
        logger.info(f"{method}：平均单次预测时间={avg_time:.0f}纳秒（{avg_time/1000000:.4f}ms）")

    # 8. 可视化（用训练宏定义窗口大小）
    plot_results(y_raw, predictions, error_metrics, mode="train", window_size=LSTM_WINDOW_SIZE_TRAIN)
    logger.info("\n====== 训练模式完成！所有结果保存至：%s ======", MODEL_SAVE_DIR)


# ===================== 推理模式核心逻辑 =====================
def run_inference_mode(model_folder_path, inference_data_path):
    """推理模式：所有参数从模型文件读取，LSTM分批次预测"""
    logger.info(f"\n====== 开始推理模式 ======")
    logger.info("推理模式：所有参数从模型文件读取，LSTM采用分批次预测")

    # 1. 数据加载与预处理（从模型文件获取多项式度数、时序窗口）
    X_raw, y_raw, X_seq, y_seq, scaler_X, scaler_y, poly_degree, window_size = load_and_preprocess_data(
        mode="inference",
        file_path=inference_data_path,
        model_folder_path=model_folder_path
    )

    # 2. 加载模型+所有参数（多项式度数、时序窗口、模型结构，均来自文件）
    mlp_models, lstm_models, coefficients, _, _, device = load_models(
        mode="inference",
        model_folder_path=model_folder_path
    )

    # 3. 预测与评估（用从文件读取的参数，LSTM分批次）
    logger.info("\n====== 推理模式：预测与误差评估 ======")
    predictions, time_metrics = predict_all_methods(
        mode="inference",
        X_raw=X_raw,
        X_seq=X_seq,
        scaler_X=scaler_X,
        scaler_y=scaler_y,
        mlp_models=mlp_models,
        lstm_models=lstm_models,
        coefficients=coefficients,
        poly_degree=poly_degree,  # 文件读取的度数
        window_size=window_size,  # 文件读取的窗口
        device=device
    )

    # 4. 误差计算与时间对比
    error_metrics = calculate_error(y_raw, predictions)
    logger.info("\n====== 预测时间对比 ======")
    for method, avg_time in time_metrics.items():
        logger.info(f"{method}：平均单次预测时间={avg_time:.0f}纳秒（{avg_time/1000000:.4f}ms）")

    # 5. 可视化（用从文件读取的窗口大小）
    plot_results(y_raw, predictions, error_metrics, mode="inference", window_size=window_size)
    logger.info("\n====== 推理模式完成！ ======")


# ===================== 主程序入口（模式分支） =====================
if __name__ == "__main__":
    # 1. 运行GUI选择模式与路径
    mode, csv_file_path, model_folder_path, inference_data_path = run_gui_selection()
    
    # 检查用户是否关闭了窗口而不是做出选择
    if mode is None:
        print("用户取消操作，程序退出。")
        exit(0)

    # 2. 配置日志
    logger = setup_logging(mode, model_folder_path)
    
    # 3. 根据模式执行对应逻辑
    if mode == "train":
        if not csv_file_path:
            logger.error("训练模式：未选择CSV数据文件")
            raise FileNotFoundError("训练模式：未选择CSV数据文件")
        run_train_mode(csv_file_path)
    elif mode == "inference":
        if not (model_folder_path and inference_data_path):
            logger.error("推理模式：未选择模型文件夹或推理数据文件")
            raise FileNotFoundError("推理模式：未选择模型文件夹或推理数据文件")
        run_inference_mode(model_folder_path, inference_data_path)
    else:
        logger.error("未选择有效模式（训练/推理）")
        raise ValueError("未选择有效模式（训练/推理）")