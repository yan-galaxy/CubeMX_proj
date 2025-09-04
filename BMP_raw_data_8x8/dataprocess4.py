import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# 每次修改时不准删除任何原有注释！！！！
plt.rcParams["font.family"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams['axes.unicode_minus'] = False

class FirstOrderLagCompensator:
    def __init__(self, K, tau, T):
        """
        初始化一阶滞后补偿器
        :param K: 传感器增益
        :param tau: 时间常数 (单位：秒)
        :param T: 采样周期 (单位：秒)
        """
        self.K = K
        self.tau = tau
        self.T = T
        self.x_prev = None  # 保存上一时刻的补偿值，实现实时递推

    def compensate(self, y_current):
        """
        在线补偿：输入当前测量值，返回补偿后的真实值
        :param y_current: 当前传感器测量值
        :return: 补偿后的真实值估计
        """
        if self.x_prev is None:
            # 初始时刻：假设无滞后，补偿值等于测量值
            self.x_prev = y_current
            return y_current
        # 递推公式计算当前补偿值
        x_current = (self.T / (self.K * self.tau)) * y_current + \
                    (1 - self.T / self.tau) * self.x_prev
        self.x_prev = x_current  # 更新上一时刻补偿值，为下一次计算做准备
        return x_current

# --------------------- 数据读取与补偿流程 --------------------- #
# 1. 读取实验数据
file_path = 'raw_data_8x8_20250903_145631.csv'
script_dir = os.path.dirname(os.path.abspath(__file__))
full_file_path = os.path.join(script_dir, file_path)
data = pd.read_csv(full_file_path)
channel_0_data = data['Channel_0'].values  # 提取传感器原始数据
T = 0.015  # 采样周期：15ms = 0.015秒

# 2. 参数辨识（需根据实际数据拟合，此处为示例值）
K_identified = 1.0       # 假设增益为1（实际需标定或拟合）
tau_identified = 0.5     # 假设时间常数为0.5秒（实际需从衰减段拟合）

# 3. 初始化补偿器
compensator = FirstOrderLagCompensator(
    K=K_identified,
    tau=tau_identified,
    T=T
)

# 4. 模拟实时在线补偿（逐点处理）
compensated_data = []
for y in channel_0_data:
    x_comp = compensator.compensate(y)
    compensated_data.append(x_comp)
compensated_data = np.array(compensated_data)

# 5. 可视化原始数据与补偿后数据
plt.figure(figsize=(14, 7))  # 每次都固定这个形状，不能修改！！！

# 子图1：原始数据
plt.subplot(2, 1, 1)
plt.plot(channel_0_data, label='Raw Sensor Data')
plt.title('Raw Channel 0 Data (With Hysteresis)')
plt.xlabel('Sample Index')
plt.ylabel('Sensor Value')
plt.grid(True)
plt.legend()

# 子图2：补偿后数据
plt.subplot(2, 1, 2)
plt.plot(compensated_data, label='Compensated Data', color='red')
plt.title('Compensated Channel 0 Data (First-Order Lag Compensation)')
plt.xlabel('Sample Index')
plt.ylabel('Compensated Value (Estimated True Force)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

# 6. 输出统计信息（补偿前后对比）
print("=== 原始数据统计 ===")
print(f"数据点数量: {len(channel_0_data)}")
print(f"最大值: {channel_0_data.max():.6f}")
print(f"最小值: {channel_0_data.min():.6f}")
print(f"平均值: {channel_0_data.mean():.6f}")
print(f"标准差: {channel_0_data.std():.6f}")

print("\n=== 补偿后数据统计 ===")
print(f"数据点数量: {len(compensated_data)}")
print(f"最大值: {compensated_data.max():.6f}")
print(f"最小值: {compensated_data.min():.6f}")
print(f"平均值: {compensated_data.mean():.6f}")
print(f"标准差: {compensated_data.std():.6f}")