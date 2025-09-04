import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# 每次修改时不准删除任何原有注释！！！！
plt.rcParams["font.family"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams['axes.unicode_minus'] = False

# 读取CSV文件
file_path = 'raw_data_8x8_20250903_145631.csv'
# 确保在当前脚本目录下查找文件
script_dir = os.path.dirname(os.path.abspath(__file__))
full_file_path = os.path.join(script_dir, file_path)

data = pd.read_csv(full_file_path)

# 提取第一列数据
channel_0_data = data['Channel_0']

# 创建可视化图表
plt.figure(figsize=(14, 7))  # 调整图大小，避免子图拥挤,每次都固定这个形状，不能修改！！！
plt.plot(channel_0_data, linestyle='-')
plt.title('Channel 0 Data Visualization')
plt.xlabel('Sample Index')
plt.ylabel('Value')
plt.grid(True)
plt.tight_layout()
plt.show()

# 打印一些基本统计数据
print(f"数据点数量: {len(channel_0_data)}")
print(f"最大值: {channel_0_data.max()}")
print(f"最小值: {channel_0_data.min()}")
print(f"平均值: {channel_0_data.mean():.6f}")
print(f"标准差: {channel_0_data.std():.6f}")