import numpy as np
import os
import sys
from tkinter import Tk
from tkinter.filedialog import askopenfilename

def convert_npz_to_csv(npz_file_path):
    """
    将7MIC.py保存的npz文件转换为CSV文件，每个通道占一列
    
    数据结构:
    - npz文件包含700个数据点
    - 分为7个通道，每个通道100个数据点
    - 通道顺序: 麦克风1到麦克风7
    - 修改为7列，每列一个麦克风通道，每帧100行数据垂直排列
    """
    
    # 加载npz文件
    data = np.load(npz_file_path)
    
    # 获取数据数组
    if 'data' in data.files:
        raw_data = data['data']
    else:
        # 如果没有'data'键，则假设第一个键是数据
        raw_data = data[data.files[0]]
    
    # 确保数据是二维数组 (帧数, 700)
    if raw_data.ndim == 1:
        # 如果是一维数组，重塑为(1, 700)
        raw_data = raw_data.reshape(1, -1)
    elif raw_data.ndim > 2:
        # 如果是更高维的数组，压平除最后一维外的所有维度
        raw_data = raw_data.reshape(-1, raw_data.shape[-1])
    
    # 将数据重塑为 (帧数*100, 7) 的形式
    # 每一列代表一个麦克风通道
    num_frames = raw_data.shape[0]
    num_points_per_channel = 100
    num_channels = 7
    
    # 创建一个新的数组来存储重新排列的数据
    reshaped_data = np.zeros((num_frames * num_points_per_channel, num_channels), dtype=np.uint16)
    
    for frame_idx in range(num_frames):
        frame_data = raw_data[frame_idx]
        for channel in range(num_channels):
            start_idx = channel * num_points_per_channel
            end_idx = (channel + 1) * num_points_per_channel
            reshaped_data[frame_idx*num_points_per_channel:(frame_idx+1)*num_points_per_channel, channel] = \
                frame_data[start_idx:end_idx]
    
    # 生成CSV文件名，保存在同一目录中
    base_name = os.path.splitext(npz_file_path)[0]
    csv_file_path = base_name + '.csv'
    
    # 确保CSV文件保存在与NPZ文件相同的目录中
    npz_dir = os.path.dirname(npz_file_path)
    if not npz_dir:
        npz_dir = "."
    csv_file_path = os.path.join(npz_dir, os.path.basename(csv_file_path))
    
    # 创建列名（7列）
    column_names = [f'MIC{i+1}' for i in range(7)]
    
    # 保存为CSV文件
    header = ','.join(column_names)
    
    # 写入CSV文件
    with open(csv_file_path, 'w') as f:
        f.write(header + '\n')
        np.savetxt(f, reshaped_data, delimiter=',', fmt='%d')  # 使用整数格式保存原始数据
    
    print(f"转换完成: {npz_file_path} -> {csv_file_path}")
    print(f"数据形状: {reshaped_data.shape} (数据点数, 通道数)")
    print(f"列数: {len(column_names)}")
    print(f"帧数: {num_frames}")
    
    return csv_file_path

def main():
    # 隐藏Tk窗口
    root = Tk()
    root.withdraw()
    
    # 设置默认目录为7MIC_raw_data（如果存在）
    initial_dir = "."
    if os.path.exists("7MIC_raw_data") and os.path.isdir("7MIC_raw_data"):
        initial_dir = "7MIC_raw_data"
    
    # 弹出文件选择对话框，只显示NPZ文件
    npz_file_path = askopenfilename(
        title="选择要转换的NPZ文件",
        initialdir=initial_dir,
        filetypes=[("NPZ文件", "*.npz"), ("所有文件", "*.*")]
    )
    
    if not npz_file_path:
        print("未选择文件")
        return
    
    if not os.path.isfile(npz_file_path) or not npz_file_path.endswith('.npz'):
        print("请选择一个有效的NPZ文件")
        return
    
    # 转换单个文件
    convert_npz_to_csv(npz_file_path)

if __name__ == "__main__":
    main()