
import matplotlib.pyplot as plt
import numpy as np

# 解决中文显示问题
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False






print("生成固定间隔步长的序列:")
# 生成0-4095每隔5的序列（0,5,10,...,4095）
step = 41
start = 0
end = 4095

# 计算元素数量
count = (end - start) // step + 1  # (4095-0)/5 +1 = 820

# 生成C数组初始化代码
print(f"uint32_t arr[{count}] = {{")
print(",".join(map(str, range(start, end+1, step))))
print("};")


print("生成固定序列长度的序列:")
def generate_dac_sequence(length=820):
    """
    生成指定长度的DAC序列（0-4095均匀分布）
    参数：
        length - 需要生成的数组长度（>=1）
    返回：
        list - 生成的DAC序列
    """
    if length <= 0:
        return []
    
    if length == 1:
        return [0]  # 单元素特殊处理
        
    # 计算实际步长（浮点数）
    step = 4095.0 / (length - 1)
    
    # 生成序列（使用四舍五入并确保最后一个元素为4095）
    sequence = [int(round(i * step)) for i in range(length)]
    sequence[-1] = 4095  # 强制修正最后一个元素
    
    return sequence
# 示例用法：生成1000点序列
dac_sequence = generate_dac_sequence(400)

# 输出C数组代码
print(f"uint32_t arr[{len(dac_sequence)}] = {{")
print(",".join(map(str, dac_sequence)))
print("};")


print("生成向下凹的单调递增序列（二次函数）:")
def generate_concave_sequence(length=400):
    """
    生成向下凹的单调递增序列（0-4095整数）
    参数：
        length - 序列长度（>=1）
    返回：
        list - 符合要求的序列
    """
    if length <= 0:
        return []
    if length == 1:
        return [0]
    
    # 二次函数参数计算（开口向上）
    max_x = length - 1
    scale = 4095 / (max_x ** 2)
    
    # 生成基础序列
    sequence = [int(round(scale * x**2)) for x in range(length)]
    
    # 强制修正最后一个元素并确保单调性
    sequence[-1] = 4095
    for i in range(1, length):
        if sequence[i] <= sequence[i-1]:
            sequence[i] = sequence[i-1] + 1
            
    return sequence

# 示例用法：生成400点序列
dac_sequence = generate_concave_sequence(400)
# 输出C数组代码
print(f"uint32_t arr[{len(dac_sequence)}] = {{")
print(",".join(map(str, dac_sequence)))
print("};")


print("生成向上凸的单调递增序列:") # 最后几个数 4093,4094,4095,4096,4097,4098,4099,4100,4101,4102,4103,4104,4095
def generate_convex_sequence(length=400):
    """
    生成严格递增的上凸函数序列（0-4095整数，无超量程）
    参数：
        length - 序列长度（>=1）
    返回：
        list - 符合要求的序列（所有值∈[0,4095]）
    """
    if length <= 0:
        return []
    if length == 1:
        return [0]
    
    max_x = length - 1  # 最大索引值
    sequence = []
    
    for x in range(length):
        # 上凸函数核心公式：y = 4095 * [1 - (1 - x/max_x)²]
        # 确保y从0递增到4095，且所有中间值≤4095
        normalized_x = x / max_x
        y = 4095 * (1 - (1 - normalized_x) ** 2)
        # 强制限制上限为4095，避免浮点误差导致超量程
        y_clamped = min(round(y), 4095)
        sequence.append(y_clamped)
    
    # 确保严格单调递增（防止浮点误差导致的相邻值相等）
    for i in range(1, length):
        if sequence[i] <= sequence[i-1]:
            sequence[i] = sequence[i-1] + 1
            # 再次检查是否超量程（极端情况修正后可能超）
            if sequence[i] > 4095:
                sequence[i] = 4095
    
    # 最终强制修正最后一个元素为4095（确保终点正确）
    sequence[-1] = 4095
    return sequence


# 示例用法：生成400点序列
dac_sequence = generate_convex_sequence(400)

# 输出C数组代码
print(f"uint32_t arr[{len(dac_sequence)}] = {{")
print(",".join(map(str, dac_sequence)))
print("};")

# 生成可调凹度的序列  以幂函数为基础
def generate_concave_with_k(length=400, k=2.0):
    """
    生成可调凹度的向下凹单调递增序列（0-4095整数）
    参数：
        length - 序列长度（>=1）
        k - 凹度控制参数（>1），k越大凹度越强
    返回：
        list - 符合要求的序列
    """
    if length <= 0:
        return []
    if length == 1:
        return [0]
    
    # 幂函数参数计算
    max_x = length - 1
    sequence = []
    
    # 使用幂函数：y = 4095 * (x/max_x)^k
    # 当k > 1时为向下凹曲线
    # 当k == 1时为直线
    for x in range(length):
        t = x / max_x
        y = 4095 * (t ** k)
        sequence.append(y)
    
    # 强制修正最后一个元素并确保单调性
    sequence[-1] = 4095
    for i in range(1, length):
        if sequence[i] <= sequence[i-1]:
            sequence[i] = sequence[i-1] + 1
    
    # 转换为整数序列            
    return [int(round(val)) for val in sequence]

def generate_concave_with_k_inverse_prop(length=400, k=2.0):
    """
    生成可调凹度的向下凹单调递增序列（0-4095整数）
    参数：
        length - 序列长度（>=1）
        k - 凹度控制参数（>1），k越大凹度越强
    返回：
        list - 符合要求的序列
    """
    if length <= 0:
        return []
    if length == 1:
        return [0]
    
    max_x = length - 1
    sequence = [int(round(4095 * (x/max_x)**k)) for x in range(length)]
    
    # 强制修正最后一个元素并确保单调性
    sequence[-1] = 4095
    for i in range(1, length):
        if sequence[i] <= sequence[i-1]:
            sequence[i] = sequence[i-1] + 1
            
    return sequence
# 获取三个序列数据
fixed_seq = generate_dac_sequence(400)        # 固定序列
concave_seq = generate_concave_sequence(400)        # 向下凹序列
convex_seq = generate_convex_sequence(400)          # 向上凸序列

# 示例用法：生成不同凹度的序列
concave_seq_weak = generate_concave_with_k(400, k=2.2)    # 弱凹度
concave_seq_strong = generate_concave_with_k(400, k=4)   # 强凹度

convex_inverse_seq = generate_concave_with_k_inverse_prop(400,k=1.5)

# 创建可视化图表
plt.figure(figsize=(12, 6))
plt.plot(fixed_seq, label='固定序列', linestyle='-', linewidth=2)
plt.plot(concave_seq, label='向下凹的单调递增序列', linestyle='--', linewidth=2)
plt.plot(convex_seq, label='向上凸的单调递增序列', linestyle='-.', linewidth=2)

plt.plot(concave_seq_weak, label='向弱凹度的单调递增序列', linestyle='--', linewidth=2)
plt.plot(concave_seq_strong, label='向强凹度的单调递增序列', linestyle='--', linewidth=2)

plt.plot(convex_inverse_seq, label='反比例下凹', linestyle='--', linewidth=2)
# 添加图表元素
plt.title('三种DAC序列对比分析', fontsize=14)
plt.xlabel('序列索引', fontsize=12)
plt.ylabel('数值', fontsize=12)
plt.grid(True, alpha=0.3)
plt.legend(loc='best')
plt.tight_layout()

# 显示图表
plt.show()