
# import matplotlib.pyplot as plt
# import numpy as np

# # 解决中文显示问题
# plt.rcParams['font.sans-serif'] = ['SimHei']
# plt.rcParams['axes.unicode_minus'] = False

print("Hello, World!");

target_cutfreq = 20000.0 # 目标截至频率 单位Hz
freq = 170000000.0/2.0
OUT = int(freq/(target_cutfreq*100.0))-1
print("计算最接近的Prescaler:",OUT)
real_cutfreq = freq/(OUT+1)/100.0
print("根据Prescaler反推真实截止频率",real_cutfreq,"Hz")
error = (real_cutfreq-target_cutfreq)/target_cutfreq
print("误差：",error*100,"%")



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

def generate_concave_sequence_2(length=400, factor=1.0):
    """
    生成可调节凹度的单调递增序列（0-4095整数）
    参数：
        length - 序列长度（>=2）
        factor - 凹度系数（>0）：
                 factor < 1：增大凹度（曲线前段更陡）
                 factor = 1：标准二次曲线
                 factor > 1：减小凹度（曲线更平缓）
    返回：
        list - 符合要求的序列
    """
    if length <= 1:
        return [0] * length if length else []
    
    max_x = length - 1
    sequence = []
    
    # 使用归一化幂函数保证终点精度
    for x in range(length):
        # 计算归一化位置
        t = x / max_x
        # 通过幂函数控制凹度：y = t^(2 - factor)
        # 当factor=1时为标准二次曲线
        # 当factor<1时凹度更大
        # 当factor>1时凹度更小
        y = t ** (2 - factor)
        # 线性插值得到精确的0-4095范围
        value = round(y * 4095)
        sequence.append(value)
    
    # 确保严格单调递增（处理可能的重复值）
    for i in range(1, length):
        if sequence[i] <= sequence[i-1]:
            sequence[i] = sequence[i-1] + 1
            
    # 强制确保终点精度
    sequence[-1] = 4095
    
    return sequence

# 示例用法：生成400点序列
dac_sequence = generate_concave_sequence(400)
# dac_sequence = generate_concave_sequence_2(length=400, factor=1.0)
# 输出C数组代码
print(f"uint32_t arr[{len(dac_sequence)}] = {{")
print(",".join(map(str, dac_sequence)))
print("};")


print("生成向上凸的单调递增序列:")
def generate_convex_sequence(length=400):
    """
    生成严格递增的上凸函数序列（0-4095整数）
    参数：
        length - 序列长度（>=1）
    返回：
        list - 符合要求的序列
    """
    if length <= 0:
        return []
    if length == 1:
        return [0]
    
    n = length - 1
    # 修正参数计算公式
    a = -4095 / (n**2)        # 二次项系数
    b = 8190 / n             # 一次项系数
    
    # 生成基础序列并四舍五入
    sequence = [int(round(a*x**2 + b*x)) for x in range(length)]
    
    # 严格单调性保障（从第二个元素开始检查）
    for i in range(1, length):
        if sequence[i] <= sequence[i-1]:
            sequence[i] = sequence[i-1] + 1
            
    # 强制修正最后一个元素
    sequence[-1] = 4095
    
    return sequence


# 示例用法：生成400点序列
dac_sequence = generate_convex_sequence(400)

# 输出C数组代码
print(f"uint32_t arr[{len(dac_sequence)}] = {{")
print(",".join(map(str, dac_sequence)))
print("};")