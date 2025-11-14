from openpyxl import Workbook

# 1. 创建工作簿和工作表
wb = Workbook()
ws = wb.active
ws.title = "数据表格"

# 2. 设置表头
headers = ["x", "y", "z", "r", "v", "w", "method"]
ws.append(headers)

# 定义参数
X_CENTER = 6800    # 高值x
Z_STIMULATE = 5290    # 激励
Z_HIGH = 2000    # 高处
Z_LOW = 4900     # 低值z
Y_FIXED = 4640   # 固定y值
W_FIXED = 1050   # 固定w值

Z_TARGET = 5280     # 按压z
POSITIONS_PER_SIDE = 16 # 单侧位置数



# 1.由步进计算边界x值 OR 2.由单边边界与中心的差值计算步进长度
# # 1.由步进计算边界x值
# STEP = 25 # 步进长度
# # 计算并打印两个边界的x值
# left_boundary = X_CENTER - POSITIONS_PER_SIDE * STEP
# right_boundary = X_CENTER + POSITIONS_PER_SIDE * STEP
# print(f"x范围: {left_boundary} ~ {right_boundary}")

# 2.由单边边界与中心的差值计算步进长度
HALF_RANGE = 400  # 单边边界与中心的差值
STEP = HALF_RANGE // POSITIONS_PER_SIDE  # 根据差值和位置数自动计算步进长度
# 计算并打印两个边界的x值
left_boundary = X_CENTER - HALF_RANGE
right_boundary = X_CENTER + HALF_RANGE
print(f"x范围: {left_boundary} ~ {right_boundary}")
print(f"步进长度: {STEP}")




CYCLE = POSITIONS_PER_SIDE * 2 + 1 # 循环次数





data_first_rows = [
    [X_CENTER, Y_FIXED, 4000, 0, 0, W_FIXED, 0,'中心麦克风 高处 z轴4000'],
    [X_CENTER, Y_FIXED, 5000, 0, 0, W_FIXED, 0,'正好接触表面'],
    [X_CENTER, Y_FIXED, Z_HIGH, 0, 0, W_FIXED, 0,'到中点高处'],
    [X_CENTER, Y_FIXED, Z_STIMULATE, 0, 0, W_FIXED, 0,'激励'],
    [X_CENTER, Y_FIXED, Z_LOW, 0, 0, W_FIXED, 0,'距离表面100']
]
for row in data_first_rows:
    ws.append(row)

# 循环生成多组数据
for i in range(CYCLE):  # 生成5组数据，可以根据需要修改循环次数
    # 自动计算边界x值，基于X_CENTER和步进值
    current_x_target = X_CENTER - (CYCLE // 2) * STEP + i * STEP   
    current_z_target = Z_TARGET
    
    # 生成数据行
    data_rows = [
        [current_x_target, Y_FIXED, Z_LOW, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, current_z_target, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, Z_LOW, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, current_z_target, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, Z_LOW, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, current_z_target, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, Z_LOW, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, current_z_target, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, Z_LOW, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, current_z_target, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, Z_LOW, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, current_z_target, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, Z_LOW, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, current_z_target, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, Z_LOW, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, current_z_target, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, Z_LOW, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, current_z_target, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, Z_LOW, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, current_z_target, 0, 0, W_FIXED, 0],
        [current_x_target, Y_FIXED, Z_LOW, 0, 0, W_FIXED, 0,'回位'],   # 回位
        [current_x_target, Y_FIXED, Z_HIGH, 0, 0, W_FIXED, 0,'到高处'],  # 到高处
        [X_CENTER, Y_FIXED, Z_HIGH, 0, 0, W_FIXED, 0,'到中点高处'],  # 到中点高处
        [X_CENTER, Y_FIXED, Z_STIMULATE, 0, 0, W_FIXED, 0,'激励'], # 激励
        [X_CENTER, Y_FIXED, Z_LOW, 0, 0, W_FIXED, 0,'距离表面100'],   # 距离表面100
        [current_x_target, Y_FIXED, Z_LOW, 0, 0, W_FIXED, 0,'回位'],   # 回位
    ]
    
    # 将数据写入工作表
    for row in data_rows:
        ws.append(row)

# 5. 保存文件
wb.save("7MIC_raw_data/标定台运动文件/yhm标定打点记录_麦克风_感受野_单边400_步进25_下降280.xlsx")
print("Excel文件生成成功！")