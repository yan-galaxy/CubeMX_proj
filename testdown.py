import numpy as np



clip8x8_matrix = np.arange(64).reshape(8, 8)# 创建一个8x8的矩阵，包含从0到63的整数
clip8x8_matrix = np.random.randint(0, 64, size=(8, 8))# 创建一个8x8的矩阵，包含从0到63的随机整数

print('clip8x8_matrix:\n',clip8x8_matrix)
print('clip8x8_matrix.mean(axis=(1)):\n',clip8x8_matrix.mean(axis=(0)))

# original_blocks_1 = clip8x8_matrix.reshape(4, 2, 4, 2)
original_blocks_2 = clip8x8_matrix.reshape(4, 2, 4, 2).transpose(0, 2, 1, 3)
# print('original_blocks_1:\n',original_blocks_1)
print('original_blocks_2:\n',original_blocks_2)
print('original_blocks_2.shape:\n',original_blocks_2.shape)

# 步骤1：计算横向差（列方向）
horizontal_diff_x = (
    original_blocks_2[:,:,:,1].mean(axis=2)  # 右边列的平均值
    - original_blocks_2[:,:,:,0].mean(axis=2)  # 左边列的平均值
)

# 步骤2：计算纵向差（行方向）
vertical_diff_y = (
    original_blocks_2[:,:,0,:].mean(axis=2)  # 上面行的平均值
    - original_blocks_2[:,:,1,:].mean(axis=2)  # 下面行的平均值
)
print('horizontal_diff_x:\n',horizontal_diff_x)
print('vertical_diff_y:\n',vertical_diff_y)

# original_blocks_1=original_blocks_1.mean(axis=(1,3))
# original_blocks_2=original_blocks_2.mean(axis=(2,3))
# print('original_blocks_1:\n',original_blocks_1)
# print('original_blocks_2:\n',original_blocks_2)