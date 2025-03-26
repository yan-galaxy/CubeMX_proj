# #示例 1 显示静止数据
# import pyqtgraph as pg
# import numpy as np

# # 创建应用和窗口
# # app = pg.mkQApp("基础示例") #这句可以不用
# win = pg.plot()

# # 生成数据并绘图
# x = np.linspace(0, 10, 1000)
# y = np.sin(x)
# win.plot(x, y, pen='g', name='正弦波')

# # 启动应用
# if __name__ == '__main__':
#     pg.exec()






# #示例 2 显示实时数据
# import pyqtgraph as pg
# import numpy as np
# from pyqtgraph.Qt import QtCore

# # 创建绘图窗口
# win = pg.GraphicsLayoutWidget(show=True, title="实时数据更新")
# plot = win.addPlot(title="传感器数据")
# curve = plot.plot(pen='y')

# # 模拟数据源
# data = np.random.normal(size=100)

# def update():
#     global data
#     # 模拟新数据到来
#     data[:-1] = data[1:]
#     data[-1] = np.random.normal()
#     curve.setData(data)

# # 设置定时器，每 fresh_time ms更新一次
# fresh_time = 10
# timer = QtCore.QTimer()
# timer.timeout.connect(update)
# timer.start(fresh_time)

# if __name__ == '__main__':
#     pg.exec()








# #示例 3 显示静态曲线和散点图
# import pyqtgraph as pg
# import numpy as np

# # 创建窗口和布局
# app = pg.mkQApp("多图表示例")
# win = pg.GraphicsLayoutWidget(show=True)
# win.resize(800, 600)

# # 添加第一个图表 - 时间序列
# p1 = win.addPlot(title="时间序列")
# x = np.arange(100)
# y = np.sin(x/10) * 3 + np.random.normal(size=100)
# p1.plot(x, y, pen='r')

# # 换行添加第二个图表 - 散点图
# win.nextRow()
# p2 = win.addPlot(title="散点图")
# scatter = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(0, 255, 0, 120))
# pos = np.random.normal(size=(100,2))
# scatter.addPoints(pos=pos)
# p2.addItem(scatter)

# if __name__ == '__main__':
#     pg.exec()









# #示例 4 交互式工具示例
# import pyqtgraph as pg
# import numpy as np

# # 创建窗口
# win = pg.plot()
# win.setWindowTitle('交互式工具示例')

# # 生成数据
# x = np.linspace(0, 10, 1000)
# y = np.sin(x) + np.random.normal(scale=0.1, size=len(x))

# # 绘制数据
# plot = win.plot(x, y, pen='b')

# # 添加交互工具
# win.addItem(pg.InfiniteLine(angle=90, movable=True, label='x={value:0.2f}'))
# win.addItem(pg.InfiniteLine(angle=0, movable=True, label='y={value:0.2f}'))

# # 启用交互功能
# win.setMouseEnabled(x=True, y=True)
# win.enableAutoRange()
# win.showGrid(x=True, y=True)

# if __name__ == '__main__':
#     pg.exec()






# #示例 5 显示多个图表实时数据
# import pyqtgraph as pg
# from pyqtgraph.Qt import QtCore, QtWidgets
# import numpy as np
# import random

# class SensorDashboard(QtWidgets.QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("传感器监控仪表盘")
#         self.resize(1000, 600)
        
#         # 创建中央部件和布局
#         central = QtWidgets.QWidget()
#         self.setCentralWidget(central)
#         layout = QtWidgets.QGridLayout(central)
        
#         # 创建多个图表
#         self.temp_plot = pg.PlotWidget(title="温度传感器")
#         self.pressure_plot = pg.PlotWidget(title="压力传感器")
#         self.humidity_plot = pg.PlotWidget(title="湿度传感器")
        
#         # 添加到布局
#         layout.addWidget(self.temp_plot, 0, 0)
#         layout.addWidget(self.pressure_plot, 0, 1)
#         layout.addWidget(self.humidity_plot, 1, 0, 1, 2)
        
#         # 设置图表属性
#         self.plots = [self.temp_plot, self.pressure_plot, self.humidity_plot]
#         colors = ['r', 'g', 'b']
#         self.curves = []
        
#         for idx, plot in enumerate(self.plots):
#             plot.showGrid(x=True, y=True)
#             plot.setLabel('left', 'Value')
#             plot.setLabel('bottom', 'Time (s)')
#             self.curves.append(plot.plot(pen=colors[idx]))
            
#         # 初始化数据
#         self.data = [np.zeros(100) for _ in range(3)]
        
#         # 设置定时器更新数据
#         self.timer = QtCore.QTimer()
#         self.timer.timeout.connect(self.update_data)
#         self.timer.start(50)
        
#     def update_data(self):
#         # 模拟传感器数据更新
#         for i in range(3):
#             self.data[i][:-1] = self.data[i][1:]
            
#         # 模拟不同传感器的数据特点
#         self.data[0][-1] = 25 + random.uniform(-1, 1)  # 温度
#         self.data[1][-1] = 101.3 + random.uniform(-0.5, 0.5)  # 压力
#         self.data[2][-1] = 50 + 5 * np.sin(time.time()) + random.uniform(-2, 2)  # 湿度
        
#         # 更新图表
#         for i, curve in enumerate(self.curves):
#             curve.setData(self.data[i])

# if __name__ == '__main__':
#     import sys
#     import time
    
#     app = pg.mkQApp()
#     dashboard = SensorDashboard()
#     dashboard.show()
#     sys.exit(app.exec())




import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow
import pyqtgraph as pg


# 基础色图（适用于大多数场景）
['viridis', 'plasma', 'inferno', 'magma', 'cividis',  # Matplotlib风格
 'hot', 'cool', 'spring', 'summer', 'autumn', 'winter',  # 季节色系
 'parula', 'jet', 'hsv', 'rainbow', 'ocean', 'terrain',  # 传统色图
 'gray', 'bone', 'pink', 'copper', 'prism']  # 特殊效果

# 高对比度色图（适合科学可视化）
['CET-L17', 'CET-L18', 'CET-L19']  # 来自ColorBrewer的色阶方案


color = 'viridis'


class MatrixVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("实时矩阵可视化")
        
        # 创建主窗口组件
        self.central_widget = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.central_widget)
        
        # 初始化图像显示区域
        self.image_item = pg.ImageItem()
        self.plot = self.central_widget.addPlot(title="10x10 实时数据矩阵")
        self.plot.addItem(self.image_item)
        
        # 设置颜色映射
        self.color_map = pg.colormap.get(color)
        self.image_item.setColorMap(self.color_map)
        
        # 初始化定时器
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(20)  # 100ms更新一次
        
        # 初始化数据
        self.data = np.random.rand(10, 10)

        self.fresh_cnt = 0
    def update_data(self):
        """动态生成新数据并更新显示"""

        new_data = (np.random.rand(10) + self.fresh_cnt) * 0.1

        # 生成新数据（示例：随机数据）
        # new_data = np.random.rand(10)
        if self.fresh_cnt < 9:
            self.fresh_cnt += 1
        else:
            self.fresh_cnt = 0
        
        
        # 更新数据源（高效更新方式）
        self.data = np.roll(self.data, 1, axis=0)  # 滚动数据
        self.data[0] = new_data  # 替换首列 防止每次循环都是一样的数据
        # print(self.data.shape)
        # 更新图像显示
        self.image_item.setImage(self.data)
        
        # 自动调整显示范围（可选）
        self.plot.autoRange(padding=0.1)

    def setup_display(self):
        """增强显示效果"""
        # 设置坐标轴标签
        self.plot.setLabels(left='Y轴', bottom='X轴')
        
        # 添加网格线
        self.plot.showGrid(x=True, y=True, alpha=0.5)
        
        # 固定显示范围（防止自动缩放干扰观察）
        self.plot.setXRange(0, 10)
        self.plot.setYRange(0, 10)
        
        # 添加颜色条
        color_bar = pg.ColorBarItem(values=(0, 1), width=20)
        self.central_widget.addItem(color_bar, 1, 0)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 创建主窗口实例
    main_win = MatrixVisualizer()
    # main_win.setup_display()  # 可选的高级配置
    main_win.resize(600, 600)
    main_win.show()
    
    sys.exit(app.exec_())