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