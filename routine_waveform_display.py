# routine_waveform_display.py
import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import os

class RoutineWaveformVisualizer(QMainWindow):
    def __init__(self, npz_path):
        super().__init__()
        self.setWindowTitle("Routine NPZ 波形显示")
        self.npz_path = npz_path

        # 创建主窗口组件
        self.central_widget = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.central_widget)

        # 初始化波形显示区域
        self.plot = self.central_widget.addPlot(title="中间四个点的波形")
        self.plot.setLabels(left='数值', bottom='帧编号')
        self.plot.showGrid(x=True, y=True)
        self.plot.setYRange(0, 4096)  # 固定Y轴范围为0~4096

        # 创建曲线对象
        self.curve = self.plot.plot(pen='y')

        # 存储波形数据
        self.show_data = [0] * 3000

        # 性能监控
        self.frame_count = 0
        self.start_time = QtCore.QTime.currentTime()

        # 定时器配置（每 1ms 更新一次）
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(4)  # 1ms 间隔

        # 加载数据
        self.data_frames = self.load_npz_data(npz_path)
        self.current_frame = 0

        print(len(self.data_frames))

    def load_npz_data(self, npz_path):
        """加载NPZ文件中的数据帧"""
        data_frames = []
        try:
            with np.load(npz_path) as data:
                if 'data' in data:
                    for row in data['data']:
                        # 拆分为 10 个子帧，每个形状为 (10, 10)
                        for i in range(10):
                            start = i * 100
                            end = start + 100
                            subframe = row[start:end]  # shape: (100,)
                            data_frames.append(subframe)
                    print(f"成功加载 {len(data_frames)} 帧数据")
                    return data_frames
        except Exception as e:
            print(f"加载NPZ文件失败: {e}")
            return []

    def update_plot(self):
        """定时更新波形，每次处理 10 帧数据"""
        if self.current_frame >= len(self.data_frames):
            self.timer.stop()
            print("数据播放已完成")
            self.convert_npz_to_csv()
            return

        new_data = []

        for _ in range(4): # 每次处理2帧数据
            if self.current_frame >= len(self.data_frames):
                break
            current_data = self.data_frames[self.current_frame]
            # center_sum = (current_data[44] + current_data[45] + current_data[54] + current_data[55]) / 4.0  # 提取中间四个点：44, 45, 54, 55
            center_sum = current_data[48]  # 使用第 44 个点的数据
            new_data.append(center_sum)
            self.current_frame += 1

        # 删除旧数据，添加新数据
        self.show_data = self.show_data[len(new_data):] + new_data

        # 更新波形
        x_data = np.arange(len(self.show_data))
        y_data = np.array(self.show_data)
        self.curve.setData(x_data, y_data)


        # 更新标题
        self.plot.setTitle(f"中间四个点波形 - 第 {self.current_frame + 1} 帧")

        # FPS计算
        self.frame_count += 1
        current_time = QtCore.QTime.currentTime()
        if self.start_time.msecsTo(current_time) >= 1000:  # 每1秒计算一次
            fps = self.frame_count
            self.setWindowTitle(f"Routine 波形显示 - FPS: {fps}")
            self.frame_count = 0
            self.start_time = current_time

        self.current_frame += 1

    def convert_npz_to_csv(self):
        """将NPZ数据转换为CSV文件，按时刻组织，列是100个点位，行是时刻"""
        base_name = os.path.splitext(self.npz_path)[0]
        csv_path = base_name + ".csv"

        if os.path.exists(csv_path):
            print(f"文件 {csv_path} 已存在，跳过转换")
            return

        try:
            with open(csv_path, 'w', newline='') as f:
                # 当前 data_frames 是 (M, 100)，每个元素是一个小帧
                data_length = len(self.data_frames)  # 总时刻数 = 小帧数

                # 构建时间序列数据：(M, 100)
                time_series_data = []

                # 按顺序添加所有小帧数据
                for time_idx in range(data_length):
                    subframe = self.data_frames[time_idx]  # shape: (100,)
                    time_series_data.append(subframe)

                # 转换为 NumPy 数组
                time_series_data = np.array(time_series_data)  # shape: (data_length, 100)

                # 写入 CSV：每行是一个时刻的 100 个点位
                for time_idx in range(data_length):
                    line = ','.join(map(str, time_series_data[time_idx]))
                    f.write(line + '\n')

            print(f"成功保存为CSV文件: {csv_path}")
        except Exception as e:
            print(f"保存CSV文件失败: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)

    # 弹出文件选择对话框
    npz_path, _ = QFileDialog.getOpenFileName(
        None,
        "选择Routine NPZ文件",
        "Routine_acq_raw_data",  # 默认路径
        "NPZ文件 (*.npz)"
    )

    # 如果用户取消选择，退出程序
    if not npz_path:
        print("未选择文件，程序退出。")
        sys.exit(1)

    # 创建并显示窗口
    main_win = RoutineWaveformVisualizer(npz_path)
    main_win.resize(1000, 600)
    main_win.show()

    sys.exit(app.exec_())