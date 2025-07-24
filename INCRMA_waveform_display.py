import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

class DataProcessor:
    """封装数据处理逻辑"""
    def __init__(self):
        self.matrix_init = None
        self.matrix_flag = 0
        self.Rref_ohm = [2.082,2.093,2.095,2.088,2.082,1.985,1.987,1.987,1.988,1.990]
    
    def process_frame(self, parsed):
        """处理单帧数据的核心逻辑"""
        if len(parsed) != 2100:
            return np.zeros((10, 10))
            
        frames = np.array(parsed).reshape(10, 21, 10)
        
        # 基准行提取
        Rref_first_rows = frames[:, 0, :]
        expanded_Rref = np.expand_dims(Rref_first_rows, axis=1)
        Rref_first_rows = np.repeat(expanded_Rref, repeats=10, axis=1)
        
        # 数据分割
        RMA_raw_part = frames[:, 1:11, :]
        INCRMA_part = frames[:, 11:, :]
        
        # 计算分子分母
        RMA_numerator = (4095 - Rref_first_rows)
        RMA_denominator = (4095 - RMA_raw_part)
        INCRMA_numerator = (RMA_raw_part - INCRMA_part)
        INCRMA_denominator = (Rref_first_rows - INCRMA_part)
        
        # 防除零处理
        RMA_denominator = np.where(RMA_denominator == 0, 1e-6, RMA_denominator)
        INCRMA_denominator = np.where(INCRMA_denominator == 0, 1e-6, INCRMA_denominator)
        
        # 电阻计算
        RMA_normalized = (RMA_numerator / RMA_denominator) * self.Rref_ohm
        INCRMA_normalized = (INCRMA_numerator / INCRMA_denominator) * self.Rref_ohm
        
        # 差值计算
        if self.matrix_flag == 0:
            # 均值计算
            avg_matrix = np.mean(INCRMA_normalized, axis=0)
            self.matrix_init = avg_matrix.copy()
            self.matrix_flag = 1
            return np.zeros((10, 10, 10))  # 初始帧返回零矩阵
        else:
            result = self.matrix_init - INCRMA_normalized
            normal_result = result / self.matrix_init
            normal_result = np.where(normal_result < 0, 0, normal_result)
            normal_result = np.where(normal_result > 1, 1, normal_result)
            return normal_result

class WaveformVisualizer(QMainWindow):
    def __init__(self, npz_path):
        super().__init__()
        self.setWindowTitle("NPZ数据波形显示")
        
        # 创建主窗口组件
        self.central_widget = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.central_widget)
        
        # 初始化波形显示区域
        self.plot = self.central_widget.addPlot(title="传感器数据波形")
        self.plot.setLabels(left='数值', bottom='时间序号')
        self.plot.showGrid(x=True, y=True)
        self.plot.setYRange(0, 1.2)  # 固定Y轴范围为0~1.0
        
        # 创建曲线对象
        self.curve = self.plot.plot(pen='y')
        
        # 初始化数据处理器
        self.processor = DataProcessor()
        
        # 数据加载
        self.data_frames = self.load_npz_data(npz_path)
        # print(self.data_frames[0].shape)
        # print(len(self.data_frames))
        self.current_frame = 0
        # 新增存储总和数据的列表
        self.summed_data = []
        self.show_data = [0] * 3000
        
        # 性能监控
        self.frame_count = 0
        self.start_time = QtCore.QTime.currentTime()
        
        # 定时器配置（每ms更新一次）
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(4)  # 20FPS

    def load_npz_data(self, npz_path):
        """加载NPZ文件数据并处理为normal_result格式"""
        data_frames = []
        try:
            with np.load(npz_path) as data:
                if 'data' in data:
                    # 对每个数据帧进行处理
                    for row in data['data']:
                        # print(row.shape)
                        processed = self.processor.process_frame(row)
                        # print(processed.shape)
                        # 拆分为 10 个子帧，每个形状为 (10, 10)
                        for i in range(10):
                            subframe = processed[i]  # 形状为 (10, 10)
                            data_frames.append(subframe.flatten())  # 展平为 (100,)
                    print(f"成功加载 {len(data_frames)} 帧处理后数据")
        except Exception as e:
            print(f"加载NPZ文件失败: {e}")
        return data_frames

    def update_plot(self):
        """定时更新波形"""
        if self.current_frame >= len(self.data_frames):# self.data_frames为所有数据帧全部读出
            # 数据播放结束时停止定时器
            self.timer.stop()
            print("数据播放已完成")
            return
            
        new_data = []

        for _ in range(4): # 每次处理2帧数据
            if self.current_frame >= len(self.data_frames):
                break
            current_data = self.data_frames[self.current_frame]
            # center_sum = (current_data[44] + current_data[45] + current_data[54] + current_data[55]) / 4.0  # 提取中间四个点：44, 45, 54, 55
            center_sum = current_data[44]  # 使用第 44 个点的数据
            new_data.append(center_sum)
            self.current_frame += 1

        # 删除旧数据，添加新数据
        self.show_data = self.show_data[len(new_data):] + new_data
        

        # 更新波形（使用累积数据）
        x_data = np.arange(len(self.show_data))
        y_data = np.array(self.show_data)

        self.curve.setData(x_data, y_data)
        
        # 更新标题显示当前帧号
        self.plot.setTitle(f"传感器数据波形 - 第 {self.current_frame + 1} 帧")
        
        # FPS计算
        self.frame_count += 1
        current_time = QtCore.QTime.currentTime()
        if self.start_time.msecsTo(current_time) >= 1000:  # 每1秒计算一次
            fps = self.frame_count
            self.setWindowTitle(f"NPZ数据波形显示 - FPS: {fps}")
            self.frame_count = 0
            self.start_time = current_time
            
        self.current_frame += 1

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 弹出文件选择对话框
    npz_path, _ = QFileDialog.getOpenFileName(
        None,
        "选择NPZ文件",
        "INCRMA_raw_data",
        "NPZ文件 (*.npz)"
    )
    
    # 如果用户取消选择，退出程序
    if not npz_path:
        print("未选择文件，程序退出。")
        sys.exit(1)
    
    # 创建并显示窗口
    main_win = WaveformVisualizer(npz_path)
    main_win.resize(1000, 600)
    main_win.show()
    
    sys.exit(app.exec_())