import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog  # 新增QFileDialog导入
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import csv
from scipy.ndimage import zoom  # 使用scipy的缩放函数 插值需要

# 数据处理核心逻辑复用
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
        
        # 均值计算
        avg_matrix = np.mean(INCRMA_normalized, axis=0)
        
        # 差值计算
        if self.matrix_flag == 0:
            self.matrix_init = avg_matrix.copy()
            self.matrix_flag = 1
            return np.zeros((10, 10))  # 初始帧返回零矩阵
        else:
            result = self.matrix_init - avg_matrix
            normal_result = result / self.matrix_init
            normal_result = np.where(normal_result < 0, 0, normal_result)
            normal_result = np.where(normal_result > 1, 1, normal_result)
            return normal_result

# 主窗口类
class PlaybackVisualizer(QMainWindow):
    def __init__(self, csv_path, interplotation=False, rotation_angle=0, flip_horizontal=False, flip_vertical=False):
        super().__init__()
        # 显示配置
        self.interplotation = interplotation
        self.rotation_angle = rotation_angle
        self.flip_horizontal = flip_horizontal
        self.flip_vertical = flip_vertical
        
        # 初始化显示组件
        self.central_widget = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.central_widget)
        self.image_item = pg.ImageItem()
        self.plot = self.central_widget.addPlot(title="数据回放")
        self.plot.addItem(self.image_item)
        self.color_map = pg.colormap.get('viridis')
        self.image_item.setColorMap(self.color_map)
        
        # 新增：帧号显示标签
        self.frame_label = pg.LabelItem(justify='left')  # 左对齐
        self.fps_label = pg.LabelItem(justify='right')   # 右对齐
        
        # 布局调整：图像在上，标签在下
        self.central_widget.addItem(self.plot, 0, 0)
        self.central_widget.addItem(self.frame_label, 1, 0)
        self.central_widget.addItem(self.fps_label, 1, 0)
        
        # 数据加载
        self.processor = DataProcessor()
        self.data_frames = self.load_csv_data(csv_path)
        self.current_frame = 0
        
        # 性能监控
        self.frame_count = 0
        self.start_time = QtCore.QTime.currentTime()
        
        # 定时器配置（10ms对应100FPS）
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(10)  # 10ms对应100FPS
    
    def load_csv_data(self, csv_path):
        """加载CSV文件数据"""
        data_frames = []
        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)
                next(reader)  # 跳过表头
                for row in reader:
                    frame_data = np.array(row[1:], dtype=np.uint16)
                    data_frames.append(frame_data)
        except Exception as e:
            print(f"加载CSV文件失败: {e}")
        return data_frames
    
    def update_plot(self):
        """定时更新图像"""
        if self.current_frame >= len(self.data_frames):
            # 数据播放结束时停止定时器并关闭程序
            self.timer.stop()  # 停止定时器防止后续错误
            print("数据播放已完成，程序即将退出")
            QtCore.QTimer.singleShot(1000, QApplication.instance().quit)  # 延迟1秒退出
            return  # 数据结束
            
        raw_frame = self.data_frames[self.current_frame]
        processed = self.processor.process_frame(raw_frame)
        
        # 旋转处理
        if self.rotation_angle == 90:
            processed = np.rot90(processed, 1)
        elif self.rotation_angle == 180:
            processed = np.rot90(processed, 2)
        elif self.rotation_angle == 270:
            processed = np.rot90(processed, 3)
            
        # 翻转处理
        if self.flip_horizontal:
            processed = np.fliplr(processed)
        if self.flip_vertical:
            processed = np.flipud(processed)
            
        # 插值处理
        if self.interplotation:
            interpolated = zoom(processed, (5, 5), order=3)
            self.image_item.setImage(interpolated, levels=(0.0, 1.0))
        else:
            self.image_item.setImage(processed, levels=(0.0, 1.0))
            
        # 更新帧号显示
        total_frames = len(self.data_frames)
        self.frame_label.setText(f"当前帧: {self.current_frame + 1}/{total_frames}")
        
        # 更新FPS计数
        self.frame_count += 1
        current_time = QtCore.QTime.currentTime()
        if self.start_time.msecsTo(current_time) >= 1000:  # 每1秒计算一次
            fps = self.frame_count
            self.fps_label.setText(f"FPS: {fps}")
            self.frame_count = 0
            self.start_time = current_time
            
        self.current_frame += 1

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 弹出文件选择对话框（保留原有参数配置）
    csv_path, _ = QFileDialog.getOpenFileName(
        None,
        "选择CSV文件",              # 对话框标题
        "INCRMA_raw_data",          # 初始目录
        "CSV文件 (*.csv)"           # 文件过滤器
    )
    
    # 如果用户取消选择，退出程序
    if not csv_path:
        print("未选择文件，程序退出。")
        sys.exit(1)
    
    # 配置回放参数（保留原有参数）  True False 0 90 180 270
    INTERPOLATION = False   # 是否启用插值
    ROTATION = 270         # 旋转角度
    FLIP_H = False         # 水平翻转
    FLIP_V = False         # 垂直翻转
    
    # 创建并显示窗口
    main_win = PlaybackVisualizer(
        csv_path, 
        interplotation=INTERPOLATION,
        rotation_angle=ROTATION,
        flip_horizontal=FLIP_H,
        flip_vertical=FLIP_V
    )
    main_win.resize(800, 800)
    main_win.show()
    
    sys.exit(app.exec_())