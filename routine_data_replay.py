# routine_data_replay.py
import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from scipy.ndimage import zoom  # 插值使用
import os

class DataProcessor:
    """Routine数据处理逻辑"""
    def __init__(self):
        self.matrix_init = None
        self.matrix_flag = 0  # 基准帧标志位
        self.matrix_avg = 0    # 新增：存储平均值
        
    def process_frame(self, frame):
        """
        将100点数据转换为10x10矩阵
        Args:
            frame_100: 100维数组或列表
        Returns:
            processed: 处理后的10x10矩阵
        """
        # 将 frame 转换为 NumPy 数组并计算沿 axis=0 的平均值
        avg_frame = np.mean(np.array(frame), axis=0)

        # 转换为10x10矩阵
        matrix = avg_frame.reshape(10, 10).astype(float)
        
        # 基准帧处理逻辑
        if self.matrix_flag == 0:
            self.matrix_init = matrix.copy()
            self.matrix_flag = 1
            # 新增：计算并保存初始矩阵的平均值
            self.matrix_avg = np.mean(self.matrix_init)
            return np.zeros((10, 10))  # 初始帧返回零矩阵
            
        # 差值计算
        result = matrix - self.matrix_init
        
        # 归一化到[0,1]范围（基于Routine数据范围0-4096）
        normalized = result/(4096.0-self.matrix_avg)  # 假设最大差值±4096
        normalized = np.clip(normalized, 0, 1)
        
        return normalized

class PlaybackVisualizer(QMainWindow):
    def __init__(self, file_path, interpolation=False, 
                rotation_angle=0, flip_horizontal=False, flip_vertical=False):
        super().__init__()
        # 显示配置
        self.interpolation = interpolation
        self.rotation_angle = rotation_angle
        self.flip_horizontal = flip_horizontal
        self.flip_vertical = flip_vertical
        
        # 初始化显示组件
        self.central_widget = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.central_widget)
        self.image_item = pg.ImageItem()
        self.plot = self.central_widget.addPlot(title="Routine 数据回放")
        self.plot.addItem(self.image_item)
        self.color_map = pg.colormap.get('viridis')
        self.image_item.setColorMap(self.color_map)
        
        # 标签组件
        self.frame_label = pg.LabelItem(justify='left')  # 帧号显示
        self.fps_label = pg.LabelItem(justify='right')   # FPS显示
        
        # 布局调整
        self.central_widget.addItem(self.plot, 0, 0)
        self.central_widget.addItem(self.frame_label, 1, 0)
        self.central_widget.addItem(self.fps_label, 1, 0)
        
        # 数据加载
        self.processor = DataProcessor()
        self.data_frames = self.load_file(file_path)
        self.current_frame = 0
        
        # 性能监控
        self.frame_count = 0
        self.start_time = QtCore.QTime.currentTime()
        
        # 定时器配置（20ms对应50FPS）
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(10)  # 可根据需求调整间隔

    def load_file(self, file_path):
        """统一文件加载接口"""
        _, ext = os.path.splitext(file_path)
        if ext.lower() == '.npz':
            return self.load_npz_data(file_path)
        elif ext.lower() == '.csv':
            return self.load_csv_data(file_path)
        else:
            raise ValueError(f"不支持的文件格式: {ext}")

    def load_npz_data(self, npz_path):
        """加载Routine NPZ文件"""
        data_frames = []
        try:
            with np.load(npz_path) as data:
                if 'data' in data:
                    # 拆分每个1000点帧为10个子帧（每个100点）
                    for row in data['data']:
                        for i in range(10):  # 每帧拆分为10个子帧
                            start = i * 100
                            end = start + 100
                            data_frames.append(row[start:end].tolist())
            print(f"成功加载 {len(data_frames)} 个子帧")
        except Exception as e:
            print(f"加载NPZ文件失败: {e}")
        return data_frames

    def load_csv_data(self, csv_path):
        """加载Routine CSV文件"""
        data_frames = []
        try:
            # CSV每行对应100点数据
            data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
            if data.ndim == 2 and data.shape[1] == 100:
                data_frames = [row for row in data]
                print(f"成功加载 {len(data_frames)} 个子帧")
            else:
                print("CSV文件格式不匹配（预期100列）")
        except Exception as e:
            print(f"加载CSV文件失败: {e}")
        return data_frames

    def update_plot(self):
        """定时更新图像"""
        if self.current_frame >= len(self.data_frames):
            self.timer.stop()
            print("数据播放已完成")
            QtCore.QTimer.singleShot(1000, QApplication.instance().quit)
            return
        
        raw_data = self.data_frames[self.current_frame:self.current_frame + 10]
        processed = self.processor.process_frame(raw_data)

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
        if self.interpolation:
            interpolated = zoom(processed, (5, 5), order=3)
            self.image_item.setImage(interpolated, levels=(0.0, 1.0))
        else:
            self.image_item.setImage(processed, levels=(0.0, 1.0))
            
        # 更新显示信息
        self.frame_label.setText(f"当前帧: {self.current_frame + 1}/{len(self.data_frames)}")
        
        # FPS计算
        self.frame_count += 1
        current_time = QtCore.QTime.currentTime()
        if self.start_time.msecsTo(current_time) >= 1000:
            fps = self.frame_count
            self.fps_label.setText(f"FPS: {fps}")
            self.frame_count = 0
            self.start_time = current_time
            
        self.current_frame += 10

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 弹出文件选择对话框
    file_path, _ = QFileDialog.getOpenFileName(
        None,
        "选择Routine数据文件",
        "Routine_acq_raw_data",
        "NPZ文件 (*.npz);;CSV文件 (*.csv)"
    )
    
    # 如果用户取消选择，退出程序
    if not file_path:
        print("未选择文件，程序退出。")
        sys.exit(1)
    
    # 文件类型校验
    file_ext = os.path.splitext(file_path)[1].lower()
    if file_ext not in ['.npz', '.csv']:
        print(f"不支持的文件类型: {file_ext}")
        sys.exit(1)
    
    # 配置回放参数（可根据需求调整）
    CONFIG = {
        'interpolation': False,    # 是否启用插值
        'rotation_angle': 0,             # 旋转角度
        'flip_horizontal': False,  # 水平翻转
        'flip_vertical': False     # 垂直翻转
    }
    
    # 创建并显示窗口
    main_win = PlaybackVisualizer(file_path, **CONFIG)
    main_win.resize(800, 800)
    main_win.show()
    
    sys.exit(app.exec_())