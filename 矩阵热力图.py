# from json.tool import main
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, mkQApp, QtGui
import threading
import serial
from skimage import transform
from scipy.ndimage import gaussian_filter

def flip90_right(arr):
    new_arr = arr.reshape(arr.size)
    new_arr = new_arr[::-1]
    new_arr = new_arr.reshape(arr.shape)
    new_arr = np.transpose(new_arr)[::-1]
    return new_arr


def flip180(arr):
    new_arr = arr.reshape(arr.size)
    new_arr = new_arr[::-1]
    new_arr = new_arr.reshape(arr.shape)
    return new_arr


def Serial():
   global corrMatrix
   global Matrix0

   while True:  
        try:
            response = ser.readline()
            row = response.decode('utf-8')[0:-3].split(';') 
            row = np.array(row).astype(int)
            corrMatrix1 = row.reshape((10, 10)) - Matrix0
            corrMatrix1 = np.where(corrMatrix1 <200, 0, corrMatrix1)
            corrMatrix1 = corrMatrix1.transpose() / 500  
            corrMatrix1= flip90_right(corrMatrix1)
            corrMatrix=flip180(corrMatrix1)
            corrMatrix = transform.resize(corrMatrix, (70, 70), order=5)
            ser.flushInput()
        except KeyboardInterrupt:
            ser.close()
            quit()
        except:
            print('fail receiving')






mkQApp("Correlation matrix display")
pg.setConfigOption('background', 'w')
main_window = QtWidgets.QMainWindow()
gr_wid = pg.GraphicsLayoutWidget(show=True)
main_window.setCentralWidget(gr_wid)
main_window.setWindowTitle('压阻阵列')
main_window.resize(700, 700)  
main_window.show()


corrMatrix = np.random.rand(10, 10)
columns = list(range(1, 11))

pg.setConfigOption(
    'imageAxisOrder', 'row-major'
)  
tr = QtGui.QTransform().translate(
    10, 10)  
correlogram = pg.ImageItem()

correlogram.setTransform(tr)

colorMap = pg.colormap.get(
    "cividis")  

bar = pg.ColorBarItem(values=(0, 1), colorMap=colorMap)

plotItem = gr_wid.addPlot()  
plotItem.invertY(True) 
plotItem.setDefaultPadding(
    0.0)  
plotItem.addItem(correlogram)  


plotItem.showAxes(True, showValues=(True, True, False, False), size=20)


ticks = [(idx, label) for idx, label in enumerate(columns)]
for side in ('left', 'top', 'right', 'bottom'):
    plotItem.getAxis(side).setTicks(
        (ticks, []))  
plotItem.getAxis('bottom').setHeight(
    10)  


correlogram.setImage(corrMatrix)
bar.setImageItem(correlogram, insert_in=plotItem)


def setImageItem(coorelogram):
    bar.setImageItem(coorelogram, insert_in=plotItem)


def plotData():
    correlogram.setImage(corrMatrix)
    bar.setImageItem(correlogram, insert_in=plotItem)


# Start Qt event loop
if __name__ == '__main__':
    ser = serial.Serial('COM22', 115200,timeout=1)
    ser.close()
    ser.open()
    count = 0
    temp = np.zeros(100)
    while count < 10: 
        try:
            response0 = ser.readline()
            row0 = response0.decode('utf-8')[0:-3].split(';')
            temp += np.array(row0).astype(int)
            print('initial:{}'.format(count))
            count += 1
            ser.flushInput()
        except KeyboardInterrupt:
            ser.close()
            quit()
        except Exception as e:
            print(e)
    Matrix0 = (temp / count).reshape((10, 10))
    print(Matrix0)
    th1 = threading.Thread(target=Serial)
    th1.start()
    timer = pg.QtCore.QTimer()
    timer.timeout.connect(plotData)
    timer.start(1)
    pg.exec()
