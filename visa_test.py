# 首先 pip install pyvisa pyvisa-py psutil zeroconf pyserial
import pyvisa 
import time

rm = pyvisa.ResourceManager('@py')  # 使用pyvisa-py后端

# 查找设备（USB或LAN）
# USB设备通常显示为"USB0::...::INSTR"，LAN为"TCPIP0::192.168.1.100::INSTR"
devices = rm.list_resources()
print("可用设备：", devices)

# 连接设备（替换为实际资源名称）
load = rm.open_resource(devices[0])# "ASRL10::INSTR"
load.timeout = 2000  # 超时时间2秒

# 发送识别命令，验证连接
print(load.query("*IDN?"))  # 应返回设备信息：ITECH,IT85XX,...

load.write("SYSTem:BEEPer:IMMediate")
time.sleep(1.0)

# 读取当前电压（单位：V）
voltage = load.query("MEAS:VOLT?")
print(f"电压：{voltage} V")
# 读取当前电流（单位：A）
current = load.query("MEAS:CURR?")
print(f"电流：{current} A")
# 读取功率（单位：W）
power = load.query("MEAS:POW?")
print(f"功率：{power} W")



print(load.query(":SYST:COMM:SEL?"))
# # 2. 初始化设备
# load.write("*RST")  # 恢复出厂设置
# time.sleep(2.0)

# 清除错误队列和保护状态
load.write(":SYST:CLE")  # 清除错误缓存
print("清除错误后状态:", load.query(":SYST:ERR?"))  # 应返回"0,No error"
time.sleep(0.5)

# 3. 配置电流模式及动态参数
load.write("SOUR:MODE CURR")  # 切换到定电流模式 CURRent VOLTage POWer RESistance DYNamic LED LIST
print("当前模式:", load.query("SOURce:FUNCtion?"))  # 应返回CURR
# print("FUNCtion当前模式:", load.query(":SOURce:FUNCtion:MODE?"))  # 应返回CURR
# load.write(":DYNamic:HIGH:LEVel 1")  # 高电平电流1A
# load.write(":DYNamic:LOW:LEVel 0")  # 低电平电流0.5A
# load.write(":DYNamic:HIGH:DWEL 1")  # 高电平持续0.5s
# load.write(":DYNamic:LOW:DWEL 1")  # 低电平持续0.5s

# load.write(":DYNamic:MODE CONTinuous")  # 设置动态模式为连续触发

# # 4. 启用动态模式并开始带载
# load.write(":TRAN:STAT ON")  # 开启动态测试
# load.write(":INP ON")  # 开启输入（开始带载）

# print("输入状态（1=开启，0=关闭）:", load.query(":INP?"))  # 应返回1
# print("动态模式状态（1=开启，0=关闭）:", load.query(":TRAN:STAT?"))  # 应返回1
# print("动态模式类型:", load.query(":DYNamic:MODE?"))  # 应返回CONT（连续模式）
# print("高电平电流设定值:", load.query(":DYNamic:HIGH:LEVel?"))  # 应返回1.0
# print("低电平电流设定值:", load.query(":DYNamic:LOW:LEVel?"))  # 应返回0.5

time.sleep(0.5)
print("设备错误信息:", load.query(":SYST:ERR?"))  # 返回错误码和描述



# # 5. 运行10秒后停止
# time.sleep(5)



# # 关闭输入
# load.write(":INP OFF")
# load.write(":TRAN:STAT OFF")  # 关闭动态模式

# 断开连接
load.close()
rm.close()