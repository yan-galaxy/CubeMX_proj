# 首先 pip install pyvisa pyvisa-py psutil zeroconf
import pyvisa 
rm = pyvisa.ResourceManager('@py')  # 使用pyvisa-py后端

# 查找设备（USB或LAN）
# USB设备通常显示为"USB0::...::INSTR"，LAN为"TCPIP0::192.168.1.100::INSTR"
devices = rm.list_resources()
print("可用设备：", devices)

# 连接设备（替换为实际资源名称）
load = rm.open_resource(devices[0])# "ASRL10::INSTR"

# 发送识别命令，验证连接
print(load.query("*IDN?"))  # 应返回设备信息：ITECH,IT85XX,...






# 读取当前电压（单位：V）
voltage = load.query(":MEAS:VOLT?")
print(f"电压：{voltage} V")

# 读取当前电流（单位：A）
current = load.query(":MEAS:CURR?")
print(f"电流：{current} A")

# 读取功率（单位：W）
power = load.query(":MEAS:POW?")
print(f"功率：{power} W")




# 关闭输入
load.write(":INP OFF")

# 断开连接
load.close()
rm.close()