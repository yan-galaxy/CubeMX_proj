target_cutfreq = 1000.0 # 目标截至频率 单位Hz
freq = 170000000.0/2.0
OUT = int(freq/(target_cutfreq*100.0))-1
print("计算最接近的Prescaler:",OUT)
real_cutfreq = freq/(OUT+1)/100.0
print("根据Prescaler反推真实截止频率",real_cutfreq,"Hz")
error = (real_cutfreq-target_cutfreq)/target_cutfreq
print("误差：",error*100,"%")