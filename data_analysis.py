#!/usr/bin/python3

import serial, io
import matplotlib.pyplot as plt
import time
import datetime

time_limit = 200 # seconds


device   = '/dev/ttyUSB0' # serial port
baud     = 9600                          # baud rate
filename = 'temp_log.txt'                # log file to save data in


times = []
temps = []
flows = []
PID_adjs = []
start_time = time.time()

with serial.Serial(device,baud) as serialPort, open(filename,'w') as outFile:
    while(time.time() - start_time < time_limit):
        line = serialPort.readline() # must send \n! get a line of log
        line = str(line)
        splitline = line.split(" ")

        if "PID_adj" in str(line):
            PID_adj = splitline[5]
            PID_adjs.append(PID_adj)

        if "PID" in str(line[:5]):
            Kp = splitline[3]
            Ki = splitline[4]
            Kd = splitline[5][:-5]


        if "Time" in str(line[:7]):
            thetime = int(splitline[3][:-1])/1000
            Tout = float(splitline[18])
            flow = float(splitline[10])
            print(line)
            times.append(thetime)
            temps.append(Tout)
            flows.append(flow)
            outFile.writelines(line+"\n")          # write line of text to file
            outFile.flush()              # make sure it actually gets written out

avg_flow = 0
for f in flows:
    avg_flow += f/len(flows)
avg_flow *= 15.85 # gpm conversion
avg_flow = round(avg_flow, 1)

fig, ax1 = plt.subplots()
ax2 = ax1.twinx()
ax1.plot(times, temps, 'g-')
ax2.plot(times, PID_adjs, 'b-')

ax1.set(xlabel='Time (s)', ylabel='Temperature (F)',
       title='Water Heater Temperature Plot. Kp=' + Kp +", Ki="+ Ki +", Kd="+ Kd + "\n flow="+str(avg_flow)+"gpm")
ax1.grid()
ax2.set_ylabel('PID Adjustment Factor', color='b')
ax2.set_ylim(0,3.0)
ax1.set_ylim(100,130)

now = datetime.datetime.now()
t = now.strftime("%m%d%y-%H%M%S")

fig.savefig("Temp-plot_"+t+".png")


# fig, ax1 = plt.subplots()
# ax1.plot(flows, temps)
# ax1.set(xlabel='Flow (kg/s)', ylabel='Temperature (F)',
#        title='Water Heater Flow v Temperature Plot. Kp=' + Kp +", Ki="+ Ki +", Kd="+ Kd)
# ax1.grid()
# fig.savefig("Flow_plot_"+Kp+"_"+Ki+"_"+Kd+"-"+str(avg_flow)+"gpm.png")

