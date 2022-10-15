#!/usr/local/bin/python3

import serial, io
import matplotlib.pyplot as plt
import time

time_limit = 200 # seconds


device   = '/dev/ttyUSB0' # serial port
baud     = 9600                          # baud rate
filename = 'temp_log.txt'                # log file to save data in


times = []
temps = []
flows = []
start_time = time.time()

with serial.Serial(device,baud) as serialPort, open(filename,'w') as outFile:
    while(time.time() - start_time < time_limit):
        line = serialPort.readline() # must send \n! get a line of log
        line = str(line)
        splitline = line.split(" ")

        if "PID" in str(line[:5]):
            Kp = splitline[3]
            Ki = splitline[4]
            Kd = splitline[5][:-5]


        if "Time" in str(line[:7]):
            thetime = int(splitline[3][:-1])/10000
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

fig, ax = plt.subplots()
ax.plot(times, temps)

ax.set(xlabel='Time (s)', ylabel='Temperature (F)',
       title='Water Heater Temperature Plot. Kp=' + Kp +", Ki="+ Ki +", Kd="+ Kd + "\n flow="+str(avg_flow)+"gpm")
ax.grid()

fig.savefig("Temp_plot_"+Kp+"_"+Ki+"_"+Kd+"-"+str(avg_flow)+"gpm.png")

