# -*- coding: utf-8 -*-
"""
Created on Sat Sep 29 10:16:05 2018

@author: albert
"""

import numpy as np
from matplotlib import pyplot as plt
from scipy.interpolate import interp1d

pose_data=[]
img_data=[]

##get pose information
with open('/media/albert/新加卷/work_log/interpolation/traj.txt', 'r') as txtData:
    lines=txtData.readlines()
    for line in lines:
        lineData=line.strip().split(' ')  
        pose_data.append(lineData)
time  = np.zeros(len(pose_data))
x     = np.zeros(len(pose_data))
y     = np.zeros(len(pose_data))
z     = np.zeros(len(pose_data))
roll  = np.zeros(len(pose_data))
pitch = np.zeros(len(pose_data))
yaw   = np.zeros(len(pose_data))

for i in range(len(pose_data)):
    time[i]  = float(pose_data[i][1])-1538000000.0 ##for precision
    x[i]     = float(pose_data[i][2])
    y[i]     = float(pose_data[i][3])
    z[i]     = float(pose_data[i][4])
    roll[i]  = float(pose_data[i][5])
    pitch[i] = float(pose_data[i][6])
    yaw[i]   = float(pose_data[i][7])
    
 ##get image time   
with open('/media/albert/新加卷/work_log/interpolation/image_time.txt', 'r') as img_txt:
    lines=img_txt.readlines()
    for line in lines:
        lineData=line.strip().split(' ')  
        img_data.append(lineData)
img_time  = np.zeros(len(img_data)-1) ##the first record of img_time is out of range, so skip
img_num   = np.zeros(len(img_data)-1)
for i in range(len(img_data)-1):
    img_time[i]  = float(img_data[i+1][2])-1538000000.0 ##for precision
    img_num[i]   = int(img_data[i+1][0])
##interpolation for x y z roll pitch yaw
fx=interp1d(time,x,kind='cubic')
image_x=fx(img_time)
fy=interp1d(time,y,kind='cubic')
image_y=fy(img_time)
fz=interp1d(time,z,kind='cubic')
image_z=fz(img_time)
froll=interp1d(time,roll,kind='cubic')
image_roll=froll(img_time)
fpitch=interp1d(time,pitch,kind='cubic')
image_pitch=fpitch(img_time)
fyaw=interp1d(time,yaw,kind='cubic')
image_yaw=fyaw(img_time)

with open('/media/albert/新加卷/work_log/interpolation/image_pose.txt', 'w') as img_pose:
    img_pose.write("Fomat: No. of image / time / x / y / z / roll / pitch / yaw \n")
    for i in range(len(img_time)):
        linewrite= "{0:10s}{1:16.5f} {2:16.6e} {3:16.6e} {4:16.6e} {5:16.6e} {6:16.6e} {7:16.6e}\n".format(str(img_num[i]),
                                                                     img_time[i] + 1538000000.0, image_x[i], image_y[i],
                                                                     image_z[i],image_roll[i], image_pitch[i], image_yaw[i])
        img_pose.write(linewrite)    

##plot the interpolation result
plt.plot(time,x,'r',label='origin')
plt.plot(img_time,image_x,'b--',label='interpolation')
plt.legend()
plt.show()