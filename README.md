# Use-sensors-to-calibrate-human-body-coordinates
基于LPMS传感器（与官方的openZen库）获取传感器数据，同时基于传感器的值，进行世界坐标系，传感器坐标系，人体坐标系的坐标矩阵换算，对人的姿态进行标定。

（个人在暑假期间复现了哈尔滨工业大学刘玉焘博士的论文《基于可穿戴式传感器的人体动作捕获与识别研究》的一部分内容，其中仍有一小部分的不同坐标系之间的矩阵换算的错误，会在逐步学习中改正）

同时，这也是我初步学习完《head first in 设计模式》对于所学到的几种设计模式的一个初步的应用；所使用的传感器链接为https://www.alubi.cn/support/download/ ,openZen的官方库为https://bitbucket.org/lpresearch/openzen/src/master/


Obtain sensor data based on the LPMS sensor (and the official openZen library), and at the same time, based on the value of the sensor, perform the coordinate matrix conversion of the world coordinate system, the sensor coordinate system, and the human body coordinate system to calibrate the human posture.

(I reproduced part of the thesis "Research on Human Motion Capture and Recognition Based on Wearable Sensors" by Dr. Liu Yutao of Harbin Institute of Technology during the summer vacation. There are still a small number of errors in matrix conversion between different coordinate systems. Will be corrected in step-by-step learning)

At the same time, this is also a preliminary application of the several design patterns I have learned after the preliminary study of "head first in design patterns".
