# Catamaran Hydrofoil USV with Target Tracking Based on ROS2  基于ROS2的目标追踪双体水翼无人船

一.主要功能  
开发了基于ROS2与OpenCV的自动避障及目标追踪双体水翼海警船，通过双体水翼船的结构设计，实现了高速、稳定的目标；基于OpenCV实现目标追踪系统，基于ROS2实现雷达自动避障导航功能，为自主避碰和目标追踪提供依据。

二.主要设备及系统  
1.树莓派4B  
2.思岚C1激光雷达  
3.Gemini深度相机  
4.无刷电机  
5.ubuntu22.04 + ROS2 Humble Hawksbill版本

三.环境配置与库安装  
1.换源换镜像+安装ROS2  
在命令行输入鱼香ROS快捷指令：  
`wget http://fishros.com/install -O fishros && . fishros`,  
自动换源，安装qq，安装ROS2——humble（Ubuntu22.04 desktop 64）

2.安装ssh，连接putty  
在命令行输入：  
`sudo apt-get update` 
`sudo apt-get install openssh-server ` 
`Ifconfig`

3.安装pigpio库  
pigpio能够在树莓派等嵌入式平台上产生PWM波以驱动无刷电机，直接pip会报错，因此采用以下方式：  
在命令行输入：  
`wget https://github.com/joan2937/pigpio/archive/master.zip #克隆项目  
unzip master.zip   
cd pigpio-master  
make`  
安装：  
`sudo apt install python-setuptools python3-setuptools  
sudo make install  
sudo pigpiod`  
运行test,py测试PWM信号驱动电机：  
`cd longwang_ws  
cd test_1  
cd test_1  
python3 test.py `  
若电机转动，则配置完成。

4.配置VNC  
为了在PC端进行树莓派可视化编程，需要配置VNC，首先在PC端安装VNC Viewer软件，  
使用putty远程控制树莓派命令行，输入：  
`sudo apt update  
sudo apt upgrade  
sudo apt install xfce4 xfce4-goodies tightvncserver`  
安装后，首次打开vnc连接：  
`vncserver  
vncserver -kill :1`  
此时无法连接，打开配置文件：
`nano ~/.vnc/xstartup`
黏贴以下内容：
`#!/bin/bash
xrdb $HOME/.Xresources
startxfce4`
退出后在命令行输入：
`chmod +x ~/.vnc/xstartup`
之后，打开service文件，
`sudo nano /etc/systemd/system/vncserver@.service`
黏贴以下内容：
`[Unit]
Description=Start TightVNC server at startup
After=syslog.target network.target
[Service]
Type=forking
User=<username>
PAMName=login
PIDFile=/home/<username>/.vnc/%H:%i.pid
ExecStartPre=-/usr/bin/vncserver -kill :%i > /dev/null 2>&1
ExecStart=/usr/bin/vncserver -geometry 1920x1080 -depth 24 -dpi 96 :%i
ExecStop=/usr/bin/vncserver -kill :%i
[Install]
WantedBy=multi-user.target`
再次授予权限：
`sudo systemctl daemon-reload
sudo systemctl enable vncserver@1.service
sudo systemctl start vncserver@1.service`
至此，vnc配置完毕。
