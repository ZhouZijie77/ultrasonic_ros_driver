# Introduction
This repo is the ROS driver of **F40-16TR7B ultrasonic sensor**.
# Hardware connection
ultrasonic sensor(CAN)---USBCAN---PC
# Dependecies
[CH341SER_LINUX](https://www.wch.cn/download/CH341SER_LINUX_ZIP.html)

# How to use
```bash
mkdir -p workspace/src
cd workspace/src
git clone https://github.com/ZhouZijie77/ultrasonic_ros_driver.git
cd ..
catkin_make
source devel/setup.bash
roslaunch ultrasonic_ros_driver ultra.launch
```




