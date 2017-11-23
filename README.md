FLASH LIDAR ROS PACKAGE V2.3.2
=====================================================================

ROS node and test application for FLASH LIDAR

Visit EAI Website for more details about FLASH LIDAR.

How to build FLASH LIDAR ros package
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    2) Running catkin_make to build flashgo_node and flashgo_client
    3) Create the name "/dev/flashlidar" for flash lidar
    --$ roscd flashgo/startup
    --$ sudo chmod 777 ./*
    --$ sudo sh initenv.sh

How to run FLASH LIDAR ros package
=====================================================================
There're two ways to run FLASH LIDAR ros package

1. Run FLASH LIDAR node and view in the rviz
------------------------------------------------------------
roslaunch flashgo lidar_view.launch

You should see FLASH LIDAR's scan result in the rviz.

2. Run FLASH LIDAR node and view using test application
------------------------------------------------------------
roslaunch flashgo lidar.launch

rosrun flashgo flashgo_client

You should see FLASH LIDAR's scan result in the console
