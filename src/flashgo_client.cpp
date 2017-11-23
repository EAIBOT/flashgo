/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2017 EAI TEAM
 *  http://www.eaibot.com
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define RAD2DEG(x) ((x)*180./M_PI)

int round_double(double number)
{
    return (number > 0.0) ? (number + 0.5) : (number - 0.5); 
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = round_double(scan->scan_time / scan->time_increment);
    ROS_INFO("[EAI INFO]: I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    ROS_INFO("[EAI INFO]: angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flash_lidar_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();

    return 0;
}
