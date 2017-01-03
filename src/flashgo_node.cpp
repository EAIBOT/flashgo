/*
 *  FLASH LIDAR SYSTEM
 *  Flash Lidar ROS Node Client 
 *
 *  Copyright 2015 - 2017 EAI TEAM
 *  http://www.eaibot.com
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "flashgo.h"
#include <vector>
#include <iostream>
#include <string>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define NODE_COUNTS 720
#define EACH_ANGLE 0.5
#define DEG2RAD(x) ((x)*M_PI/180.)

void publish_scan(ros::Publisher *pub, 
                  node_info *nodes, 
                  size_t node_count, ros::Time start,
                  double scan_time,
                  float angle_min, float angle_max, 
                  std::string frame_id, std::vector<int> ignore_array , double ignore_value)
{
    static int scan_count = 0;
    sensor_msgs::LaserScan scan_msg;

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    scan_count++;

    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count-1);

    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(node_count-1);

    scan_msg.range_min = 0.08;
    scan_msg.range_max = 15.;

    scan_msg.ranges.resize(node_count);
    scan_msg.intensities.resize(node_count);
    for (size_t i = 0; i < node_count; i++) {
        scan_msg.ranges[i] = (float)nodes[i].distance_q2/4.0f/1000;
        scan_msg.intensities[i] = (float) (nodes[i].sync_quality >> 2);
        //if(scan_msg.ranges[i]!=0){
        //    fprintf(stderr, "scan_msg.ranges[%d] : %f .\n" , (int)i , (float)scan_msg.ranges[i]);
        //}
    }
    //fprintf(stderr, "---------------------------------------------------\n");

    if(ignore_array.size() != 0)
    {
	for (size_t i = 0; i < node_count; i++) {
            float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
            for(unsigned int j = 0; j < ignore_array.size();j = j+2){
               if((ignore_array[j] < angle) && (angle <= ignore_array[j+1])){
	           scan_msg.ranges[i] = ignore_value;
                   break;
               }
            }
        }
    }

    pub->publish(scan_msg);
}

bool checkFlashLidarHealth(Flashgo * drv)
{
    int op_result;
    device_health healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (op_result == 0) { 
        printf("Flash Lidar running correctly ! The health status is %d\n", healthinfo.status);
        
        if (healthinfo.status == 2) {
            fprintf(stderr, "Error, Flash Lidar internal error detected. Please reboot the device to retry.\n");
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve Flash Lidar health code: %x\n", op_result);
        return false;
    }
}


std::vector<int> split(const std::string &s, char delim) {
    std::vector<int> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atoi(number.c_str()));
    }
    return elems;
}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "flash_lidar_node");

    std::string serial_port;
    int serial_baudrate;
    std::string frame_id;
    bool angle_compensate;

    std::string list;
    std::vector<int> ignore_array;
    double ignore_value;

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyACM0"); 
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200); 
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("angle_compensate", angle_compensate, "true");
    nh_private.param<std::string>("ignore_array",list,"");
    nh_private.param<double>("ignore_value", ignore_value, 0.1);

    ignore_array = split(list ,',');
    if(ignore_array.size()%2){
        ROS_ERROR_STREAM("ignore array is odd need be even");
    }

    for(unsigned int i =0 ; i < ignore_array.size();i++){
        if(ignore_array[i] < 0 ||ignore_array[i] > 360){
            ROS_ERROR_STREAM("ignore array should be 0<=  <=360");
        }
    }

    u_int32_t op_result;

    // create the driver instance
    Flashgo * drv = new Flashgo();
    
    if (!drv) {
        fprintf(stderr, "Create Driver fail, exit\n");
        return -2;
    }

    // make connection...
    op_result = drv->connect(serial_port.c_str(), (u_int32_t)serial_baudrate);
    if (op_result == -1) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n" , serial_port.c_str());
        return -1;
    }else{
        int isEAI = drv->getEAI();
        if(isEAI != 0){
	    drv->disconnect();
            op_result = drv->connect(serial_port.c_str(), (u_int32_t)serial_baudrate);
    	    if (op_result == -1) {
	        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n" , serial_port.c_str());
	        return -1;
            }else{
	        isEAI = drv->getEAI();
	        if(isEAI != 0){
		    fprintf(stderr, "EAI Error, cannot bind to the specified serial port %s.\n" , serial_port.c_str());
		    return -1;
	        }
	    }
        }
    }

    // check health...
    if (!checkFlashLidarHealth(drv)) {
        return -1;
    }

    // start scan...
    drv->startScan();

    ros::Time start_scan_time;
    ros::Time end_scan_time;
    double scan_duration;
    ros::Rate rate(60);

    const int angle_compensate_nodes_count = NODE_COUNTS;
    node_info angle_compensate_nodes[angle_compensate_nodes_count];

    while (ros::ok()) {
        node_info nodes[360*2];
        size_t   count = _countof(nodes);

        start_scan_time = ros::Time::now();
        op_result = drv->grabScanData(nodes, count);
        end_scan_time = ros::Time::now();
        scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;

        if (op_result == 0) {
            op_result = drv->ascendScanData(nodes, count);
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(360.0f);
            if (op_result == 0) {
                if (angle_compensate) {
                    memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(node_info));
                    int i = 0 ;
                    for( ; i < count; i++) {
                        if (nodes[i].distance_q2 != 0) {
                            float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                            int inter =(int)( angle / EACH_ANGLE );
                            float angle_pre = angle - inter * EACH_ANGLE;
                            float angle_next = (inter+1) * EACH_ANGLE - angle;
                            if(angle_pre < angle_next){
                                angle_compensate_nodes[inter]=nodes[i];
                            }else{
                                angle_compensate_nodes[inter+1]=nodes[i];
                            }
                        }
                    }

                    publish_scan(&scan_pub, angle_compensate_nodes, angle_compensate_nodes_count,
                             start_scan_time, scan_duration,  
                             angle_min, angle_max, 
                             frame_id,ignore_array,ignore_value);
                } else {
                    int start_node = 0, end_node = 0;
                    int i = 0;
                    // find the first valid node and last valid node
                    while (nodes[i++].distance_q2 == 0);
                    start_node = i-1;
                    i = count -1;
                    while (nodes[i--].distance_q2 == 0);
                    end_node = i+1;

                    angle_min = DEG2RAD((float)(nodes[start_node].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                    angle_max = DEG2RAD((float)(nodes[end_node].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);

                    publish_scan(&scan_pub, &nodes[start_node], end_node-start_node +1, 
                             start_scan_time, scan_duration, 
                             angle_min, angle_max, 
                             frame_id,ignore_array,ignore_value);
               }
            } else if (op_result == -2) {
                // All the data is invalid, just publish them
                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(360.0f);

                publish_scan(&scan_pub, nodes, count, 
                             start_scan_time, scan_duration,  
                             angle_min, angle_max, 
                             frame_id,ignore_array,ignore_value);
            }
        }
        rate.sleep();
        ros::spinOnce();
    }

    drv->stop();
    drv->disconnect();
    return 0;
}
