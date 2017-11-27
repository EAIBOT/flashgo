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
#include <signal.h>

#define NODE_COUNTS 720
#define EACH_ANGLE 0.5
#define DELAY_SECONDS 26
#define DEG2RAD(x) ((x)*M_PI/180.)

static bool flag = true;

void publish_scan(ros::Publisher *pub,  node_info *nodes,  size_t node_count, ros::Time start, double scan_time, float angle_min, float angle_max, std::string frame_id, std::vector<int> ignore_array)
{
    sensor_msgs::LaserScan scan_msg;
    float nodes_array[node_count];
    float quality_array[node_count];
    for (size_t i = 0; i < node_count; i++) {
        if(i<node_count/2){
            nodes_array[node_count/2-1-i] = (float)nodes[i].distance_q2/4.0f/1000;
            quality_array[node_count/2-1-i] = (float)(nodes[i].sync_quality >> 2);
        }else{
            nodes_array[node_count-1-(i-node_count/2)] = (float)nodes[i].distance_q2/4.0f/1000;
            quality_array[node_count-1-(i-node_count/2)] = (float)(nodes[i].sync_quality >> 2);
        }

        if(ignore_array.size() != 0){
	    float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
            if(angle>180){
                angle=360-angle;
            }else{
                angle=-angle;
            }
	    for(unsigned int j = 0; j < ignore_array.size();j = j+2){
                if((ignore_array[j] < angle) && (angle <= ignore_array[j+1])){
                    if(i<node_count/2){
                        nodes_array[node_count/2-1-i] = 0;
                    }else{
                        nodes_array[node_count-1-(i-node_count/2)] = 0;
                    }
		    break;
		}
	    }
	}

    }

    int counts = node_count*((angle_max-angle_min)/360.0f);
    
    int angle_start = 180+angle_min;
    int node_start = node_count*(angle_start/360.0f);

    scan_msg.ranges.resize(counts);
    scan_msg.intensities.resize(counts);
    for (size_t i = 0; i < counts; i++) {
        scan_msg.ranges[i] = nodes_array[i+node_start];
        scan_msg.intensities[i] = quality_array[i+node_start];
    }

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    float radian_min = DEG2RAD(angle_min);
    float radian_max = DEG2RAD(angle_max);
    scan_msg.angle_min = radian_min;
    scan_msg.angle_max = radian_max;
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)counts;
    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)counts;
    scan_msg.range_min = 0.08;
    scan_msg.range_max = 15.;

    pub->publish(scan_msg);
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

static void Stop(int signo)   
{  
    
    printf("Received exit signal\n");
    flag = false;
    Flashgo::singleton()->disconnect();
    Flashgo::done(); 
    exit(1);
     
} 

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "flash_lidar_node");

    std::string port;
    int baudrate;
    std::string frame_id;
    bool angle_fixed;
    double angle_max,angle_min;
    u_int32_t op_result;

    std::string list;
    std::vector<int> ignore_array;

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("port", port, "/dev/ttyACM0"); 
    nh_private.param<int>("baudrate", baudrate, 115200); 
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("angle_fixed", angle_fixed, "true");
    nh_private.param<double>("angle_max", angle_max , 180);
    nh_private.param<double>("angle_min", angle_min , -180);
    nh_private.param<std::string>("ignore_array",list,"");

    ignore_array = split(list ,',');
    if(ignore_array.size()%2){
        ROS_ERROR_STREAM("ignore array is odd need be even");
    }

    for(unsigned int i =0 ; i < ignore_array.size();i++){
        if(ignore_array[i] < -180 && ignore_array[i] > 180){
            ROS_ERROR_STREAM("ignore array should be -180<=  <=180");
        }
    }


    Flashgo::initDriver(); 
    if (!Flashgo::singleton()) {
        fprintf(stderr, "[EAI ERROR]: Create Driver fail, exit\n");
        return -2;
    }
    signal(SIGINT, Stop); 
    signal(SIGTERM, Stop);

    op_result = Flashgo::singleton()->connect(port.c_str(), (u_int32_t)baudrate);
    if (op_result == -1) {
        int seconds=0;
        while((op_result == -1) && (seconds <= DELAY_SECONDS) && flag){
            sleep(2);
            seconds = seconds + 2;
            Flashgo::singleton()->disconnect();
            op_result = Flashgo::singleton()->connect(port.c_str(), (u_int32_t)baudrate);
            fprintf(stdout, "[EAI INFO]: Try to connect the port %s again  after %d s .\n", port.c_str() , seconds);
            if(op_result==0){
                break;
            }
        }
        
        if(seconds > DELAY_SECONDS){
            fprintf(stderr, "[EAI ERROR]: Cannot bind to the specified serial port %s.\n" , port.c_str());
	    Flashgo::singleton()->disconnect();
	    Flashgo::done();
            return -1;
        }
    }

    fprintf(stdout, "[EAI INFO]: Connected the port %s , start to scan ......\n", port.c_str());
    int ans=Flashgo::singleton()->startScan();
    if(ans != 0){
	ans = Flashgo::singleton()->startScan();
	if(ans !=0){
	    fprintf(stdout, "Start LIDAR is failed! Exit!! ......\n");
	    Flashgo::singleton()->disconnect();
	    Flashgo::done();
	}
     }
    fprintf(stdout, "[EAI INFO]: Now LIDAR is scanning ......\n");
    flag = false;

    ros::Time start_scan_time;
    ros::Time end_scan_time;
    double scan_duration;
    ros::Rate rate(30);

    node_info all_nodes[NODE_COUNTS];
    memset(all_nodes, 0, NODE_COUNTS*sizeof(node_info));

    while (ros::ok()) {
        try{
            node_info nodes[NODE_COUNTS];
            size_t   count = _countof(nodes);

            start_scan_time = ros::Time::now();
            op_result = Flashgo::singleton()->grabScanData(nodes, count);

            end_scan_time = ros::Time::now();
            scan_duration = (end_scan_time - start_scan_time).toSec();

            if (op_result == 0) {
                op_result = Flashgo::singleton()->ascendScanData(nodes, count);
            
                if (op_result == 0) {
                    if (angle_fixed) {
                        memset(all_nodes, 0, NODE_COUNTS*sizeof(node_info));
                        int i = 0 ;
                        for( ; i < count; i++) {
                            if (nodes[i].distance_q2 != 0) {
                                float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                                int inter =(int)( angle / EACH_ANGLE );
                                float angle_pre = angle - inter * EACH_ANGLE;
                                float angle_next = (inter+1) * EACH_ANGLE - angle;
                                if(angle_pre < angle_next){
                                    all_nodes[inter]=nodes[i];
                                }else{
                                    all_nodes[inter+1]=nodes[i];
                                }
                            }
                        }
                        publish_scan(&scan_pub, all_nodes, NODE_COUNTS, start_scan_time, scan_duration, angle_min, angle_max, frame_id, ignore_array);
                    } else {
                        int start_node = 0, end_node = 0;
                        int i = 0;
                        while (nodes[i++].distance_q2 == 0);
                        start_node = i-1;
                        i = count -1;
                        while (nodes[i--].distance_q2 == 0);
                        end_node = i+1;

                        angle_min = (float)(nodes[start_node].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
                        angle_max = (float)(nodes[end_node].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;

                        publish_scan(&scan_pub, &nodes[start_node], end_node-start_node +1,  start_scan_time, scan_duration, angle_min, angle_max, frame_id, ignore_array);
                    }
                }
            }else if(op_result == -2){
                // All the data is invalid, just publish them
                publish_scan(&scan_pub, all_nodes, NODE_COUNTS, start_scan_time, scan_duration,angle_min, angle_max,frame_id,ignore_array);
            }

            rate.sleep();
            ros::spinOnce();
	}catch(std::exception &e){//
            std::cerr << "Unhandled Exception: " << e.what() << std::endl;
    	    Flashgo::singleton()->disconnect();
	    printf("[EAI INFO]: Now LIDAR is stopping .......\n");
    	    Flashgo::done();
	    return 0;
	}catch(...){//anthor exception
	    std::cerr << "Unhandled Exception:Unknown " <<std::endl;
    	    Flashgo::singleton()->disconnect();
	    printf("[EAI INFO]: Now LIDAR is stopping .......\n");
    	    Flashgo::done();
	    return 0;
	}
    }

    Flashgo::singleton()->disconnect();
    printf("[EAI INFO]: Now LIDAR is stopping .......\n");
    Flashgo::done();
    return 0;
}
