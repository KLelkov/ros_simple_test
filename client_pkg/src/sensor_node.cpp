#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include <iostream>
#include <std_msgs/Float32.h>


  int main(int argc, char** argv)
  {
  	ros::init(argc, argv, "sensor_node");
  	ros::NodeHandle nh;
    ros::Publisher sensorPub;
    sensorPub = nh.advertise<std_msgs::Float32>("sensor_topic", 10);
    ros::Rate rate(2); // ROS Rate at 2Hz
	
	float measurement = 0.0;
	
	while (ros::ok()) {
		measurement += 1.5;
        ROS_INFO("New measurement!");
        std_msgs::Float32 msg;
		msg.data = measurement;
		sensorPub.publish(msg);
        rate.sleep();
    }
	


    //ros::spin();
  	return 0;
  }
