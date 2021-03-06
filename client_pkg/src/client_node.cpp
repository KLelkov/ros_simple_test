#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>


    void data_callback(const std_msgs::Float32 msg)
    {

		printf("Data received!\n");
		printf("received: %f \n", (float)msg.data);
		//printf("\n");
    }


  int main(int argc, char** argv)
  {
  	ros::init(argc, argv, "client_node");
  	ros::NodeHandle nh;
    ros::Publisher requestPub;
    ros::Subscriber dataSub;
    requestPub = nh.advertise<std_msgs::String>("request_topic", 10);
	dataSub = nh.subscribe("data_topic", 100, data_callback);
ros::Duration(0.5).sleep(); // sleep for half a second
	std_msgs::String msg;
	msg.data = "Please, give me some data";
	requestPub.publish(msg);


	ros::spin();

  	return 0;
  }
