#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>


/* Communication Class */
class SubscribeAndPublish
{
public:

private:
    // Parameter initial values
    float last_measurement;
    
    // ROS variables
    // Although the subscribers are never actually used,
    // Aparently you need them for you subscription to stay alive
    ros::Subscriber sensorSub;
    ros::Subscriber requestSub;
    ros::NodeHandle nh;
    ros::Publisher dataPub;

public:
    SubscribeAndPublish() // This is the constructor
    {
		last_measurement = 0.0;
		
		dataPub = nh.advertise<std_msgs::Float32>("data_topic", 10);
		sensorSub = nh.subscribe<std_msgs::Float32>("sensor_topic", 10, &SubscribeAndPublish::sensor_callback, this);
		requestSub = nh.subscribe<std_msgs::String>("request_topic", 100, &SubscribeAndPublish::request_callback, this);

		printf("Server is ready!\n");
    }


    void sensor_callback(const std_msgs::Float32 msg)
    {
      last_measurement = msg.data;
    }


    void request_callback(const std_msgs::String msg)
    {
		//printf("\n");
		//printf(msg.data);
		printf("Request received\n");
		if (true)
		{
			std_msgs::Float32 out_msg;
			out_msg.data = (float)last_measurement * 10.0;
			dataPub.publish(out_msg);
		}
    }

  };//End of class SubscribeAndPublish


  int main(int argc, char** argv)
  {
  	ros::init(argc, argv, "server_node");

	SubscribeAndPublish SAPObject;

    ros::spin();

  	return 0;
  }
