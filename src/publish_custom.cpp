
//include ros header file
#include "ros/ros.h" 
//header file of the custom message
//  This message belongs to the package in which it is defined
#include "ros_topic_custom/my_msg.h"


int main(int argc, char **argv) {

	//Initialize the ROS node with name: ros_topic_publisher
	ros::init(argc, argv,"ros_topic_publisher");
	
	//Declare the node handle: our interface with the ROS system
	ros::NodeHandle nh;

	//Create a publisher object:
	//	Input:  - type of message: std_msgs::Int32
	//			- topic name: /numbers
	//			- message queue: 10 (0: infinite queue)
	ros::Publisher topic_pub = nh.advertise<ros_topic_custom::my_msg>("/custom", 1);

	//Rate object: 10 Hz of rate
	ros::Rate rate(10); 

	int count = 0;

    //Define the custom datatype
    ros_topic_custom::my_msg data;
    //Fill the name part
    data.name = "custom datatype";


	// Typical loop: neverending loop: a controller works until actuators are activated
	//		while (ros::ok()): works until the ROS node is not terminated (by the user with ctrl+c or similar)
	while ( ros::ok() ) {

        //Fill the data part
		data.data = count++;

		//ROS_INFO: Like a printf, but with the timestamp
		ROS_INFO("%d",data.data); 

		//Publish the message over the ROS network
		topic_pub.publish(data);
		
		//Rate to maintain the 10 Hz
		rate.sleep();
	}
	
	return 0;
}
