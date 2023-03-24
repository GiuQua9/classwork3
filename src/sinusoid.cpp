
//include ros header file
#include "ros/ros.h" 
//header file of the custom message
//  This message belongs to the package in which it is defined
#include "cwork3/sinusoid.h"
#include "math.h"


int main(int argc, char **argv) {

	//Initialize the ROS node with name: ros_topic_publisher
	ros::init(argc, argv,"sinusoide_topic");
	
	//Declare the node handle: our interface with the ROS system
	ros::NodeHandle nh;

	//Create a publisher object:
	//	Input:  - type of message: std_msgs::Int32
	//			- topic name: /numbers
	//			- message queue: 10 (0: infinite queue)
	ros::Publisher topic_sin = nh.advertise<cwork3::sinusoid>("/sinusoid", 1);


    //Define the custom datatype
    cwork3::sinusoid data;
	if(argc > 1){
		data.amp = atof(argv[1]);
		data.period = atof(argv[2]);
	}

	//printf("amp: %f\tfreq: %f\n",data.amp,1/data.period);

	//Rate object: 100 Hz of rate
	ros::Rate rate(10); 

	int count=0;


	// Typical loop: neverending loop: a controller works until actuators are activated
	//		while (ros::ok()): works until the ROS node is not terminated (by the user with ctrl+c or similar)
	while ( ros::ok() ) {

        //Fill the data part
		data.v = data.amp*sin(2*3.14*(1/data.period)*count*0.1);
		//ROS_INFO: Like a printf, but with the timestamp
		ROS_INFO("sin: %.3f\tcount: %d",data.v,count); 

		//Publish the message over the ROS network
		topic_sin.publish(data);

		count++;
		
		//Rate to maintain the 100 Hz
		rate.sleep();
	}
	
	return 0;
}
