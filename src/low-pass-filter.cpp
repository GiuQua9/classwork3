
#include "ros/ros.h"
#include "cwork3/sinusoid.h"
#include "cwork3/low_pass.h"

using namespace std;

//Use a class to store the topic data 
//	Just a choice: the best way to share data between a main loop and the ROS callbacks
class Low_Pass {
	public:
		Low_Pass();
		void topic_cb( cwork3::sinusoid in);
	
	private:
		ros::NodeHandle _nh;
		//Subscriber object
		ros::Subscriber _topic_sin;
		ros::Publisher _topic_pub;
		cwork3::low_pass _data;
		float _LPF_Beta;
};

Low_Pass::Low_Pass() {
	_data.out = 0.0;
	_topic_pub = _nh.advertise<cwork3::low_pass>("/lowpass", 1);
	_LPF_Beta = 0.6; // 0<ß<1
	//Initialize a subscriber:
	//	Input: 	topic name: /numbers
	//			queue:	1
	//			Callback function
	//			Object context: the value of data members
	_topic_sin = _nh.subscribe("/sinusoid", 1, &Low_Pass::topic_cb, this);
}

//Callback function: the input of the function is the data to read
//	In this function, a smart pointer is used
void Low_Pass::topic_cb( cwork3::sinusoid in) {
	
	_data.out = _data.out - (_LPF_Beta * (_data.out - in.v));

	//data is a pointer of std_msgs::float32 type
	//	to access to its fiel, the "." can not be used
	ROS_INFO("Listener: %.3f", _data.out);

	
}

int main( int argc, char** argv ) {

	//Init the ros node with ros_subscriber name
	ros::init(argc, argv, "lowpass_topic");

	//Create the ROS_SUB class object
	Low_Pass ls;
	
	//ros::spin() blocks the main thread from exiting until ROS invokes a shutdown - via a Ctrl + C for example
	// It is written as the last line of code of the main thread of the program.
	//Also the spin invokes the callbacks to flush their queue and process incoming data
	ros::spin(); 

	//----This function will be never overcome

	return 0;
}

