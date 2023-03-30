
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "boost/thread.hpp"
#include "math.h"
#include "cwork3/sinusoid.h"
#include "cwork3/low_pass.h"

using namespace std;

//Use a class to store the topic data 
//	Just a choice: the best way to share data between a main loop and the ROS callbacks
class Low_Pass {
	public:
		Low_Pass();
		void topic_cb( cwork3::sinusoid in); //callback
		void filter();
	
	private:
		ros::NodeHandle _nh;
		ros::Subscriber _topic_sin;
		ros::Publisher _topic_pub;
		float _in;
		float _LPF_Beta;
		cwork3::low_pass _out;
};

Low_Pass::Low_Pass() {
	_in = 0.0;
	_out.out= 0.0;
	_topic_pub = _nh.advertise<cwork3::low_pass>("/lowpass", 1);
	_LPF_Beta = 1/(1+0.01); // 0<ß<1
	//Initialize a subscriber:
	//	Input: 	topic name: /sinusoid
	//			queue:	1
	//			Callback function
	//			Object context: the value of data members
	_topic_sin = _nh.subscribe("/sinusoid", 1, &Low_Pass::topic_cb, this);
	boost::thread loop_t( &Low_Pass::filter, this);
}

//Callback function: the input of the function is the data to read
void Low_Pass::topic_cb( cwork3::sinusoid in) {

	_in = in.v;
	
}

void Low_Pass::filter(){
	int count = 0;

	while(ros::ok()){

		//filter
		_out.out = _LPF_Beta*_out.out +(1-_LPF_Beta)*_in;

		ROS_INFO("sin: %.3f\t time: %.2f", _out.out, count*0.01);
		_topic_pub.publish(_out);
		count++;
		usleep(10000);

	}
}

int main( int argc, char** argv ) {

	//Init the ros node with lowpass_topic name
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

