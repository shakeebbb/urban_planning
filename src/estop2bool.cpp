
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "estop_msgs/EStopStatus.h"

using namespace std;

ros::Publisher boolPub;
ros::Subscriber estopSub;

// Function Declarations
void estop_cb(const estop_msgs::EStopStatus&);

// Main
int main(int argc, char **argv)
{
	ros::init(argc, argv, "estop2bool");
	ros::NodeHandle nh;
	
	boolPub = nh.advertise<std_msgs::Bool>("bool_topic", 10);
	
	estopSub = nh.subscribe("estop_topic", 10, estop_cb);
	
	ros::spin();
	return 0;
}

// ************************************************************
void estop_cb(const estop_msgs::EStopStatus& msg)
{
	std_msgs::Bool boolMsg;
	boolMsg.data = msg.radio_state;
	boolPub.publish(boolMsg);
}

