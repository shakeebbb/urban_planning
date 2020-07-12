
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"

double nRecvdMsgs_;
double timeElapsed_;

ros::Subscriber topicSub_;

// Function Declarations
template<typename T> void topic_cb(const T&);
template<typename> void topic_cb(const geometry_msgs::PoseStamped&);

// Main
int main(int argc, char **argv)
{
	ros::init(argc, argv, "topic_check");
	ros::NodeHandle nh("topic_check");
	
	topicSub_ = nh.subscribe("topic_in", 10, topic_cb);

  double topicRateTol;
  double topicRateExp;
  double topicDur;

  nh.getParam("topic_rate_tolerance", topicRateTol);
  nh.getParam("expected_topic_rate", topicRateExp);
  nh.getParam("listen_duration_secs", topicDur);

  nRecvdMsgs_ = 0;
	
  while(ros::ok() && (timeElapsed_ <= topicDur) )
	 ros::spinOnce();

  double topicRate = nRecvdMsgs_ / timeElapsed_;
  
  if(topicRate >= (topicRateExp+topicRateTol) )
	 return 1;
  if(topicRate <= (topicRateExp-topicRateTol) )
	 return 1;

  return 0;
}

// ************************************************************
template<typename T>
void topic_cb(const T& msg)
{
	static double tic = ros::Time::now().toSec();

  nRecvdMsgs_ ++; 
  
  timeElapsed_ = ros::Time::now().toSec() - tic;
}

