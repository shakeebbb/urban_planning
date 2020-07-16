
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

// **************************************************************
template <class T> 
class topic_class
{
private:
  bool alive_;
  double duration_;
  double rate_;

  ros::Subscriber topicSub_;

public: 
  topic_class(ros::NodeHandle* nh, std::string topic_name, double duration)
  {
    duration_ = duration;
    rate_ = -1.0;
    alive_ = false;

    topicSub_ = nh->subscribe(topic_name, 1, &topic_class::topic_cb, this);
  }

  void topic_cb(const T& msg)
  {
    const static double tic = ros::Time::now().toSec();
    static double nRecvdMsgs = -1;
    alive_ = true;
    nRecvdMsgs ++; 
  
    double timeElapsed = ros::Time::now().toSec() - tic;
    if ( timeElapsed > duration_)
    {
		  rate_ = nRecvdMsgs / timeElapsed;
		  topicSub_.shutdown();
    }
    std::cout << "Time Elapsed= " << timeElapsed << ", Messages= " << nRecvdMsgs << std::endl;
  }

  double rate()
  {
    return rate_;
  }

  bool alive()
  {
   return alive_;
  }
};

// **************************************************************
class uav_master_class
{
private:

double rateTol_;

ros::Publisher resPub_;
ros::Subscriber reqSub_;

topic_class<geometry_msgs::PoseStamped>* mavrosTopic_;

public:

uav_master_class(ros::NodeHandle* nh)
{
  nh->getParam("topic_rate_tolerance", rateTol_);
  ROS_INFO("%s: Parameters retrieved from parameter server", nh->getNamespace().c_str());

  resPub_ = nh->advertise<std_msgs::String>("response_topic", 10);
  reqSub_ = nh->subscribe("request_topic", 10, &uav_master_class::req_cb, this);

  mavrosTopic_ = new topic_class<geometry_msgs::PoseStamped>(nh, "topic_in", 3);
}

void loop()
{
    ROS_INFO_THROTTLE(1, "%f", mavrosTopic_->rate());
}

void req_cb(const std_msgs::String& reqMsg)
{
  
}

};

// **************************************************************
int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_master");
  ros::NodeHandle nh("uav_master");

  uav_master_class obj(&nh);

  while(ros::ok())
  {
   obj.loop();
   ros::spinOnce();
  }
}



// **************************************************************
