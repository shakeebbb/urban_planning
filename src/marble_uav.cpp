
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "marble_uav_msgs/GlobalComm.h"

enum class STATUS { TOPIC_UNRELIABLE, TOPIC_TIMEOUT, TOPIC_OK, PROCESS_DOWN, PROCESS_UP, PROCESS_UNRELIABLE, PREREQ_UNAVAILABLE, PREREQ_OK };

// **************************************************************
template <class T> 
class topic_class
{
private:
  bool alive_;
  double duration_; // listen duration
  double rate_; // current topic rate
  double minRateExp_; // minimum expected rate
  double maxRateExp_; // maximum expected rate

  std::string topicName_;
  ros::NodeHandle* nh_;

  ros::Subscriber topicSub_;

public: 
  topic_class(ros::NodeHandle* nh, std::string topicName, double duration, double expRate, double rateTol)
  {
    queryTimeOut_ = queryTimeOut;
    duration_ = duration;
    topicName_ = topicName;
    minRateExp_ = expRate - rateTol;
    maxRateExp_ = expRate + rateTol;
    nh_ = nh;

    update();
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

  double rate(double timeOut)
  {
    ros::Rate r(10); // 10 hz
    static double tic = ros::Time::now().toSec();

    while (ros::ok() && (ros::Time::now().toSec() - tic) < timeOut)
    {
      if(rate_ > 0.0)
        break;
      r.sleep();
    }
    return rate_;
  }

  STATUS get_status(double timeOut)
  {
    double rate = rate(timeOut);
    if (rate > minRateExp_ && rate < maxRateExp_)
      return STATUS::TOPIC_OK;
    if (rate < minRateExp_ && rate_ > 0.0)
      return STATUS::TOPIC_UNRELIABLE;
    if (rate > maxRateExp_ && rate_ > 0.0)
      return STATUS::TOPIC_UNRELIABLE;
    
    return STATUS::TOPIC_TIMEOUT;
  }

  bool alive()
  {
   return alive_;
  }

  void update()
  {
    rate_ = 0.0;
    alive_ = false;
    topicSub_ = nh->subscribe(topicName_, 1, &topic_class::topic_cb, this);
  }
};

// **************************************************************
class process_class
{
private:
  marble_uav_msgs::GlobalComm::process_id processId_;
  std::string launchCommand_;
  std::vector<topic_class> topics_;
  std::vector<process_class> parents_;
  double timeOut_; // should be greater than the listen time of the topics
  double rateTol_;
public:
  process_class(marble_uav_msgs::GlobalComm::process_id processId, std::vector<topic_class>& topics, std::string launchCommand, double timeOut, double rateTol, std::vector<process_class>& parents)
  {
    topics_ = topics;
    launchCommand_ = launchCommand;
    processId_ = processId;
    timeOut_ = timeOut;
    rateTol_ = rateTol;
    parents_ parents;
  }

  STATUS get_parent_status
  {
    for (int i=0;i<parents_.size();i++)
    {
      STATUS processStatus = parents_[i].get_status();
   
      if (processStatus != STATUS::PROCESS_UP)
        return STATUS::PREREQ_UNAVAILABLE;
    }
    return STATUS::PREREQ_OK;
  }

  STATUS get_status()
  {
    for (int i=0;i<topics_.size();i++)
    {
      int nOkTopics = 0;
      int nUnreliableTopics = 0;
      STATUS topicStatus = topics_[i].get_status(timeOut_);
      
      if(topicStatus = STATUS::TOPIC_OK())
       nOkTopics++;
      if(topicStatus = STATUS::TOPIC_UNRELIABLE())
       nUnreliableTopics++;
    }
    
    if (nOkTopics == topics_.size())
      return STATUS::PROCESS_UP;
    if (nOkTopics > 0 && nOkTopics < topics_.size())
      return STATUS::PROCESS_UNRELIABLE;
    if (nUnreliableTopics > 0)
      return STATUS::PROCESS_UNRELIABLE;
    
    return STATUS::PROCESS_DOWN;
  }

  refresh_status()
  {
    for (int i=0;i<topics_.size();i++)
      topics_[i].update();
  }
  
  marble_uav_msgs::GlobalComm::process_id get_id()
  {
    return processId_;
  }
};


// **************************************************************
class marble_uav_class
{
private:

double rateTol_;

ros::ServiceServer globalCommSrvr_;

topic_class<geometry_msgs::PoseStamped>* mavrosTopic_;

public:

uav_master_class(ros::NodeHandle* nh)
{
  nh->getParam("topic_rate_tolerance", rateTol_);
  ROS_INFO("%s: Parameters retrieved from parameter server", nh->getNamespace().c_str());

  globalCommSrvr_ = nh->advertiseService("global_comm_service", &marble_uav_class::global_comm_cb, this);

  mavrosTopic_ = new topic_class<geometry_msgs::PoseStamped>(nh, "topic_in", 3);
}

void loop()
{
    ROS_INFO_THROTTLE(1, "%f", mavrosTopic_->rate());
}

bool global_comm_cb(marble_uav_msgs::GlobalComm::Request&, marble_uav_msgs::GlobalComm::Response&)
{
  
}

bool check_process(marble_uav_msgs::GlobalComm::process_id processId)
{
  switch()
  {
   case marble_uav_msgs::GlobalComm::Sensors:
     
   break;
   default:
  }
}

bool check_parents(marble_uav_msgs::GlobalComm::process_id processId)
{
 
}

};

// **************************************************************
int main(int argc, char **argv)
{
  ros::init(argc, argv, "marble_uav");
  ros::NodeHandle nh("marble_uav");

  marble_uav_class obj(&nh);

  while(ros::ok())
  {
   obj.loop();
   ros::spinOnce();
  }
}



// **************************************************************
