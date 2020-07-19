
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Imu.h"
#include "marble_uav_msgs/GlobalComm.h"

enum class STATUS {TOPIC_UNRELIABLE, TOPIC_TIMEOUT, TOPIC_OK}; // Topic status

// **************************************************************
template <class T> 
class topic_class
{
private:
  bool alive_;
  double duration_; // listen duration
  double timeOut_;
  double rate_; // current topic rate
  double minRateExp_; // minimum expected rate
  double maxRateExp_; // maximum expected rate

  double firstMsgStamp_;
  double nRecvdMsgs_;

  std::string topicName_;
  ros::NodeHandle* nh_;

  ros::Subscriber topicSub_;

public: 
  topic_class(ros::NodeHandle* nh, std::string topicName, double duration, double timeOut, double expRate, double rateTol)
  {
    duration_ = duration;
    timeOut_ = timeOut;
    topicName_ = topicName;
    minRateExp_ = expRate - rateTol;
    maxRateExp_ = expRate + rateTol;
    nh_ = nh;

    update_status();
  }

  void topic_cb(const T& msg)
  {
    if(!alive_)
    {
     firstMsgStamp_ = ros::Time::now().toSec();
     nRecvdMsgs_ = -1;
    }

    alive_ = true;
    nRecvdMsgs_ ++; 
  
    double timeElapsed = ros::Time::now().toSec() - firstMsgStamp_;
    if ( timeElapsed > duration_)
    {
		  rate_ = nRecvdMsgs_ / timeElapsed;
		  topicSub_.shutdown();
    }
    //std::cout << "Time Elapsed= " << timeElapsed << ", Messages= " << nRecvdMsgs_ << std::endl;
  }

  double topic_rate()
  {
    if(rate_ == 0.0)
     sleep(timeOut_);
    return rate_;
  }

  STATUS get_status()
  {
    double rate = topic_rate();
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

  void update_status()
  {
    rate_ = 0.0;
    alive_ = false;
    topicSub_ = nh_->subscribe(topicName_, 1, &topic_class::topic_cb, this);
  }
};


// **************************************************************
class marble_uav_class
{
private:

ros::ServiceServer globalCommSrvr_;
topic_class<geometry_msgs::Pose>* cartPose_;
topic_class<geometry_msgs::Pose>* mavrosPose_;
topic_class<sensor_msgs::Imu>* imuMsg_;
topic_class<geometry_msgs::Vector3Stamped>* foreRepVel_;
topic_class<geometry_msgs::Vector3Stamped>* upRepVel_;
topic_class<geometry_msgs::Vector3Stamped>* downRepVel_;
topic_class<geometry_msgs::TwistStamped>* cmdVel_;

/* *****
topic_class<ROS_TYPE>* TOPIC_OBJ_;
***** */

public:

marble_uav_class(ros::NodeHandle* nh)
{
  std::cout << "Constructor called" << std::endl;

  double topicRateTol; double topicListenDur; double topicTimeout;
  while(!nh->getParam("topic_listen_duration", topicListenDur));
  while(!nh->getParam("topic_time_out", topicTimeout));
  while(!nh->getParam("topic_rate_tolerance_hz", topicRateTol));

  std::cout << "Double params" << std::endl;
  std::vector<std::string> topicParam;

  while(!nh->getParam("cart_pose_topic", topicParam));
  cartPose_ = new topic_class<geometry_msgs::Pose>(nh, topicParam[1], topicListenDur, topicTimeout, stod(topicParam[0]), topicRateTol);

  while(!nh->getParam("mavros_pose_topic", topicParam));
  mavrosPose_ = new topic_class<geometry_msgs::Pose>(nh, topicParam[1], topicListenDur, topicTimeout, stod(topicParam[0]), topicRateTol);
  
  while(!nh->getParam("imu_topic", topicParam));
  imuMsg_ = new topic_class<sensor_msgs::Imu>(nh, topicParam[1], topicListenDur, topicTimeout, stod(topicParam[0]), topicRateTol);

  while(!nh->getParam("fore_rep_vel_topic", topicParam));
  foreRepVel_ = new topic_class<geometry_msgs::Vector3Stamped>(nh, topicParam[1], topicListenDur, topicTimeout, stod(topicParam[0]), topicRateTol);

  while(!nh->getParam("up_rep_vel_topic", topicParam));
  upRepVel_ = new topic_class<geometry_msgs::Vector3Stamped>(nh, topicParam[1], topicListenDur, topicTimeout, stod(topicParam[0]), topicRateTol);
  
  while(!nh->getParam("down_rep_vel_topic", topicParam));
  downRepVel_ = new topic_class<geometry_msgs::Vector3Stamped>(nh, topicParam[1], topicListenDur, topicTimeout, stod(topicParam[0]), topicRateTol);

  while(!nh->getParam("command_velocity_topic", topicParam));
  cmdVel_ = new topic_class<geometry_msgs::TwistStamped>(nh, topicParam[1], topicListenDur, topicTimeout, stod(topicParam[0]), topicRateTol);

/* *****
while(!nh->getParam("TOPIC", topicParam));
TOPIC_CLASS_OBJ = new topic_class<ROS_TYPE>(nh, topicParam[1], topicListenDur, topicTimeout, stod(topicParam[0]), topicRateTol);
***** */

  ROS_INFO("%s, Parameters retreived from the parameter server", nh->getNamespace().c_str());

  globalCommSrvr_ = nh->advertiseService("global_comm_service", &marble_uav_class::global_comm_cb, this);
}

bool global_comm_cb(marble_uav_msgs::GlobalComm::Request& req, marble_uav_msgs::GlobalComm::Response& res)
{
  if(req.action == marble_uav_msgs::GlobalComm::Request::GET_STATUS)
  {
    std::string message;
    res.status = get_process_status(req.process_id, message);
    res.message = message;
    return true;
  }
  if(req.action == marble_uav_msgs::GlobalComm::Request::LAUNCH)
  {
    std::string message;
    launch_process(req.process_id, message);
    return true;
  }
}

void launch_process(uint32_t processId, std::string& msg)
{
  msg = "Launching process";
  if(processId == marble_uav_msgs::GlobalComm::Request::SYSTEM)
   system("/home/shakeeb/dev_ws/src/marble_uav_pkgs/scripts/launch_system.sh");
}

uint32_t get_process_status(uint32_t processId, std::string& errorMsg)
{
  if(processId == marble_uav_msgs::GlobalComm::Request::SYSTEM)
    return get_system_status(errorMsg);
}

uint32_t get_system_status(std::string& errorMsg)
{
  int badTopics = 0;

  cartPose_->update_status();
  mavrosPose_->update_status();
  imuMsg_->update_status();
  foreRepVel_ -> update_status();
  upRepVel_ -> update_status();
  downRepVel_ -> update_status();
  cmdVel_ -> update_status();

/* *****
topic_class_obj -> update_status();
***** */  

  if(cartPose_->get_status() != STATUS::TOPIC_OK)
  {
   badTopics ++;
   errorMsg += "SYSTEM: Cartographer pose not available \n";
  }
  if(mavrosPose_->get_status() != STATUS::TOPIC_OK)
  {
   badTopics ++;
   errorMsg += "SYSTEM: Mavros pose not available \n";
  }
  if(imuMsg_->get_status() != STATUS::TOPIC_OK)
  {
   badTopics ++;
   errorMsg += "SYSTEM: Imu topic not available \n";
  }
  if(foreRepVel_->get_status() != STATUS::TOPIC_OK)
  {
   badTopics ++;
   errorMsg += "SYSTEM: Fore repulsive velocity not available \n";
  }
  if(upRepVel_->get_status() != STATUS::TOPIC_OK)
  {
   badTopics ++;
   errorMsg += "SYSTEM: Up repulsive velocity not available \n";
  }
  if(downRepVel_->get_status() != STATUS::TOPIC_OK)
  {
   badTopics ++;
   errorMsg += "SYSTEM: Down repulsive velocity not available \n";
  }
  if(cmdVel_->get_status() != STATUS::TOPIC_OK)
  {
   badTopics ++;
   errorMsg += "SYSTEM: Command velocity not available \n";
  }

/* *****
if(TOPIC_CLASS_OBJ_->get_status() != STATUS::TOPIC_OK)
  {
   badTopics ++;
   errorMsg += "SYSTEM: TOPIC not available \n";
  }
***** */

  if(badTopics > 0)
   return marble_uav_msgs::GlobalComm::Response::DOWN; 
  else
   return marble_uav_msgs::GlobalComm::Response::UP;
}

void loop()
{
  std::string msg;

  static double tic = ros::Time::now().toSec();

  if((ros::Time::now().toSec() - tic) > 1)
  {
   std::cout << cartPose_->topic_rate() << std::endl;
   tic = ros::Time::now().toSec();
  }


  //ROS_INFO_THROTTLE(2,"%i", get_system_status(msg));
  //ROS_INFO_THROTTLE(5,"%i", cartPose_->topic_rate());
}

};

// **************************************************************
int main(int argc, char **argv)
{
  ros::init(argc, argv, "marble_uav");
  ros::NodeHandle nh(ros::this_node::getName());

  marble_uav_class obj(&nh);

  ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown(); 

}
// **************************************************************
