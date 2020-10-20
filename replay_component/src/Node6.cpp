#include "replay_component/Node6.hpp"

void StatusNode::RegisterSubscribers()
{
  sub = nh.subscribe(N1_DataOverMessageTopic, N1_DataOverMessageQueueSize, &StatusNode::statusCallback, this);
}

void StatusNode::statusCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO("Node6: [%s]", msg->data.c_str());
  buffer = msg->data.c_str();
  if (buffer == "N1_Data_Over")
  {
    dataOver_cmd();
  }
  if (buffer == "N1_Data_got_Interrupted")
  {
    dataInterrupted_cmd();
  }
  if (buffer == "N1_Processing")
  {
    processingStatus_cmd();
  }
  if (buffer == "N1_Idle")
  {
    idleStatus_cmd();
  }
}

void StatusNode::idleStatus_cmd()
{
  std::string command = "curl -H \"Content-Type: application/json\" -X GET -d '{\"data\":1234567}' localhost:8002/status_idle";
  system(command.c_str());
}

void StatusNode::processingStatus_cmd()
{

  std::string command = "curl -H \"Content-Type: application/json\" -X GET -d '{\"data\":1234567}' localhost:8002/status_processing";
  system(command.c_str());
}

void StatusNode::dataInterrupted_cmd()
{

  std::string command = "curl -H \"Content-Type: application/json\" -X GET -d '{\"data\":1234567}' localhost:8002/data_interrupted";
  system(command.c_str());
}

void StatusNode::dataOver_cmd()
{
  std::string command = "curl -H \"Content-Type: application/json\" -X GET -d '{\"data\":1234567}' localhost:8002/data_over";
  system(command.c_str());
}

void StatusNode::Run()
{
  RegisterSubscribers();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Node6");
  StatusNode object;
  object.Run();
  ros::spin();
  return 0;
}