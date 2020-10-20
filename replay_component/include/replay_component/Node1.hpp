#ifndef REPLAY_COMPONENT_INCLUDE_NODE1_HPP
#define REPLAY_COMPONENT_INCLUDE_NODE1_HPP

#include "replay_component/constants.hpp"

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dirent.h>
#include <image_transport/image_transport.h>
#include <string.h>
#include <unistd.h>
#include "std_msgs/String.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>

class ImageVisualizer
{
public:
  DIR *ImageDIR;
  DIR *DataDIR;
  struct dirent *entry;
  volatile int flag = 0;
  volatile int count = 1;
  std::vector<std::string> ImageFiles;
  std::vector<std::string> FolderFiles;
  std::vector<std::string> CsvFilename;
  std::string imgpath;
  std::string folder;
  std::string command = unzip + zippath + unzipdata;
  std::string filename;
  
  void getImageDirectory(std::string folder);
  void ImagePublisher(std::string folder);
  void Run(int argc, char **argv);
  void chatterCallback(const std_msgs::String::ConstPtr &status);
  void RegisterPublishers();
  void RegisterSubscribers();
  void messageDeliver(std::string Message_);
  void csv_reader(int argc, char **argv);
  void publishFileName(std::string filename);

private:
  ros::NodeHandle nh;
  ros::Subscriber trigger_sub;
  image_transport::Publisher image_pub;
  sensor_msgs::ImagePtr ros_image;
  cv::Mat image;
  std_msgs::String Data_Over;
  std_msgs::String File_Name;
  ros::Publisher Data_Over_;
  ros::Publisher File_Name_;
};
#endif // REPLAY_COMPONENT_INCLUDE_NODE1_HPP
