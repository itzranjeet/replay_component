#include "replay_component/constants.hpp"

#include "ros/ros.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "replay_component/ADAS_Support.h"
#include <pthread.h>

#include <iostream>

using namespace cv;

static int stop_transfers = 0;
static unsigned int u32_ImageWidth = 1280;
static unsigned int u32_ImageHeight = 960;

void Stop_Handler(int s)
{
  stop_transfers = 1;
}

int fd;

class Node3
{
public:
  void Run();
  void RegisterSubscribers();
  void ImageData_Callback(const sensor_msgs::ImageConstPtr &msg);
  void Init_FIFO();
  void Close_FIFO();
  char *writer;
  const char *myfifo = "/tmp/myfifo";
  unsigned int u32ImageSize;
  unsigned int FrameSize = u32_ImageWidth * u32_ImageHeight * 2;
  unsigned char *UYVYData;
  unsigned char *final_data = NULL;

private:
  ros::NodeHandle nh;
  ros::Subscriber ImageData_sub;
  cv_bridge::CvImagePtr cv_ptr;
};

void Node3::ImageData_Callback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //cv_image
    Mat cv_ptr_ = cv_ptr->image;                                           // mat
    resize(cv_ptr_, cv_ptr_, Size(u32_ImageWidth, u32_ImageHeight));       //ROI
    std::cout << "cv_ptr_"<< ' '<< cv_ptr_.cols << 'x'<<cv_ptr_.rows << std::endl;
    final_data = cv_ptr_.data; // conversion to uchar
    std::cout << "I am in Image callback" << std::endl;
    RGB888_BGRtoYUV422_UYVY(final_data, UYVYData, u32_ImageWidth * u32_ImageHeight); // BGR to yuv
    write(fd, UYVYData, u32_ImageWidth * u32_ImageHeight * 2);                       //write to pipe
    usleep(5000);                                                                    //delay
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void Node3::RegisterSubscribers()
{
  ImageData_sub = nh.subscribe(N1_ImagePublisherMessageTopic, 1, &Node3::ImageData_Callback, this);
  std::cout << "I am in RegisterSubscribers " << std::endl;
  ros::Rate loop_rate(5);
  loop_rate.sleep();
  ros::spin();
}

void Node3::Init_FIFO()
{
  mkfifo(myfifo, 0666);                     //mkdir
  //fd = open(myfifo, O_WRONLY | O_NONBLOCK); //open pipe in write mode
  fd = open(myfifo, O_WRONLY);
  UYVYData = (unsigned char *)malloc(FrameSize);
  std::cout << "I am in Init Fifo" << std::endl;
}

void Node3::Close_FIFO()
{
  free(UYVYData); //free memory
  close(fd);      //close pipe
}

void Node3::Run()
{
  Init_FIFO();           //init
  RegisterSubscribers(); //callback
  Close_FIFO();          //close
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Node3");
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = Stop_Handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  Node3 object;
  object.Run();
  return 0;
}
