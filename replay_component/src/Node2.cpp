#include "replay_component/constants.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

//Call this function only once at the begining.
char **CreateOutBuffer(int nsize, int bytesize)
{
  char **OutBuffer;
  OutBuffer = static_cast<char **>(calloc(nsize, sizeof(char **)));
  //OutBuffer = new char *[nsize];
  for (int i = 0; i < nsize; i++)
    // {
    // OutBuffer[i] = new char[bytesize];
    OutBuffer[i] = static_cast<char *>(calloc(bytesize, sizeof(char *)));

  // std::cout << "nsize is: " << (int)OutBuffer[nsize] << std::endl;
  //}
  return OutBuffer;
}

//Call this function only once at the End.
void deleteOutBuffer(int nsize, char **OutBuffer)
{
  for (int i = 0; i < nsize; i++)
    free(OutBuffer[nsize]);
  free(OutBuffer);
}
class Node2
{
public:
  void RegisterSubscribers();
  void csvfilenameCallback(const std_msgs::String::ConstPtr &msg);
  void closecsvfileCallback(const std_msgs::String::ConstPtr &msg);
  int getlines(char *InBuffer, char **OutBuffer, char *OutBufferTail);
  void WriteToFile();
  void Run();
  std::string prev_Frame_Number;
  std::string N1_data_over = "Nothing";
  int n;
  int i;
  int nsize = 8;
  int device;
  volatile int sequence_number = 0;
  ros::Time Input_Time;
  std::ofstream csv_file;

private:
  ros::NodeHandle nh;
  ros::Subscriber csvfilename_sub;
  ros::Subscriber closecsvfile_sub;
};

void Node2::RegisterSubscribers()
{
  csvfilename_sub = nh.subscribe(N1_FileNameMessageTopic, N1_FileNameMessageQueueSize, &Node2::csvfilenameCallback, this);
  closecsvfile_sub = nh.subscribe(N1_DataOverMessageTopic, N1_DataOverMessageQueueSize, &Node2::closecsvfileCallback, this);
}

void Node2::csvfilenameCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO("Node2: [%s]", msg->data.c_str());

  std::string path = msg->data.c_str();
  csv_file.open("/home/jesmij/ros_ws/src/replay_component/data/bench_input_output/bench_output/Reprocessed_Output/" + path, ios::out);
  csv_file << headers;
  std::cout << "I am in headers section" << std::endl;
  WriteToFile();
}

void Node2::closecsvfileCallback(const std_msgs::String::ConstPtr &msg1)
{

  ROS_INFO("I heard: [%s]", msg1->data.c_str());
  std::string status = msg1->data.c_str();

  if (status == "N1_Data_Over")
  {
    N1_data_over = status;
    csv_file.close();
  }
  if (status == "N1_Data_got_Interrupted")
  {
    std::cout << status << std::endl;
  }
}

//Call this function after every read from serial port. E.g. func below.
int Node2::getlines(char *InBuffer, char **OutBuffer, char *OutBufferTail)
{
  char *InPointer = InBuffer;
  char *OutPointer = NULL;
  char *InTailPointer = OutBufferTail;
  char *OutTailPointer = OutBufferTail;

  int NoOfLines = 0;
  bool NullFound = false;
  //std::cout << "Tail" << OutBufferTail << std::endl;
  OutPointer = OutBuffer[NoOfLines];
  if (OutBufferTail[0] != '\0')
  {
    OutBuffer[NoOfLines] =
        OutPointer = OutBuffer[NoOfLines];
    do
    {
      //std::cout << *InTailPointer << std::flush;
      *OutPointer = *InTailPointer;
      OutPointer++;
      InTailPointer++;

    } while (*InTailPointer != '\0');
    *OutPointer = '\0';
  }
  //std::cout<<"I am here"<<std::endl;
  do
  {
    // std::cout << "Lines " << NoOfLines << std::endl;

    //std::cout << "OutPointer " << OutPointer << std::endl;
    OutTailPointer = OutBufferTail;
    while ((*InPointer != '\0') && (*InPointer != '\n'))
    {
      //std::cout << *InPointer << std::flush;
      *OutPointer = *InPointer;

      *OutTailPointer = *InPointer;
      OutPointer++;
      InPointer++;
      OutTailPointer++;
    }
    //std::cout << "I am here 11" << std::endl;
    if (*InPointer == '\0')
    {
      //std::cout << "I am here 111" << std::endl;
      *OutTailPointer = *InPointer;
      OutTailPointer++;

      *OutTailPointer = '\0';
      break;
    }
    //std::cout << "I am here 222" << std::endl;
    *OutPointer = *InPointer;
    //std::cout << "I am here 22" << std::endl;
    OutPointer++;
    *OutPointer = '\0';
    InPointer++;
    //std::cout << "I am here 33" << std::endl;
    std::string str(OutBuffer[NoOfLines]);
    //std::cout << "OutBuffer" << str << std::endl;
    NoOfLines++;
    OutPointer = OutBuffer[NoOfLines];
  } while (1);

  return NoOfLines;
}
void Node2::WriteToFile()
{

  char **OutBuffer = CreateOutBuffer(nsize, 512);
  char *BufferTail;
  BufferTail = new char[1024];
  int NumOfLines = 0;
  char Buffer[1024];
  struct termios settings;
  speed_t baud = 115200; // baud rate
  std::string port = "/dev/ttyUSB0";
  cfsetospeed(&settings, baud); //baud rate
  std::cout << "Port:" << port << "\nBaud rate: " << baud << std::endl;
  device = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (device == 1)
    printf("\n  Error! in Opening ttyUSB0\n");
  else
    printf("\n  ttyUSB0 Opened Successfully\n");
  Buffer[0] = '\0';
  do
  {
    Input_Time = ros::Time::now();
    //Buffer[0] = '\0';
    //std::cout << "I am in the do loop" << std::endl;
    n = read(device, Buffer, sizeof Buffer);
    //std::cout << "The value of n is" << n << std::endl;
    //TO DO: get ROS time
    ros::Time timestamp = ros::Time::now();
    Buffer[n] = '\0';
    if (n > 0)
    {

      NumOfLines = getlines(Buffer, OutBuffer, BufferTail);
      //std::cout << "The value of NumOfLines is: " << NumOfLines << std::endl;
      for (i = 0; i < NumOfLines; i++)
      {
        // std::cout << "I am inside for Loop" << std::endl;
        std::string str2("PDDAY"); // This is the selection criteria
        std::string str(OutBuffer[i]);
        //std::cout << "The Buffer value is   " << Buffer << std::endl;
        std::size_t found = str.find(str2); // Find the string in the CSV Row
                                            // std::cout << __LINE__ << std::endl;
        if (found != string::npos)
        { /*
          cout << "***";
          // push ROS time first and then OutBuffer data
          csv_file << begin << "," << OutBuffer[i] << std::flush;*/
          //std::cout << "The Buffer value is:  " << str << std::endl;

          // cout << "***";
          //std::string str = "[HOST ] 308.713897 s: ***PDDAY Output:,203,1,316,208,344,260,0.47,35.53,-1.40,0.00";
          std::vector<std::string> v;
          std::stringstream ss(str);
          while (ss.good())
          {
            std::string substr;
            std::getline(ss, substr, ',');
            v.push_back(substr);
          }
          /* for (size_t i = 0; i < v.size(); i++)
          std::cout << v[i] << std::endl;*/
          //std::string Sequence_Number = 0;
          std::string Frame_Number = v[1];
          std::string Obj_X1 = v[3];
          std::string Obj_Y1 = v[4];
          std::string Obj_X2 = v[5];
          std::string Obj_Y2 = v[6];
          srand((unsigned)time(0));
          int randomNumber;
          int max = 99;
          int min = 65;
          int range = max - min + 1;
          //int randomNumber = rand() % (99-65+1)) + 65;
          randomNumber = rand() % range + min;
          std::string Obj_Confidence = std::to_string(randomNumber);
          std::string Obj_Classification = "Pedestrian";
          //ros::Time offset = 2.0;
          //ros::Time Input_Time = timestamp;
          // std::cout << "Before writing Csv file" << std::endl;
          if ((Frame_Number.compare(prev_Frame_Number)) != 0)
          {
            sequence_number += 1;
          }
          std::string sequence_number_ = std::to_string(sequence_number);
          csv_file << sequence_number_ << "," << Frame_Number << "," << timestamp << "," << Obj_X1 << "," << Obj_Y1 << "," << Obj_X2 << "," << Obj_Y2 << "," << Obj_Confidence << "," << Obj_Classification << "," << Input_Time << std::endl;
          prev_Frame_Number = Frame_Number;
          //std::cout << "After writing Csv file" << std::endl;
          //sequence_number += 1;
          v.clear();
        }
      }
    }
    ros::spinOnce();
  } while (N1_data_over != "N1_Data_Over");
  sequence_number = 0;
  N1_data_over = "Nothing";
  close(device);
  deleteOutBuffer(nsize, OutBuffer);
}

void Node2::Run()
{
  system(jq_cmd.c_str());
  RegisterSubscribers();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Node2");
  Node2 object;
  object.Run();
  ros::spin();
  return 0;
}
