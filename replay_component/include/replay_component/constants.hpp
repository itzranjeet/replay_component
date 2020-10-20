#ifndef REPLAY_COMPONENT_INCLUDE_CONSTANTS_HPP
#define REPLAY_COMPONENT_INCLUDE_CONSTANTS_HPP

#include <iostream>

std::string default_path = "/home/jesmij/ros_ws/src/replay_component/data/unzip_data/";
std::string unzip = "unzip ";
std::string zippath = "\"/home/jesmij/ros_ws/src/replay_component/data/bench_input_output/bench_input/*.zip\" ";
std::string unzipdata = "-d /home/jesmij/ros_ws/src/replay_component/data/unzip_data/";

std::string jq_cmd = "jq .video_list[].scenario_uuid /home/jesmij/ros_ws/src/replay_component/data/bench_input_output/bench_input/configuration.json > /home/jesmij/ros_ws/src/replay_component/config/input.csv";
std::string headers = "Sequence_Number,Frame_Number,Timestamp,Obj_X1,Obj_Y1,Obj_X2,Obj_Y2,Obj_Confidence,Obj_Classification,Input_Time\n";

static constexpr char N1_ImagePublisherMessageTopic[]{"N1_test_image"};
static constexpr char N1_DataOverMessageTopic[]{"N1_Data_over_Msg_Topic"};
static constexpr char N1_FileNameMessageTopic[]{"N1_File_Name_Msg_Topic"};
static constexpr char N5_TriggerStatusMessageTopic[]{"N5_Trigger"};

static constexpr unsigned N1_ImagePublisherMessageQueueSize{10};
static constexpr unsigned N1_DataOverMessageQueueSize{100};
static constexpr unsigned N1_FileNameMessageQueueSize{10};
static constexpr unsigned N5_TriggerStatusMessageQueueSize{10};

#endif // REPLAY_COMPONENT_INCLUDE_CONSTANTS_HPP