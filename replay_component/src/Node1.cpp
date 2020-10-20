#include "replay_component/Node1.hpp"

void ImageVisualizer::RegisterPublishers()
{
  image_transport::ImageTransport it_(nh);
  image_pub = it_.advertise(N1_ImagePublisherMessageTopic, N1_ImagePublisherMessageQueueSize);
  Data_Over_ = nh.advertise<std_msgs::String>(N1_DataOverMessageTopic, N1_DataOverMessageQueueSize);
  File_Name_ = nh.advertise<std_msgs::String>(N1_FileNameMessageTopic, N1_FileNameMessageQueueSize);
}

void ImageVisualizer::RegisterSubscribers()
{
  system(command.c_str()); //unzipping
  trigger_sub = nh.subscribe(N5_TriggerStatusMessageTopic, N5_TriggerStatusMessageQueueSize, &ImageVisualizer::chatterCallback, this);
  std::cout << "I am in RegisterSubscribers " << std::endl;
}

void ImageVisualizer::getImageDirectory(std::string folder)
{
  ImageFiles.clear();
  if (ImageDIR = opendir(folder.c_str()))
  {
    while (entry = readdir(ImageDIR))
    {
      if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
        ImageFiles.push_back(entry->d_name);
    }
  }
  closedir(ImageDIR);
  std::sort(ImageFiles.begin(), ImageFiles.end());
}

void ImageVisualizer::ImagePublisher(std::string folder) //attribute folder name
{
  getImageDirectory(folder); // attribute folder

  for (int i = 0; (i < ImageFiles.size()) && (flag == 1); i++)
  {
    imgpath = folder + ImageFiles[i];
    image = cv::imread(imgpath, CV_LOAD_IMAGE_COLOR);
    ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    ros::Rate loop_rate(5);
    image_pub.publish(ros_image);
    std::cout << "Publishing->>>>" << ImageFiles[i] << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  count += 1;
}
void ImageVisualizer::publishFileName(std::string filename)
{
  std::string Message = filename;
  File_Name.data = Message;
  File_Name_.publish(File_Name);
  std::cout << "File_Name Published: " << File_Name << std::endl;
  ROS_INFO("Node1: [%s]", File_Name.data.c_str());
}

void ImageVisualizer::messageDeliver(std::string Message_)
{
  std::string Message = Message_;
  Data_Over.data = Message;
  Data_Over_.publish(Data_Over);
  ROS_INFO("Node1: [%s]", Data_Over.data.c_str());
}

void ImageVisualizer::chatterCallback(const std_msgs::String::ConstPtr &status)
{
  ROS_INFO("Node1: [%s]", status->data.c_str());

  if (status->data == "start_bench")
  {
    std::cout << "start_bench Trigger pressed" << std::endl;

    flag = 1;
    for (int i = 0; (i < FolderFiles.size()); i++)
    {
      std::string csv_filename = CsvFilename[i];
      std::string sample_keyword = "bench_";
      std::string extension = ".csv";
      publishFileName(sample_keyword + csv_filename + extension);
      std::string folder = FolderFiles[i];
      ImagePublisher(folder);

      if (flag == 1)
      {
        messageDeliver("N1_Data_Over");
        usleep(5000000);
      }
      if (flag == 0)
      {
        messageDeliver("N1_Data_got_Interrupted");
      }
    }
  }

  if (status->data == "checking_status" && flag != 0)
  {
    std::cout << '\n'
              << "Status: The data is Processing" << std::endl;
    messageDeliver("N1_Processing");
  }

  if (status->data == "checking_status" && flag == 0)
  {
    std::cout << '\n'
              << "Status: The system is Idle" << std::endl;
    messageDeliver("N1_Idle");
  }

  if (status->data == "stop_bench")
  {
    std::cout << '\n'
              << "Stop Bench is pressed" << std::endl;
    flag = 0;
  }
}

void ImageVisualizer::csv_reader(int argc, char **argv)
{
  std::ifstream input_csv;
  input_csv.open(argv[1]);
  if (!input_csv)
  {
    std::cout << " file could not be opened\n";
    exit(0);
  }
  std::string folder_path;
  while (std::getline(input_csv, folder_path))
  {
    if (folder_path.size() > 0)
    {
      folder_path.erase(std::remove(folder_path.begin(), folder_path.end(), '\"'), folder_path.end());
      std::string sample_keyword = "samples_";
      std::string forwardslash = "/";
      CsvFilename.push_back(folder_path);
      FolderFiles.push_back(default_path + sample_keyword + folder_path + forwardslash + sample_keyword + folder_path + forwardslash);
    }
  }
  input_csv.close();
  for (int i = 0; i < FolderFiles.size(); i++)
    std::cout << "FolderFiles:   " << FolderFiles.at(i) << ' ' << std::endl;
}

void ImageVisualizer::Run(int argc, char **argv)
{
  csv_reader(argc, argv);
  RegisterSubscribers();
  RegisterPublishers();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Node1");
  ImageVisualizer object;
  object.Run(argc, argv);
  ros::spin();
  return 0;
}
