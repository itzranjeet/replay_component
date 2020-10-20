#include "replay_component/Node5.hpp"

void ControlNode::RegisterPublishers()
{
    trigger_pub = nh.advertise<std_msgs::String>(N5_TriggerStatusMessageTopic, N5_TriggerStatusMessageQueueSize);
}

void ControlNode::ServerFunction()
{
label:
    int sockfd, newsockfd, portno;
    socklen_t clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n;
    portno = 5005;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0)
    {
        std::cout << "ERROR opening socket" << std::endl;
    }
    int enable = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
    {
        std::cout << "setsockopt(SO_REUSEADDR) failed" << std::endl;
    }

    bzero((char *)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        std::cout << "ERROR on binding" << std::endl;
    }
    listen(sockfd, 5);
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);

    if (newsockfd < 0)
    {
        std::cout << "ERROR on accept" << std::endl;
    }

    while (1)
    {
        bzero(buffer, 256);
        n = recv(newsockfd, buffer, 256, 0);
        std::cout << "The Value of n:" << n << std::endl;
        if (n < 0)
        {
            std::cout << "ERROR reading from socket" << std::endl;
        }

        if (n == 0)
            continue;
        if (n > 0)
        {
            msg.data = buffer;
            std::cout << "buffer:" << msg.data << std::endl;
            trigger_pub.publish(msg);
            ROS_INFO("Data published %s", msg.data.c_str());
            ros::spinOnce();
        }

        n = send(newsockfd, buffer, strlen(buffer), 0);

        if (n < 0)
        {
            std::cout << "ERROR writing to socket" << std::endl;
        }

        close(newsockfd);
        close(sockfd);
        goto label;
    }
}

void ControlNode::Run()
{
    RegisterPublishers();
    ServerFunction();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Node5");
    ControlNode object;
    object.Run();
    return 0;
}
