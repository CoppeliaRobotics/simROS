#ifndef SIM_ROS_INTERFACE_H_INCLUDED
#define SIM_ROS_INTERFACE_H_INCLUDED

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "config.h"
#include "plugin.h"

struct ScriptCallback
{
    int scriptId;
    std::string name;
};

struct Proxy
{
    std::string handle;
};

#include <ros_msg_builtin_io.h>

struct SubscriberProxy : Proxy
{
    std::string topicName;
    std::string topicType;
    ScriptCallback topicCallback;
    ros::Subscriber subscriber;
    image_transport::Subscriber imageTransportSubscriber;
    ROSWriteOptions wr_opt;
};

struct PublisherProxy : Proxy
{
    std::string topicName;
    std::string topicType;
    ros::Publisher publisher;
    image_transport::Publisher imageTransportPublisher;
    ROSReadOptions rd_opt;
};

struct ServiceClientProxy : Proxy
{
    std::string serviceName;
    std::string serviceType;
    ros::ServiceClient client;
    ROSReadOptions rd_opt;
    ROSWriteOptions wr_opt;
};

struct ServiceServerProxy : Proxy
{
    std::string serviceName;
    std::string serviceType;
    ScriptCallback serviceCallback;
    ros::ServiceServer server;
    ROSReadOptions rd_opt;
    ROSWriteOptions wr_opt;
};

#include <stubs.h>
#include <ros_msg_io.h>
#include <ros_srv_io.h>

#endif // SIM_ROS_INTERFACE_H_INCLUDED
