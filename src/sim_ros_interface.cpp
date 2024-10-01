#include <sim_ros_interface.h>
#include <simPlusPlus/Plugin.h>
#include <simPlusPlus/Handles.h>

#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/bind.hpp>
#include <cstdlib>

void ros_imtr_callback(const sensor_msgs::ImageConstPtr& msg, SubscriberProxy *subscriberProxy)
{
    if(msg->is_bigendian)
    {
        std::cerr << "ros_imtr_callback: error: big endian image not supported" << std::endl;
        return;
    }

    int data_len = msg->step * msg->height;

    imageTransportCallback_in in_args;
    imageTransportCallback_out out_args;

    in_args.width = msg->width;
    in_args.height = msg->height;
    in_args.data.resize(data_len);

    for(unsigned int i = 0; i < msg->height; i++)
    {
        int msg_idx = (msg->height - i - 1) * msg->step;
        int buf_idx = i * msg->step;
        for(unsigned int j = 0; j < msg->step; j++)
        {
            in_args.data[buf_idx + j] = msg->data[msg_idx + j];
        }
    }

    if(!imageTransportCallback(subscriberProxy->topicCallback.scriptId, subscriberProxy->topicCallback.name.c_str(), &in_args, &out_args))
    {
        std::cerr << "ros_imtr_callback: error: failed to call callback" << std::endl;
        return;
    }
}

class Plugin : public sim::Plugin
{
public:
    void onInit()
    {
        if(!getenv("ROS_MASTER_URI"))
            throw std::runtime_error("ROS_MASTER_URI is not set");

        if(!initialize())
            throw std::runtime_error("ROS master is not running");

        if(!registerScriptStuff())
            throw std::runtime_error("failed to register script stuff");

        setExtVersion("ROS Interface Plugin");
        setBuildDate(BUILD_DATE);
    }

    void onCleanup()
    {
        shutdown();
    }

    void onScriptStateAboutToBeDestroyed(int scriptHandle, long long scriptUid)
    {
        for(auto proxy : publisherHandles.find(scriptHandle))
        {
            if(proxy->publisher)
            {
                shutdownPublisher_in in;
                in.publisherHandle = proxy->handle;
                shutdownPublisher_out out;
                shutdownPublisher(&in, &out);
            }
            if(proxy->imageTransportPublisher)
            {
                imageTransportShutdownPublisher_in in;
                in.publisherHandle = proxy->handle;
                imageTransportShutdownPublisher_out out;
                imageTransportShutdownPublisher(&in, &out);
            }
        }
        for(auto proxy : subscriberHandles.find(scriptHandle))
        {
            if(proxy->subscriber)
            {
                shutdownSubscriber_in in;
                in.subscriberHandle = proxy->handle;
                shutdownSubscriber_out out;
                shutdownSubscriber(&in, &out);
            }
            if(proxy->imageTransportSubscriber)
            {
                imageTransportShutdownSubscriber_in in;
                in.subscriberHandle = proxy->handle;
                imageTransportShutdownSubscriber_out out;
                imageTransportShutdownSubscriber(&in, &out);
            }
        }
        for(auto proxy : serviceClientHandles.find(scriptHandle))
        {
            shutdownServiceClient_in in;
            in.serviceClientHandle = proxy->handle;
            shutdownServiceClient_out out;
            shutdownServiceClient(&in, &out);
        }
        for(auto proxy : serviceServerHandles.find(scriptHandle))
        {
            shutdownServiceServer_in in;
            in.serviceServerHandle = proxy->handle;
            shutdownServiceServer_out out;
            shutdownServiceServer(&in, &out);
        }
    }

    void onInstancePass(const sim::InstancePassFlags &flags)
    {
        ros::spinOnce();
    }

    void onMainScriptAboutToBeCalled(int &out)
    {
        int stopSimulationRequestCounter;
        simGetInt32Param(sim_intparam_stop_request_counter, &stopSimulationRequestCounter);
        bool doNotRun = simGetBoolParam(sim_boolparam_rosinterface_donotrunmainscript);
        if(doNotRun > 0)
        {
            if(previousStopSimulationRequestCounter == -1)
                previousStopSimulationRequestCounter = stopSimulationRequestCounter;
            if(previousStopSimulationRequestCounter == stopSimulationRequestCounter)
                out = 0; // this tells CoppeliaSim that we don't wanna execute the main script
        }
        else
            previousStopSimulationRequestCounter = -1;
    }

    void onSimulationAboutToStart()
    {
        previousStopSimulationRequestCounter = -1;
    }

    void subscribe(subscribe_in *in, subscribe_out *out)
    {
        SubscriberProxy *subscriberProxy = new SubscriberProxy();
        subscriberProxy->topicName = in->topicName;
        subscriberProxy->topicType = in->topicType;
        subscriberProxy->topicCallback.scriptId = in->_.scriptID;
        subscriberProxy->topicCallback.name = in->topicCallback;

        ros::TransportHints th;
        th.tcpNoDelay(in->transportHints.tcpNoDelay);
        th.maxDatagramSize(in->transportHints.maxDatagramSize);
        for(const auto &t : in->transportHints.transports)
        {
            if(t == "reliable") th.reliable();
            else if(t == "unreliable") th.unreliable();
            else if(t == "tcp") th.tcp();
            else if(t == "udp") th.udp();
            else throw sim::exception("invalid transport: '%s'", t);
        }

        if(0) {}
#include <sub.cpp>
        else
        {
            throw sim::exception("unsupported message type. please edit and recompile ROS plugin");
        }

        if(!subscriberProxy->subscriber)
        {
            throw sim::exception("failed creation of ROS subscriber");
        }

        out->subscriberHandle = subscriberProxy->handle = subscriberHandles.add(subscriberProxy, in->_.scriptID);
    }

    void shutdownSubscriber(shutdownSubscriber_in *in, shutdownSubscriber_out *out)
    {
        SubscriberProxy *subscriberProxy = subscriberHandles.get(in->subscriberHandle);
        subscriberProxy->subscriber.shutdown();
        delete subscriberHandles.remove(subscriberProxy);
    }

    void subscriberTreatUInt8ArrayAsString(subscriberTreatUInt8ArrayAsString_in *in, subscriberTreatUInt8ArrayAsString_out *out)
    {
        SubscriberProxy *subscriberProxy = subscriberHandles.get(in->subscriberHandle);
        subscriberProxy->wr_opt.uint8array_as_string = true;
    }

    void advertise(advertise_in *in, advertise_out *out)
    {
        PublisherProxy *publisherProxy = new PublisherProxy();
        publisherProxy->topicName = in->topicName;
        publisherProxy->topicType = in->topicType;

        if(0) {}
#include <adv.cpp>
        else
        {
            throw sim::exception("unsupported message type. please edit and recompile ROS plugin");
        }

        if(!publisherProxy->publisher)
        {
            throw sim::exception("failed creation of ROS publisher");
        }

        out->publisherHandle = publisherProxy->handle = publisherHandles.add(publisherProxy, in->_.scriptID);
    }

    void shutdownPublisher(shutdownPublisher_in *in, shutdownPublisher_out *out)
    {
        PublisherProxy *publisherProxy = publisherHandles.get(in->publisherHandle);
        publisherProxy->publisher.shutdown();
        delete publisherHandles.remove(publisherProxy);
    }

    void publisherTreatUInt8ArrayAsString(publisherTreatUInt8ArrayAsString_in *in, publisherTreatUInt8ArrayAsString_out *out)
    {
        PublisherProxy *publisherProxy = publisherHandles.get(in->publisherHandle);
        publisherProxy->rd_opt.uint8array_as_string = true;
    }

    void publish(publish_in *in, publish_out *out)
    {
        PublisherProxy *publisherProxy = publisherHandles.get(in->publisherHandle);

        simMoveStackItemToTop(in->_.stackID, 0);

        if(0) {}
#include <pub.cpp>
        else
        {
            throw sim::exception("unsupported message type. please edit and recompile ROS plugin");
        }
    }

    void serviceClient(serviceClient_in *in, serviceClient_out *out)
    {
        ServiceClientProxy *serviceClientProxy = new ServiceClientProxy();
        serviceClientProxy->serviceName = in->serviceName;
        serviceClientProxy->serviceType = in->serviceType;

        if(0) {}
#include <srvcli.cpp>
        else
        {
            throw sim::exception("unsupported service type. please edit and recompile ROS plugin");
        }

        if(!serviceClientProxy->client)
        {
            throw sim::exception("failed creation of ROS service client");
        }

        out->serviceClientHandle = serviceClientProxy->handle = serviceClientHandles.add(serviceClientProxy, in->_.scriptID);
    }

    void shutdownServiceClient(shutdownServiceClient_in *in, shutdownServiceClient_out *out)
    {
        ServiceClientProxy *serviceClientProxy = serviceClientHandles.get(in->serviceClientHandle);
        serviceClientProxy->client.shutdown();
        delete serviceClientHandles.remove(serviceClientProxy);
    }

    void serviceClientTreatUInt8ArrayAsString(serviceClientTreatUInt8ArrayAsString_in *in, serviceClientTreatUInt8ArrayAsString_out *out)
    {
        ServiceClientProxy *serviceClientProxy = serviceClientHandles.get(in->serviceClientHandle);
        serviceClientProxy->rd_opt.uint8array_as_string = true;
        serviceClientProxy->wr_opt.uint8array_as_string = true;
    }

    void call(call_in *in, call_out *out)
    {
        ServiceClientProxy *serviceClientProxy = serviceClientHandles.get(in->serviceClientHandle);

        simMoveStackItemToTop(in->_.stackID, 0);

        if(0) {}
#include <srvcall.cpp>
        else
        {
            throw sim::exception("unsupported service type. please edit and recompile ROS plugin");
        }
    }

    void advertiseService(advertiseService_in *in, advertiseService_out *out)
    {
        ServiceServerProxy *serviceServerProxy = new ServiceServerProxy();
        serviceServerProxy->serviceName = in->serviceName;
        serviceServerProxy->serviceType = in->serviceType;
        serviceServerProxy->serviceCallback.scriptId = in->_.scriptID;
        serviceServerProxy->serviceCallback.name = in->serviceCallback;

        if(0) {}
#include <srvsrv.cpp>
        else
        {
            throw sim::exception("unsupported service type. please edit and recompile ROS plugin");
        }

        if(!serviceServerProxy->server)
        {
            throw sim::exception("failed creation of ROS service server");
        }

        out->serviceServerHandle = serviceServerProxy->handle = serviceServerHandles.add(serviceServerProxy, in->_.scriptID);
    }

    void shutdownServiceServer(shutdownServiceServer_in *in, shutdownServiceServer_out *out)
    {
        ServiceServerProxy *serviceServerProxy = serviceServerHandles.get(in->serviceServerHandle);
        serviceServerProxy->server.shutdown();
        delete serviceServerHandles.remove(serviceServerProxy);
    }

    void serviceServerTreatUInt8ArrayAsString(serviceServerTreatUInt8ArrayAsString_in *in, serviceServerTreatUInt8ArrayAsString_out *out)
    {
        ServiceServerProxy *serviceServerProxy = serviceServerHandles.get(in->serviceServerHandle);
        serviceServerProxy->rd_opt.uint8array_as_string = true;
        serviceServerProxy->wr_opt.uint8array_as_string = true;
    }

    void sendTransform(sendTransform_in *in, sendTransform_out *out)
    {
        geometry_msgs::TransformStamped t;
        read__geometry_msgs__TransformStamped(in->_.stackID, &t);
        tfbr->sendTransform(t);
    }

    void sendTransforms(sendTransforms_in *in, sendTransforms_out *out)
    {
        std::vector<geometry_msgs::TransformStamped> v;

        sim::moveStackItemToTop(in->_.stackID, 0);
        int i = sim::getStackTableInfo(in->_.stackID, 0);
        if(i < 0)
            throw sim::exception("error reading input argument 1 (origin): expected array");
        int oldsz = sim::getStackSize(in->_.stackID);
        sim::unfoldStackTable(in->_.stackID);
        int sz = (sim::getStackSize(in->_.stackID) - oldsz + 1) / 2;
        for(int i = 0; i < sz; i++)
        {
            sim::moveStackItemToTop(in->_.stackID, oldsz - 1);
            int j;
            read__int32(in->_.stackID, &j);
            simMoveStackItemToTop(in->_.stackID, oldsz - 1);
            geometry_msgs::TransformStamped t;
            read__geometry_msgs__TransformStamped(in->_.stackID, &t);
            v.push_back(t);
        }

        tfbr->sendTransform(v);
    }

    void imageTransportSubscribe(imageTransportSubscribe_in *in, imageTransportSubscribe_out *out)
    {
        SubscriberProxy *subscriberProxy = new SubscriberProxy();
        subscriberProxy->topicName = in->topicName;
        subscriberProxy->topicType = "@image_transport";
        subscriberProxy->topicCallback.scriptId = in->_.scriptID;
        subscriberProxy->topicCallback.name = in->topicCallback;

        subscriberProxy->imageTransportSubscriber = imtr->subscribe(in->topicName, in->queueSize, boost::bind(ros_imtr_callback, _1, subscriberProxy));

        if(!subscriberProxy->imageTransportSubscriber)
        {
            throw sim::exception("failed creation of ROS ImageTransport subscriber");
        }

        out->subscriberHandle = subscriberProxy->handle = subscriberHandles.add(subscriberProxy, in->_.scriptID);
    }

    void imageTransportShutdownSubscriber(imageTransportShutdownSubscriber_in *in, imageTransportShutdownSubscriber_out *out)
    {
        SubscriberProxy *subscriberProxy = subscriberHandles.get(in->subscriberHandle);
        subscriberProxy->imageTransportSubscriber.shutdown();
        delete subscriberHandles.remove(subscriberProxy);
    }

    void imageTransportAdvertise(imageTransportAdvertise_in *in, imageTransportAdvertise_out *out)
    {
        PublisherProxy *publisherProxy = new PublisherProxy();
        publisherProxy->topicName = in->topicName;
        publisherProxy->topicType = "@image_transport";

        publisherProxy->imageTransportPublisher = imtr->advertise(in->topicName, in->queueSize);

        if(!publisherProxy->imageTransportPublisher)
        {
            throw sim::exception("failed creation of ROS ImageTransport publisher");
        }

        out->publisherHandle = publisherProxy->handle = publisherHandles.add(publisherProxy, in->_.scriptID);
    }

    void imageTransportShutdownPublisher(imageTransportShutdownPublisher_in *in, imageTransportShutdownPublisher_out *out)
    {
        PublisherProxy *publisherProxy = publisherHandles.get(in->publisherHandle);
        publisherProxy->imageTransportPublisher.shutdown();
        delete publisherHandles.remove(publisherProxy);
    }

    void imageTransportPublish(imageTransportPublish_in *in, imageTransportPublish_out *out)
    {
        PublisherProxy *publisherProxy = publisherHandles.get(in->publisherHandle);

        sensor_msgs::Image image_msg;
        image_msg.header.stamp = ros::Time::now();
        image_msg.header.frame_id = in->frame_id;
        image_msg.encoding = sensor_msgs::image_encodings::RGB8;
        image_msg.width = in->width;
        image_msg.height = in->height;
        image_msg.step = image_msg.width * 3;

        int data_len = image_msg.step * image_msg.height;
        image_msg.data.resize(data_len);
        image_msg.is_bigendian = 0;

        for(unsigned int i = 0; i < image_msg.height; i++)
        {
            int msg_idx = (image_msg.height - i - 1) * image_msg.step;
            int buf_idx = i * image_msg.step;
            for(unsigned int j = 0; j < image_msg.step; j++)
            {
                image_msg.data[msg_idx + j] = in->data[buf_idx + j];
            }
        }

        publisherProxy->imageTransportPublisher.publish(image_msg);
    }

    void getTime(getTime_in *in, getTime_out *out)
    {
        if(in->flag == 0)
            out->time = ros::Time::now().toSec();
    }

    void getParamString(getParamString_in *in, getParamString_out *out)
    {
        out->value = in->defaultValue;
        out->exists = ros::param::get(in->name, out->value);
    }

    void getParamInt(getParamInt_in *in, getParamInt_out *out)
    {
        out->value = in->defaultValue;
        out->exists = ros::param::get(in->name, out->value);
    }

    void getParamDouble(getParamDouble_in *in, getParamDouble_out *out)
    {
        out->value = in->defaultValue;
        out->exists = ros::param::get(in->name, out->value);
    }

    void getParamBool(getParamBool_in *in, getParamBool_out *out)
    {
        out->value = in->defaultValue;
        out->exists = ros::param::get(in->name, out->value);
    }

    void setParamString(setParamString_in *in, setParamString_out *out)
    {
        ros::param::set(in->name, in->value);
    }

    void setParamInt(setParamInt_in *in, setParamInt_out *out)
    {
        ros::param::set(in->name, in->value);
    }

    void setParamDouble(setParamDouble_in *in, setParamDouble_out *out)
    {
        ros::param::set(in->name, in->value);
    }

    void setParamBool(setParamBool_in *in, setParamBool_out *out)
    {
        ros::param::set(in->name, in->value);
    }

    void hasParam(hasParam_in *in, hasParam_out *out)
    {
        out->exists = ros::param::has(in->name);
    }

    void deleteParam(deleteParam_in *in, deleteParam_out *out)
    {
        ros::param::del(in->name);
    }

    void searchParam(searchParam_in *in, searchParam_out *out)
    {
        out->found = ros::param::search(in->name, out->name);
    }

    bool initialize()
    {
        int argc = 0;
#if _MSC_VER
        char **argv = nullptr;
#else
        char *argv[] = {};
#endif

        int node_name_length = 0;
        char *node_name = nullptr;
        node_name = simGetNamedStringParam("ROSInterface.nodeName", &node_name_length);

        ros::init(argc, argv, node_name && node_name_length ? node_name : "sim_ros_interface");

        if(node_name) simReleaseBuffer(node_name);

        if(!ros::master::check())
            return false;

        nh = new ros::NodeHandle("~");
        tfbr = new tf::TransformBroadcaster();
        imtr = new image_transport::ImageTransport(*nh);

        return true;
    }

    void shutdown()
    {
        ros::shutdown();

        delete imtr;
        delete tfbr;
        delete nh;
    }

private:
    int previousStopSimulationRequestCounter = -1;

    ros::NodeHandle *nh = NULL;

    tf::TransformBroadcaster *tfbr = NULL;
    image_transport::ImageTransport *imtr = NULL;

    sim::Handles<SubscriberProxy*> subscriberHandles;
    sim::Handles<PublisherProxy*> publisherHandles;
    sim::Handles<ServiceClientProxy*> serviceClientHandles;
    sim::Handles<ServiceServerProxy*> serviceServerHandles;
};

SIM_PLUGIN(Plugin)
#include "stubsPlusPlus.cpp"
