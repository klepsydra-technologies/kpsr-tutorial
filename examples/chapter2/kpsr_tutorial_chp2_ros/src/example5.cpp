#include "kpsr_tutorial_chp2_ros/simple_publisher.h"
#include <iostream>
#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <klepsydra/serialization/identity_mapper.h>
#include <kpsr_ros_core/from_ros_middleware_provider.h>
#include <kpsr_ros_core/to_ros_middleware_provider.h>
#include <std_msgs/String.h>

void callbackFunction(const std_msgs::String &message)
{
    std::cout << "Message received: " << message.data << std::endl;
    std::cout << "Callback (ros subscriber) thread ID: " << std::this_thread::get_id() << std::endl;
}

void rosTosKpsr(ros::NodeHandle &nodeHandle)
{
    std::string topicName1 = "tutorial_api_example5_1";
    ros::Publisher strPublisher = nodeHandle.advertise<std_msgs::String>(topicName1, 20);
    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    // Declare the eventloop and vector publisher
    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);

    eventloop.start();

    // auto strpublisher = eventloop.getPublisher<std_msgs::String>("example5", 0, nullptr, nullptr);
    fromRosProvider.registerToTopic<std_msgs::String>(
        topicName1.c_str(),
        10,
        eventloop.getPublisher<std_msgs::String>(topicName1, 0, nullptr, nullptr));

    eventloop.getSubscriber<std_msgs::String>(topicName1)
        ->registerListener("listener", [](const std_msgs::String &message) {
            std::cout << "Message received: " << message.data << std::endl;
            std::cout << "Eventloop (subscriber) thread ID: " << std::this_thread::get_id()
                      << std::endl;
        });
    std::this_thread::sleep_for(std::chrono::milliseconds(
        1)); // Ensures listener has been registered before publisher runs.

    // Set up sample publisher.
    std_msgs::String msg;
    msg.data = "ROS message from ROS";
    strPublisher.publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    eventloop.getSubscriber<std_msgs::String>("example5")->removeListener("listener");
    eventloop.stop();
}

void kpsrToRos(ros::NodeHandle &nodeHandle)
{
    std::string topicName2 = "tutorial_api_example5_2";
    ros::Publisher stringPublisher = nodeHandle.advertise<std_msgs::String>(topicName2, 1);

    ros::Subscriber sub = nodeHandle.subscribe(topicName2, 1000, callbackFunction);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<std_msgs::String> *kpsrPublisher =
        toRosProvider.getToMiddlewareChannel<std_msgs::String>(topicName2,
                                                               1,
                                                               nullptr,
                                                               stringPublisher);
    std_msgs::String msg;
    msg.data = "ROS message from KPSR";
    kpsrPublisher->publish(msg);
    std::cout << "KPSR (publisher) thread ID: " << std::this_thread::get_id() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kpsr_ros_subscriber");
    ros::NodeHandle nodeHandle;

    std::thread t1([&]() { rosTosKpsr(nodeHandle); });

    std::thread t2([&]() { kpsrToRos(nodeHandle); });

    t1.join();
    t2.join();

    return 0;
}