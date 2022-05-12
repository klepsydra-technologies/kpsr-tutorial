#include <iostream>
#include <kpsr_ros_core/from_ros_middleware_provider.h>
#include <kpsr_ros_core/to_ros_middleware_provider.h>
#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>
#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kpsr_ros_subscriber");
    ros::NodeHandle nodeHandle;

    std::string topicName = "tutorial_api_example5";
    ros::Publisher strPublisher = nodeHandle.advertise<std_msgs::String>(topicName, 20);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    // Declare the eventloop and vector publisher
    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);

    eventloop.start();

    // auto strpublisher = eventloop.getPublisher<std_msgs::String>("example5", 0, nullptr, nullptr);
    fromRosProvider.registerToTopic<std_msgs::String>(
        topicName.c_str(), 10, eventloop.getPublisher<std_msgs::String>(topicName, 0, nullptr, nullptr));

    eventloop.getSubscriber<std_msgs::String>(topicName)
        ->registerListener("listener", [](const std_msgs::String &message) {
            std::cout << "Message received: " << message.data << std::endl;
            std::cout << "Eventloop (subscriber) thread ID: " << std::this_thread::get_id()
                      << std::endl;
        });
    std::this_thread::sleep_for(std::chrono::milliseconds(
        1)); // Ensures listener has been registered before publisher runs.

    // Set up sample publisher.
    std_msgs::String msg;
    msg.data = "Hello World!";
    strPublisher.publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    eventloop.getSubscriber<std_msgs::String>("example5")->removeListener("listener");
    eventloop.stop();

    return 0;
}