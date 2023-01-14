// Copyright 2023 Klepsydra Technologies AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>

#include "kpsr_tutorial_chp2_ros/simple_publisher.h"

#include <kpsr_ros_core/from_ros_middleware_provider.h>
#include <kpsr_ros_core/to_ros_middleware_provider.h>
#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kpsr_ros_subscriber");
    ros::NodeHandle nodeHandle;

    std::string topicName = "tutorial_api_example2";
    ros::Publisher stringPublisher = nodeHandle.advertise<std_msgs::String>(topicName, 10);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);
    eventloop.start();

    fromRosProvider.registerToTopic<std::string, std_msgs::String>(
        topicName.c_str(), 10, eventloop.getPublisher<std::string>(topicName, 0, nullptr, nullptr));

    eventloop.getSubscriber<std::string>(topicName)
        ->registerListener(topicName, [](const std::string &message) {
            std::cout << "Message received: " << message << std::endl;
            std::cout << "Eventloop (subscriber) thread ID: " << std::this_thread::get_id()
                      << std::endl;
        });

    std_msgs::String testMessage;
    std::stringstream ss;
    ss << "hello world ";
    testMessage.data = ss.str();
    stringPublisher.publish(testMessage);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    eventloop.getSubscriber<std::string>(topicName)->removeListener(topicName);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    eventloop.stop();
    return 0;
}
