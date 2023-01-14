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

#include "kpsr_tutorial_chp2_ros/simple_publisher.h"
#include <iostream>

#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <kpsr_ros_core/to_ros_middleware_provider.h>
#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>

void callbackFunction(const std_msgs::String &message)
{
    std::cout << "Message received: " << message.data << std::endl;
    std::cout << "Callback (ros subscriber) thread ID: " << std::this_thread::get_id() << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kpsr_ros_publisher");
    ros::NodeHandle nodeHandle;

    std::string topicName = "tutorial_api_example1";
    ros::Publisher stringPublisher = nodeHandle.advertise<std_msgs::String>(topicName, 1);

    ros::Subscriber sub = nodeHandle.subscribe(topicName, 1000, callbackFunction);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<std::string> *kpsrPublisher =
        toRosProvider.getToMiddlewareChannel<std::string, std_msgs::String>(topicName,
                                                                            1,
                                                                            nullptr,
                                                                            stringPublisher);
    SimplePublisher publisher(kpsrPublisher);

    publisher.run();

    ros::spinOnce();
    return 0;
}
