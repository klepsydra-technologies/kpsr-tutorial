/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*****************************************************************************/
#include <iostream>
#include "kpsr_tutorial_chp2_ros/simple_publisher.h"

#include <kpsr_ros_core/to_ros_middleware_provider.h>
#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>

void callbackFunction(const std_msgs::String& message) {
    std::cout << "Message received: " << message.data<< std::endl;
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "kpsr_ros_publisher");
    ros::NodeHandle nodeHandle;

    std::string topicName = "tutorial_api_example1";
    ros::Publisher stringPublisher = nodeHandle.advertise<std_msgs::String>(topicName, 1);

    ros::Subscriber sub = nodeHandle.subscribe(
        topicName, 1000,
        callbackFunction);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<std::string> * kpsrPublisher = toRosProvider.getToMiddlewareChannel<std::string, std_msgs::String>(topicName, 1, nullptr, stringPublisher);
    SimplePublisher publisher(kpsrPublisher);

    publisher.run();

    ros::spinOnce();
    return 0;
}
