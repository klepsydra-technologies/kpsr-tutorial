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

#include <kpsr_ros_core/from_ros_middleware_provider.h>
#include <kpsr_ros_core/to_ros_middleware_provider.h>
#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

int main(int argc, char ** argv) {
   ros::init(argc, argv, "kpsr_ros_subscriber");
   ros::NodeHandle nodeHandle;

   std::string topicName = "tutorial_api_example2";
   ros::Publisher stringPublisher = nodeHandle.advertise<std_msgs::String>(topicName, 10);

   kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
   kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);
   eventloop.start();
   
   fromRosProvider.registerToTopic<std::string, std_msgs::String>(topicName.c_str(), 10,
                                                                  eventloop.getPublisher<std::string>(topicName, 0, nullptr, nullptr));

   eventloop.getSubscriber<std::string>(topicName)->registerListener(topicName, [](const std::string & message) {
         std::cout << "Message received: " << message << std::endl;
      }
   );

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
