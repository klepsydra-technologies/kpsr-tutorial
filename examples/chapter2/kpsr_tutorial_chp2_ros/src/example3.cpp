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

#include "kpsr_tutorial_chp2_ros/sum_vector_data.h"

#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <klepsydra/serialization/identity_mapper.h>

#include <kpsr_ros_core/from_ros_middleware_provider.h>
#include <kpsr_ros_core/to_ros_middleware_provider.h>
#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "kpsr_ros_subscriber");
    ros::NodeHandle nodeHandle;

    // Declare the eventloop and vector publisher
    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);
    eventloop.start();
      
    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);
    ros::Publisher rosVectorPublisher = nodeHandle.advertise<std_msgs::Float32MultiArray>("vector", 1);

    // Set up the klepsydra class to publish ROS vectors.
    auto vectorPublisher = toRosProvider.getToMiddlewareChannel<std_msgs::Float32MultiArray, std_msgs::Float32MultiArray>(
        "vector", 10, nullptr, rosVectorPublisher);

    // Declare the ROS publisher that will publish the vector sum
    std::string vectorSum("VectorSum");
    ros::Publisher rosPublisher = nodeHandle.advertise<std_msgs::Float32>(vectorSum, 1);
    // Set up corresponding kelpsydra publisher.
    kpsr::Publisher<float> * sumPublisher = toRosProvider.getToMiddlewareChannel<float, std_msgs::Float32>(
        vectorSum, 10, nullptr, rosPublisher);   

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);

    // Register ROS subscriber for the ROS vector.
    fromRosProvider.registerToTopic<std_msgs::Float32MultiArray, std_msgs::Float32MultiArray>(
        "vector", 10, eventloop.getPublisher<std_msgs::Float32MultiArray>("vector", 10, nullptr, nullptr));
    {
        SumVectorData sumVectorData(
            eventloop.getSubscriber<std_msgs::Float32MultiArray>("vector"),
            sumPublisher);
        // Define the ROS subscriber to receive the vector sum
        fromRosProvider.registerToTopic<float, std_msgs::Float32>(
            vectorSum.c_str(), 10, eventloop.getPublisher<float>(vectorSum, 10, nullptr, nullptr));

        // Register the listener that will print the sum to screen.
        eventloop.getSubscriber<float>(vectorSum)->registerListener(
            "sum",
            [](const float & message) {
                std::cout << "Sum received: " << message << std::endl;
            });


        // Set up sample publisher.
        std::thread t([&vectorPublisher]() {
                          std_msgs::Float32MultiArray msg;
                          int const vectorLength(10);
                          msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  
                          msg.layout.dim[0].size = vectorLength;
                          msg.layout.dim[0].stride = 1;
                          for (int i = 0; i < 100; i ++) {
                              msg.data.clear();
                              std::vector<float> vector(vectorLength);
                              for (int j = 0; j < vectorLength; j++) {
                                  vector[j] = static_cast<float>(j+1);
                              }
                              msg.data.insert(msg.data.end(), vector.begin(), vector.end());
                              vectorPublisher->publish(msg);
                              ros::spinOnce();
                          }
                      });
        t.join();
    }
    eventloop.getSubscriber<float>(vectorSum)->removeListener("sum");
    eventloop.stop();
}
