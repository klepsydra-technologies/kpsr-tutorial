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

#include "kpsr_tutorial_chp2_ros/sum_vector_data.h"

#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <klepsydra/serialization/identity_mapper.h>

#include <kpsr_ros_core/from_ros_middleware_provider.h>
#include <kpsr_ros_core/to_ros_middleware_provider.h>
#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kpsr_ros_subscriber");
    ros::NodeHandle nodeHandle;

    // Declare the eventloop and vector publisher
    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);
    eventloop.start();

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);
    ros::Publisher rosVectorPublisher = nodeHandle.advertise<std_msgs::Float32MultiArray>("vector",
                                                                                          1);

    // Set up the klepsydra class to publish ROS vectors.
    auto vectorPublisher =
        toRosProvider.getToMiddlewareChannel<std_msgs::Float32MultiArray,
                                             std_msgs::Float32MultiArray>("vector",
                                                                          10,
                                                                          nullptr,
                                                                          rosVectorPublisher);

    // Declare the ROS publisher that will publish the vector sum
    std::string vectorSum("VectorSum");
    ros::Publisher rosPublisher = nodeHandle.advertise<std_msgs::Float32>(vectorSum, 1);
    // Set up corresponding kelpsydra publisher.
    kpsr::Publisher<float> *sumPublisher =
        toRosProvider.getToMiddlewareChannel<float, std_msgs::Float32>(vectorSum,
                                                                       10,
                                                                       nullptr,
                                                                       rosPublisher);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);

    // Register ROS subscriber for the ROS vector.
    fromRosProvider.registerToTopic<std_msgs::Float32MultiArray, std_msgs::Float32MultiArray>(
        "vector",
        10,
        eventloop.getPublisher<std_msgs::Float32MultiArray>("vector", 10, nullptr, nullptr));
    {
        SumVectorData sumVectorData(eventloop.getSubscriber<std_msgs::Float32MultiArray>("vector"),
                                    sumPublisher);
        // Define the ROS subscriber to receive the vector sum
        fromRosProvider.registerToTopic<float, std_msgs::Float32>(
            vectorSum.c_str(), 10, eventloop.getPublisher<float>(vectorSum, 10, nullptr, nullptr));

        // Register the listener that will print the sum to screen.
        eventloop.getSubscriber<float>(vectorSum)->registerListener("sum", [](const float &message) {
            std::cout << "Sum received: " << message << std::endl;
        });

        // Set up sample publisher.
        std::thread t([&vectorPublisher]() {
            std_msgs::Float32MultiArray msg;
            int const vectorLength(10);
            msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            msg.layout.dim[0].size = vectorLength;
            msg.layout.dim[0].stride = 1;
            for (int i = 0; i < 100; i++) {
                msg.data.clear();
                std::vector<float> vector(vectorLength);
                for (int j = 0; j < vectorLength; j++) {
                    vector[j] = static_cast<float>(j + 1);
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
