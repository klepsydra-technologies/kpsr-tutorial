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

#include <string>

#include <gtest/gtest.h>

#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <klepsydra/core/subscriber.h>

#include "sensor_msgs/battery_state_ros_mapper.h"
#include <kpsr_ros_serialization/enum_ros_mapper.h>

#include <kpsr_ros_core/from_ros_middleware_provider.h>
#include <kpsr_ros_core/to_ros_middleware_provider.h>

#include "klepsydra/tutorial/battery_state.h"
#include "klepsydra/tutorial/temperature.h"
#include <klepsydra/tutorial/application_service.h>
#include <klepsydra/tutorial/control_service.h>

TEST(KprsRosTest, batteryTest)
{
    int argc = 0;
    char **argv = nullptr;
    std::string batteryTopicName = "testBattery";
    std::string temperatureTopicName = "testTemperature";
    std::string statusTopicName = "testStatus";
    std::string serviceNameManaged = "application_service";
    const int POOL_SIZE = 0;

    ros::init(argc, argv, "kpsr_ros_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(1000);

    kpsr::EventEmitterMiddlewareProvider<kpsr::sensors::BatteryState>
        batteryProvider(nullptr, batteryTopicName, POOL_SIZE, nullptr, nullptr);
    kpsr::EventEmitterMiddlewareProvider<kpsr::sensors::Temperature>
        temperatureProvider(nullptr, temperatureTopicName, POOL_SIZE, nullptr, nullptr);
    kpsr::EventEmitterMiddlewareProvider<kpsr::SystemEventData> statusProvider(nullptr,
                                                                               statusTopicName,
                                                                               POOL_SIZE,
                                                                               nullptr,
                                                                               nullptr);

    ros::Publisher statusPublisher = nodeHandle.advertise<std_msgs::Int32>(statusTopicName, 1);
    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::SystemEventData> *kpsrStatusPublisher =
        toRosProvider
            .getToMiddlewareChannel<kpsr::SystemEventData, std_msgs::Int32>(statusTopicName.c_str(),
                                                                            1,
                                                                            nullptr,
                                                                            statusPublisher);
    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<kpsr::sensors::BatteryState, sensor_msgs::BatteryState>(
        batteryTopicName.c_str(), 1, batteryProvider.getPublisher());

    kpsr::ControlService controlService(nullptr,
                                        batteryProvider.getSubscriber(),
                                        temperatureProvider.getSubscriber(),
                                        statusProvider.getPublisher());

    controlService.start();

    sensor_msgs::BatteryState sampleBatteryValue;
    sampleBatteryValue.percentage = 0.34;
    std::thread batterySensorThread([&rate, &batteryTopicName, &nodeHandle, &sampleBatteryValue] {
        ros::Publisher batteryPublisher =
            nodeHandle.advertise<sensor_msgs::BatteryState>(batteryTopicName.c_str(), 1);
        batteryPublisher.publish(sampleBatteryValue);
        ros::spinOnce();
        rate.sleep();
        ros::spinOnce();
        rate.sleep();
    });

    batterySensorThread.join();
    auto receivedValue = controlService.getBatteryValue();
    ASSERT_FLOAT_EQ(sampleBatteryValue.percentage, receivedValue);
    controlService.stop();
}
