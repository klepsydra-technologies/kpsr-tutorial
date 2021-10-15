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

#include <thread>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

#include "spdlog/spdlog.h"

#include "klepsydra/tutorial/battery_state.h"
#include "klepsydra/tutorial/temperature.h"
#include <klepsydra/tutorial/application_service.h>

#include "sensor_msgs/battery_state_ros_mapper.h"
#include "sensor_msgs/temperature_ros_mapper.h"
#include <kpsr_ros_serialization/enum_ros_mapper.h>

#include <kpsr_ros_core/from_ros_middleware_provider.h>
#include <kpsr_ros_core/ros_env.h>
#include <kpsr_ros_core/to_ros_middleware_provider.h>

#include "control_service_ros.h"

int main()
{
    std::string batteryTopicName = "testBattery";
    std::string temperatureTopicName = "testTemperature";
    std::string statusTopicName = "testStatus";
    std::string serviceNameManaged = "application_service";
    static const int BUFFER_SIZE = 8; // Must be a power of 2
    static const int QUEUE_SIZE = 5;
    static const int POOL_SIZE = 0;
    static const int ROS_QUEUE_SIZE = 1;

    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, "kpsr_ros_example");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    kpsr::high_performance::EventLoopMiddlewareProvider<BUFFER_SIZE> provider(nullptr);

    ros::Publisher statusPublisher = nodeHandle.advertise<std_msgs::Int32>(statusTopicName, 1);
    // Create the main to ros wizard instance. One for the whole application.
    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::SystemEventData> *kpsrStatusPublisher =
        toRosProvider
            .getToMiddlewareChannel<kpsr::SystemEventData, std_msgs::Int32>(statusTopicName.c_str(),
                                                                            1,
                                                                            nullptr,
                                                                            statusPublisher);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<sensor_msgs::BatteryState>(
        batteryTopicName.c_str(),
        ROS_QUEUE_SIZE,
        provider.getPublisher<sensor_msgs::BatteryState>(batteryTopicName,
                                                         POOL_SIZE,
                                                         nullptr,
                                                         nullptr));

    kpsr::ControlServiceRos controlService(&environment,
                                           provider.getSubscriber<sensor_msgs::BatteryState>(
                                               batteryTopicName),
                                           provider.getSubscriber<sensor_msgs::Temperature>(
                                               temperatureTopicName),
                                           kpsrStatusPublisher);
    fromRosProvider.registerToTopic<kpsr::SystemEventData, std_msgs::Int32>(
        statusTopicName.c_str(),
        ROS_QUEUE_SIZE,
        provider.getPublisher<kpsr::SystemEventData>(statusTopicName, POOL_SIZE, nullptr, nullptr));

    kpsr::ApplicationService applicationClass(&environment,
                                              provider.getSubscriber<kpsr::SystemEventData>(
                                                  statusTopicName),
                                              serviceNameManaged);
    provider.start();

    std::thread batterySensorThread([&rate, &batteryTopicName, &nodeHandle] {
        ros::Publisher batteryPublisher =
            nodeHandle.advertise<::sensor_msgs::BatteryState>(batteryTopicName, 1);
        sensor_msgs::BatteryState dummyState;
        for (int i = 0; i < 11; i++) {
            dummyState.percentage = 0.7 - (0.06 * i);
            batteryPublisher.publish(dummyState);
            ros::spinOnce();
            rate.sleep();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    std::thread temperatureSensorThread([&rate, &temperatureTopicName, &nodeHandle] {
        ros::Publisher temperaturePublisher =
            nodeHandle.advertise<sensor_msgs::Temperature>(temperatureTopicName, 1);
        sensor_msgs::Temperature dummyState;
        for (int i = 0; i < 11; i++) {
            dummyState.temperature = 34 - (0.06 * i);
            temperaturePublisher.publish(dummyState);
            ros::spinOnce();
            rate.sleep();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    spdlog::info("Start the listeners");
    controlService.start();

    // Do something here. to get the info?

    temperatureSensorThread.join();
    batterySensorThread.join();
    ros::spinOnce();
    rate.sleep();
    controlService.stop();
    spdlog::info("Stopping program");
    provider.stop();
}
