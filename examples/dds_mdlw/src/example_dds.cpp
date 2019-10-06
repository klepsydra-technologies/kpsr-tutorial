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
#include <klepsydra/mem_core/mem_env.h>

#include <klepsydra/dds_core/to_dds_middleware_provider.h>
#include <klepsydra/dds_core/from_dds_middleware_provider.h>
#include <klepsydra/dds_serialization/primitive_type_dds_mapper.h>
#include <klepsydra/dds_serialization/enum_dds_mapper.h>

#include "spdlog/spdlog.h"

#include <klepsydra/tutorial/control_service.h>
#include <klepsydra/tutorial/application_service.h>
#include "klepsydra/tutorial/battery_state.h"
#include "klepsydra/tutorial/temperature.h"

#include "klepsydra/tutorial/dds/battery_state_dds_mapper.h"
#include "klepsydra/tutorial/dds/temperature_dds_mapper.h"

int main() {

    std::string batteryTopicName = "testBattery";
    std::string temperatureTopicName = "testTemperature";
    std::string statusTopicName = "testStatus";
    std::string serviceNameManaged = "application_service";
    static const int BUFFER_SIZE = 8;  // Must be a power of 2
    static const int POOL_SIZE = 0;

    kpsr::mem::MemEnv environment;

    kpsr::high_performance::EventLoopMiddlewareProvider<BUFFER_SIZE> provider(nullptr);

    dds::domain::DomainParticipant dp(0);
    dds::sub::Subscriber sub(dp);
    dds::pub::Publisher pub(dp);
    dds::topic::Topic<kpsr_dds_sensors::Temperature> topicTemperature(dp, temperatureTopicName);
    dds::topic::Topic<kpsr_dds_sensors::BatteryState> topicBattery(dp, batteryTopicName);
    dds::topic::Topic<kpsr_dds_serialization::LongData> topicStatus(dp, statusTopicName);

    kpsr::dds_mdlw::ToDDSMiddlewareProvider toDDSProvider(nullptr);
    dds::pub::DataWriter<kpsr_dds_serialization::LongData> statusDataWriter(pub, topicStatus);
    kpsr::Publisher<kpsr::SystemEventData> * statusPublisher =
        toDDSProvider.getToMiddlewareChannel<kpsr::SystemEventData, kpsr_dds_serialization::LongData>(
            statusTopicName, 0, nullptr, &statusDataWriter);

    // Creation of the From DDS Provider. It is a one-liner only needed once for all readers.
    kpsr::dds_mdlw::FromDDSMiddlewareProvider fromDDSProvider;
    // Creating a concrete DDS reader and connect it to the publisher side
    dds::sub::DataReader<kpsr_dds_sensors::BatteryState> batteryDataReader(sub, topicBattery);
    fromDDSProvider.registerToTopic(
        batteryTopicName,
        &batteryDataReader,
        true,
        provider.getPublisher<kpsr::sensors::BatteryState>(batteryTopicName, POOL_SIZE, nullptr, nullptr));

    // Finally, we pass the associated subscriber to the service that is going to use. Please note that this is
    // Transparent of the DDS completely, and yet extremely performing.
	kpsr::ControlService controlService(&environment,
                                        provider.getSubscriber<kpsr::sensors::BatteryState>(batteryTopicName),
                                        provider.getSubscriber<kpsr::sensors::Temperature>(temperatureTopicName),
                                        statusPublisher);

    // Creating a concrete DDS reader and connect it to the publisher side
    dds::sub::DataReader<kpsr_dds_serialization::LongData> statusDataReader(sub, topicStatus);
    fromDDSProvider.registerToTopic(
        statusTopicName,
        &statusDataReader,
        true,
        provider.getPublisher<kpsr::SystemEventData>(statusTopicName, POOL_SIZE, nullptr, nullptr));

    kpsr::ApplicationService applicationClass(&environment,
                                              provider.getSubscriber<kpsr::SystemEventData>(statusTopicName),
                                              serviceNameManaged);

    provider.start();

	std::thread batterySensorThread(
        [&pub, &toDDSProvider, &topicBattery, &batteryTopicName]{
            dds::pub::DataWriter<kpsr_dds_sensors::BatteryState> batteryDataWriter(pub, topicBattery);
            kpsr::Publisher<kpsr::sensors::BatteryState> * batteryPublisher =
                toDDSProvider.getToMiddlewareChannel<kpsr::sensors::BatteryState, kpsr_dds_sensors::BatteryState>(
                    batteryTopicName, POOL_SIZE, nullptr, &batteryDataWriter);
            kpsr::sensors::BatteryState dummyState;
            for (int i = 0; i < 11; i++) {
                dummyState.percentage = 0.7 - (0.06 * i);
				batteryPublisher->publish(dummyState);
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        });
	std::thread temperatureSensorThread(
        [&pub, &toDDSProvider, &topicTemperature, &temperatureTopicName]{
            dds::pub::DataWriter<kpsr_dds_sensors::Temperature> temperatureDataWriter(pub, topicTemperature);
            kpsr::Publisher<kpsr::sensors::Temperature> * temperaturePublisher =
                toDDSProvider.getToMiddlewareChannel<kpsr::sensors::Temperature, kpsr_dds_sensors::Temperature>(
                    temperatureTopicName, POOL_SIZE, nullptr, &temperatureDataWriter);
            kpsr::sensors::Temperature dummyState;
            for (int i = 0; i < 11; i++) {
                dummyState.temperature = 34 - (0.06 * i);
				temperaturePublisher->publish(dummyState);
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        });
    spdlog::info("Start the listeners");
    controlService.start();

    // Do something here. to get the info?

    temperatureSensorThread.join();
    batterySensorThread.join();
    fromDDSProvider.unregisterFromTopic(batteryTopicName, &batteryDataReader);
    fromDDSProvider.unregisterFromTopic(statusTopicName, &statusDataReader);
    controlService.stop();
    spdlog::info("Stopping program");

    provider.stop();
}
