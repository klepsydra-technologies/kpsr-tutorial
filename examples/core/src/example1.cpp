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

#include <thread>

#include <klepsydra/mem_core/basic_middleware_provider.h>

#include "spdlog/spdlog.h"

#include "klepsydra/tutorial/battery_state.h"
#include "klepsydra/tutorial/temperature.h"
#include <klepsydra/tutorial/application_service.h>
#include <klepsydra/tutorial/control_service.h>

int main()
{
    const std::string batteryTopicName = "testBattery";
    const std::string temperatureTopicName = "testTemperature";
    const std::string statusTopicName = "testStatus";
    const std::string serviceNameManaged = "application_service";

    static const int QUEUE_SIZE = 5;
    static const int POOL_SIZE = 0;

    kpsr::mem::BasicMiddlewareProvider<kpsr::sensors::BatteryState>
        batteryProvider(nullptr, batteryTopicName, QUEUE_SIZE, POOL_SIZE, nullptr, nullptr, false);
    kpsr::mem::BasicMiddlewareProvider<kpsr::sensors::Temperature> temperatureProvider(
        nullptr, temperatureTopicName, QUEUE_SIZE, POOL_SIZE, nullptr, nullptr, false);
    kpsr::mem::BasicMiddlewareProvider<kpsr::SystemEventData>
        statusProvider(nullptr, statusTopicName, QUEUE_SIZE, POOL_SIZE, nullptr, nullptr, false);

    batteryProvider.start();
    temperatureProvider.start();
    statusProvider.start();

    kpsr::ControlService controlService(nullptr,
                                        batteryProvider.getSubscriber(),
                                        temperatureProvider.getSubscriber(),
                                        statusProvider.getPublisher());

    kpsr::ApplicationService applicationClass(nullptr,
                                              statusProvider.getSubscriber(),
                                              serviceNameManaged);

    std::thread batterySensorThread([&batteryProvider] {
        kpsr::sensors::BatteryState dummyState;
        for (int i = 0; i < 10; i++) {
            dummyState.percentage = 0.7 - (0.06 * i);
            batteryProvider.getPublisher()->publish(dummyState);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    std::thread temperatureSensorThread([&temperatureProvider] {
        kpsr::sensors::Temperature dummyTemp;
        for (int i = 0; i < 10; i++) {
            dummyTemp.temperature = 35 + (0.5 * i);
            temperatureProvider.getPublisher()->publish(dummyTemp);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    spdlog::info("Start the listeners");
    controlService.start();

    // Do something here. to get the info?

    temperatureSensorThread.join();
    batterySensorThread.join();
    controlService.stop();
    spdlog::info("Stopping program");
    batteryProvider.stop();
}
