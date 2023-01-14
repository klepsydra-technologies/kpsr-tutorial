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

#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <klepsydra/mem_core/mem_env.h>

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
    const std::string containerName = "socketContainer";
    static const int BUFFER_SIZE = 8; // Must be a power of 2
    static const int POOL_SIZE = 0;

    kpsr::mem::MemEnv environment;

    kpsr::high_performance::EventLoopMiddlewareProvider<BUFFER_SIZE> provider(nullptr);

    kpsr::ControlService controlService(
        &environment,
        provider.getSubscriber<kpsr::sensors::BatteryState>(batteryTopicName),
        provider.getSubscriber<kpsr::sensors::Temperature>(temperatureTopicName),
        provider.getPublisher<kpsr::SystemEventData>(statusTopicName, POOL_SIZE, nullptr, nullptr));

    provider.start();

    kpsr::ApplicationService applicationClass(&environment,
                                              provider.getSubscriber<kpsr::SystemEventData>(
                                                  statusTopicName),
                                              serviceNameManaged);

    std::thread batterySensorThread([&provider, &batteryTopicName] {
        kpsr::sensors::BatteryState dummyState;
        for (int i = 0; i < 30; i++) {
            dummyState.percentage = 0.7 - (0.02 * i);
            provider
                .getPublisher<kpsr::sensors::BatteryState>(batteryTopicName, 0, nullptr, nullptr)
                ->publish(dummyState);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    std::thread temperatureSensorThread([&provider, &temperatureTopicName] {
        kpsr::sensors::Temperature dummyTemp;
        for (int i = 0; i < 10; i++) {
            dummyTemp.temperature = 35 + (0.5 * i);
            provider
                .getPublisher<kpsr::sensors::Temperature>(temperatureTopicName, 0, nullptr, nullptr)
                ->publish(dummyTemp);
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
    provider.stop();
}
