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
#include <klepsydra/tutorial/control_service.h>

int main()
{
    const std::string batteryTopicName = "testBattery";
    const std::string temperatureTopicName = "testTemperature";
    const std::string statusTopicName = "testStatus";
    const std::string serviceNameManaged = "application_service";
    static const int BUFFER_SIZE = 8; // Must be a power of 2
    static const int POOL_SIZE = 0;
    kpsr::high_performance::EventLoopMiddlewareProvider<BUFFER_SIZE> provider(nullptr);

    provider.start();

    kpsr::ControlService controlService(
        nullptr,
        provider.getSubscriber<kpsr::sensors::BatteryState>(batteryTopicName),
        provider.getSubscriber<kpsr::sensors::Temperature>(temperatureTopicName),
        provider.getPublisher<kpsr::SystemEventData>(statusTopicName, POOL_SIZE, nullptr, nullptr));

    kpsr::ApplicationService applicationClass(nullptr,
                                              provider.getSubscriber<kpsr::SystemEventData>(
                                                  statusTopicName),
                                              serviceNameManaged);

    std::thread batterySensorThread([&provider, &batteryTopicName] {
        kpsr::sensors::BatteryState dummyState;
        for (int i = 0; i < 10; i++) {
            dummyState.percentage = 0.7 - (0.06 * i);
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
    provider.stop();
    spdlog::info("Stopping program");
}
