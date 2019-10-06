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

#include <klepsydra/mem_core/basic_middleware_provider.h>

#include "spdlog/spdlog.h"

#include <klepsydra/tutorial/control_service.h>
#include <klepsydra/tutorial/application_service.h>
#include "klepsydra/tutorial/battery_state.h"
#include "klepsydra/tutorial/temperature.h"

int main() {

    const std::string batteryTopicName = "testBattery";
    const std::string temperatureTopicName = "testTemperature";
    const std::string statusTopicName = "testStatus";
    const std::string serviceNameManaged = "application_service";

    static const int QUEUE_SIZE = 5;
    static const int POOL_SIZE = 0;

    kpsr::mem::BasicMiddlewareProvider<kpsr::sensors::BatteryState> batteryProvider(
        nullptr, batteryTopicName, QUEUE_SIZE, POOL_SIZE, nullptr, nullptr, false);
    kpsr::mem::BasicMiddlewareProvider<kpsr::sensors::Temperature> temperatureProvider(
        nullptr, temperatureTopicName, QUEUE_SIZE, POOL_SIZE, nullptr, nullptr, false);
    kpsr::mem::BasicMiddlewareProvider<kpsr::SystemEventData> statusProvider(
        nullptr, statusTopicName, QUEUE_SIZE, POOL_SIZE, nullptr, nullptr, false);

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

    std::thread batterySensorThread(
        [&batteryProvider]{
            kpsr::sensors::BatteryState dummyState;
            for (int i = 0; i < 10; i++) {
                dummyState.percentage = 0.7 - (0.06 * i);
                batteryProvider.getPublisher()->publish(dummyState);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        });
    std::thread temperatureSensorThread(
        [&temperatureProvider]{
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
