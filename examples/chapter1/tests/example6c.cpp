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
#include <stdio.h>
#include <thread>

#include "gtest/gtest.h"
#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>

#include <klepsydra/mem_core/mem_env.h>

#include "application_service.h"
#include "control_service.h"

TEST(ApplicationServiceTest, nominalTest)
{
    // Create the environment.
    kpsr::mem::MemEnv environment;
    // Set the dummy values of environment variables to test with.
    environment.setPropertyFloat("low_threshold", 5.0f);
    environment.setPropertyFloat("high_threshold", 15.0f);

    kpsr::EventEmitterMiddlewareProvider<float> batteryValueProvider(
        nullptr, "battery_provider", 10, [](float &element) { element = 0.0f; }, nullptr);
    kpsr::EventEmitterMiddlewareProvider<kpsr::SystemEventData> statusProvider(nullptr,
                                                                               "status_provider",
                                                                               10,
                                                                               nullptr,
                                                                               nullptr);

    kpsr::ControlService controlService(batteryValueProvider.getSubscriber(),
                                        statusProvider.getPublisher(),
                                        &environment);
    controlService.startup();
    kpsr::ApplicationService applicationService(&environment,
                                                statusProvider.getSubscriber(),
                                                "application");
    kpsr::mem::CacheListener<kpsr::SystemEventData> eventListener;
    statusProvider.getSubscriber()->registerListener("cacheListener",
                                                     eventListener.cacheListenerFunction);

    { // Test with high value.
        batteryValueProvider.getPublisher()->publish(30.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        auto lastEvent = eventListener.getLastReceivedEvent();
        ASSERT_NE(lastEvent, nullptr);
        ASSERT_EQ(*lastEvent, kpsr::SystemEventData::Start);
        ASSERT_TRUE(applicationService._started);
    }
    {
        // Test with low value
        batteryValueProvider.getPublisher()->publish(3.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        auto firstEvent = eventListener.getLastReceivedEvent();
        ASSERT_NE(firstEvent, nullptr);
        ASSERT_EQ(*firstEvent, kpsr::SystemEventData::Stop);
        ASSERT_FALSE(applicationService._started);
    }

    controlService.shutdown();
    statusProvider.getSubscriber()->removeListener("cacheListener");
}
