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
        ASSERT_TRUE(applicationService.started);
    }
    {
        // Test with low value
        batteryValueProvider.getPublisher()->publish(3.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        auto firstEvent = eventListener.getLastReceivedEvent();
        ASSERT_NE(firstEvent, nullptr);
        ASSERT_EQ(*firstEvent, kpsr::SystemEventData::Stop);
        ASSERT_FALSE(applicationService.started);
    }

    controlService.shutdown();
    statusProvider.getSubscriber()->removeListener("cacheListener");
}
