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

#include "spdlog/spdlog.h"
#include <klepsydra/tutorial/application_service.h>

kpsr::ApplicationService::ApplicationService(
    kpsr::Environment *environment,
    kpsr::Subscriber<SystemEventData> *systemStatusEventSubscriber,
    std::string serviceName)
    : ManagedService(environment, systemStatusEventSubscriber, serviceName)
{
    spdlog::info("Created object");
}

void kpsr::ApplicationService::start()
{
    // Actions to start the cubesat.
    spdlog::info("Started cubesat");
}

void kpsr::ApplicationService::stop()
{
    // Actions to put the cubesat in energy saving mode.
    spdlog::info("Stopping cubesat");
}

void kpsr::ApplicationService::execute() {}
