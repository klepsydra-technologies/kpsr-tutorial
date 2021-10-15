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
