/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef APPLICATION_SERVICE_H
#define APPLICATION_SERVICE_H

#include <iostream>
#include <klepsydra/core/manage_service.h>

namespace kpsr {
class ApplicationService : public ManagedService
{
public:
    ApplicationService(kpsr::Environment *environment,
                       kpsr::Subscriber<SystemEventData> *systemStatusEventSubscriber,
                       std::string const &serviceName)
        : ManagedService(environment, systemStatusEventSubscriber, serviceName)
    {}

protected:
    void start() { std::cout << "Started application\n"; }

    void stop() { std::cout << "Stopping application\n"; }

    void execute() {}
};
} // namespace kpsr
#endif
