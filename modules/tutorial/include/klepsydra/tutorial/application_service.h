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

#include <klepsydra/core/manage_service.h>
#include <klepsydra/core/system_event.h>

namespace kpsr {

class ApplicationService : public ManagedService
{
public:
    ApplicationService(Environment *environment,
                       Subscriber<SystemEventData> *systemStatusEventSubscriber,
                       std::string serviceName);

    void start();

    void stop();

    void execute();
};

} // namespace kpsr
