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

#ifndef SIMPLE_SUBSCRIBER_SERVICE_H
#define SIMPLE_SUBSCRIBER_SERVICE_H

#include <klepsydra/core/service.h>
#include <klepsydra/core/subscriber.h>

#include <iostream>

class SimpleSubscriberService : public kpsr::Service
{
public:
    SimpleSubscriberService(kpsr::Subscriber<std::string> *subscriber)
        : Service(nullptr, "subcriber_service")
        , _subscriber(subscriber)
    {}

protected:
    void start() override
    {
        _subscriber->registerListener("subscriber_service", [](const std::string &message) {
            std::cout << "Message received: " << message << std::endl;
        });
    }

    void stop() override { _subscriber->removeListener("subscriber_service"); }

    void execute() {} // Empty on purpose.

private:
    kpsr::Subscriber<std::string> *_subscriber;
};
#endif // SIMPLE_SUBSCRIBER_SERVICE_H
