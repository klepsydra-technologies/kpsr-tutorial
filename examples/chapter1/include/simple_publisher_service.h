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

#ifndef SIMPLE_PUBLISHER_SERVICE_H
#define SIMPLE_PUBLISHER_SERVICE_H

#include <klepsydra/sdk/publisher.h>
#include <klepsydra/sdk/service.h>

class SimplePublisherService : public kpsr::Service
{
public:
    SimplePublisherService(kpsr::Publisher<std::string> *publisher)
        : kpsr::Service(nullptr, nullptr, "publisher_service")
        , _publisher(publisher)
    {}

protected:
    void start(){}; // Empty on purpose

    void stop(){}; // Empty on purpose

    void execute() { _publisher->publish("Hello world"); }

private:
    kpsr::Publisher<std::string> *_publisher;
};
#endif // SIMPLE_PUBLISHER_SERVICE_H
