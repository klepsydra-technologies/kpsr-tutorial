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

#ifndef SIMPLE_EVENT_PUBLISHER_H
#define SIMPLE_EVENT_PUBLISHER_H

#include <iostream>
#include <klepsydra/sdk/publisher.h>

#include "test_event.h"

class SimpleEventPublisher
{
public:
    explicit SimpleEventPublisher(kpsr::Publisher<TestEvent> *publisher)
        : _publisher(publisher)
    {}

    void run(const TestEvent &event)
    {
        _publisher->publish(event);
        std::cout << "SimpleEventPublisher publishes TestEvent(" << event.id << "," << event.message
                  << ")\n\tat addr " << &event << "\n\ton thread " << std::this_thread::get_id()
                  << std::endl;
    }

    void runWithoutCopy(std::shared_ptr<TestEvent> eventPtr)
    {
        _publisher->publish(eventPtr);
        std::cout << "SimpleEventPublisher publishes TestEvent(" << eventPtr->id << ","
                  << eventPtr->message << ") without copy\n\tat addr " << eventPtr
                  << "\n\ton thread " << std::this_thread::get_id() << std::endl;
    }

private:
    kpsr::Publisher<TestEvent> *_publisher;
};

#endif
