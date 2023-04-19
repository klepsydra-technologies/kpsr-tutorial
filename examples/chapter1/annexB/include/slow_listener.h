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

#ifndef SLOW_LISTENER_H
#define SLOW_LISTENER_H

#include <iostream>
#include <sstream>

#include <klepsydra/core/subscriber.h>

class SlowListener
{
public:
    SlowListener(kpsr::Subscriber<TestEvent> &subscriber, std::string listenerName)
        : _subscriber(subscriber)
    {
        _subscriber.registerListener(listenerName, [listenerName](const TestEvent &event) {
            std::stringstream msgFlushed;
            msgFlushed << "SlowListener (" << listenerName << ") received TestEvent(" << event.id
                       << "," << event.message << ")\n\tat addr " << &event << "\n\ton thread "
                       << std::this_thread::get_id() << std::endl;
            std::cout << msgFlushed.str();
            msgFlushed.str(std::string());

            std::this_thread::sleep_for(std::chrono::seconds(1));

            msgFlushed << "SlowListener (" << listenerName << ") finished, now TestEvent("
                       << event.id << "," << event.message << ")" << std::endl;
            std::cout << msgFlushed.str();
        });
    }

    ~SlowListener() { _subscriber.removeListener("slowListener"); }

private:
    kpsr::Subscriber<TestEvent> &_subscriber;
};

#endif
