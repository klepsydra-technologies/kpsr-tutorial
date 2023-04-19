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

#include <iostream>
#include <string>
#include <thread>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

#include "helper_functions.h"
#include "simple_event_publisher.h"
#include "slow_listener.h"
#include "test_event.h"

const int bufferSize{16};
const int poolSize{8};
const auto initializerFunction{[](TestEvent &event) { event.init(); }};
const auto eventClonerFunction{nullptr};
const int publishIterations = 3;

int main(int argc, char *argv[])
{
    resetInvocationsCounters();

    {
        std::cout << "Main thread " << std::this_thread::get_id() << std::endl;

        kpsr::high_performance::EventLoopMiddlewareProvider<bufferSize> eventLoop(nullptr);
        printInvocations("event loop creation");

        kpsr::Publisher<TestEvent> *kpsrPublisher =
            eventLoop.getPublisher<TestEvent>("TestEvent",
                                              poolSize,
                                              initializerFunction,
                                              eventClonerFunction);
        printInvocations("getPublisher");

        kpsr::Subscriber<TestEvent> *kpsrSubscriber = eventLoop.getSubscriber<TestEvent>(
            "TestEvent");
        printInvocations("getSubscriber");

        SimpleEventPublisher testPublisher(kpsrPublisher);
        printInvocations("testPublisher");

        SlowListener testListener(*kpsrSubscriber, "slowListener");
        printInvocations("testListener");

        eventLoop.start();
        std::thread concurrentDataUpdaterThread([&testPublisher]() {
            for (int i = 1; i <= publishIterations; i++) {
                TestEvent event(i, "hello");
                std::cout << "SimpleEventPublisher is about to receive TestEvent(" << event.id
                          << "," << event.message << ")\n\tat addr " << &event << "\n\ton thread "
                          << std::this_thread::get_id() << std::endl;
                testPublisher.run(event);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                event.id = -i;
                event.message = "bye";
            }
        });
        concurrentDataUpdaterThread.join();
        printInvocations("publishing");

        while (kpsrSubscriber->getSubscriptionStats("slowListener")->totalProcessed <
               publishIterations) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        printInvocations("slow listeners finished");

        eventLoop.stop();
        printInvocations("stopping event loop");
    }
    printInvocations("program finished");

    return 0;
}