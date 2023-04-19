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
const int poolSize{4};
const auto initializerFunction{nullptr};
const auto eventClonerFunction{nullptr};
const int publishIterations = 6;

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
        auto eventLoopPublisher =
            static_cast<kpsr::high_performance::EventLoopPublisher<TestEvent, bufferSize> *>(
                kpsrPublisher);
        printInvocations("getPublisher");

        kpsr::Subscriber<TestEvent> *kpsrSubscriber = eventLoop.getSubscriber<TestEvent>(
            "TestEvent");
        printInvocations("getSubscriber");

        SimpleEventPublisher fastPublisher(kpsrPublisher);
        printInvocations("fastPublisher");

        SlowListener testListener(*kpsrSubscriber, "slowListener");
        printInvocations("testListener");

        eventLoop.start();
        int publishIter{1};
        do {
            std::thread fastThread([&fastPublisher, &publishIter]() {
                for (; publishIter <= publishIterations; publishIter++) {
                    TestEvent event(publishIter, "hello");
                    std::cout << "SimpleEventPublisher is about to receive TestEvent(" << event.id
                              << "," << event.message << ")\n\tat addr " << &event
                              << "\n\ton thread " << std::this_thread::get_id() << std::endl;
                    fastPublisher.run(event);
                }
            });
            fastThread.join();
            printInvocations("publishing");

            auto publishDiscarded = eventLoopPublisher->_discardedMessages;
            while (kpsrSubscriber->getSubscriptionStats("slowListener")->totalProcessed +
                       publishDiscarded <
                   publishIterations) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            printInvocations("slow listeners finished");

            std::cout << "\nDiscarded messages were " << publishDiscarded;
            if (publishDiscarded > 0) {
                std::cout << ", retrying";
            }
            std::cout << std::endl;

            publishIter -= publishDiscarded;
            eventLoopPublisher->_discardedMessages = 0;
        } while (publishIter <= publishIterations);

        eventLoop.stop();
        printInvocations("stopping event loop");
    }
    printInvocations("program finished");

    return 0;
}