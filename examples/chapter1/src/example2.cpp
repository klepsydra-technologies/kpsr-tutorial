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

#include "simple_publisher.h"
#include <iostream>
#include <klepsydra/high_performance/event_loop_middleware_provider.h>

int main()
{
    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);

    std::cout << "Main thread ID: " << std::this_thread::get_id() << std::endl;

    eventloop.start();
    eventloop.getSubscriber<std::string>("example2")
        ->registerListener("listener", [](const std::string &message) {
            std::cout << "Message received: " << message << std::endl;
            std::cout << "Eventloop (subscriber) thread ID: " << std::this_thread::get_id()
                      << std::endl;
        });
    std::this_thread::sleep_for(std::chrono::milliseconds(
        1)); // Ensures listener has been registered before publisher runs.

    SimplePublisher simplePublisher(
        eventloop.getPublisher<std::string>("example2", 0, nullptr, nullptr));
    simplePublisher.run();
    std::this_thread::sleep_for(std::chrono::milliseconds(
        1)); // Ensures published message ALWAYS reaches subscriber before the
             // listener is removed in next line.

    eventloop.getSubscriber<std::string>("example2")->removeListener("listener");
    eventloop.stop();
}
