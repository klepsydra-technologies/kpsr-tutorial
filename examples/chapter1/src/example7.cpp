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

#include <iostream>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <klepsydra/sdk/event_transform_forwarder.h>

int main()
{
    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);

    kpsr::Publisher<std::string> *stringPublisher = eventloop.getPublisher<std::string>("string",
                                                                                        0,
                                                                                        nullptr,
                                                                                        nullptr);
    kpsr::Subscriber<std::string> *stringSubscriber = eventloop.getSubscriber<std::string>(
        "string");

    kpsr::Publisher<int> *sizeStringPublisher = eventloop.getPublisher<int>("size_string",
                                                                            0,
                                                                            nullptr,
                                                                            nullptr);
    kpsr::Subscriber<int> *sizeStringSubscriber = eventloop.getSubscriber<int>("size_string");

    eventloop.start();

    {
        kpsr::EventTransformForwarder<std::string, int>
            eventTransformer([](const std::string &eventString,
                                int &transformed) { transformed = eventString.size(); },
                             sizeStringPublisher);

        stringSubscriber->registerListener("transformer",
                                           eventTransformer.forwarderListenerFunction);
        sizeStringSubscriber->registerListener("print_output_listener", [](const int &event) {
            std::cout << "The size of the string received is: " << event << std::endl;
        });

        std::thread stringPublisherThread([&stringPublisher]() {
            stringPublisher->publish("Example");
            stringPublisher->publish("Example about");
            stringPublisher->publish("Example about EventTransformForwarder");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        });

        stringPublisherThread.join();
    }

    stringSubscriber->removeListener("transformer");
    sizeStringSubscriber->removeListener("output_listener");
    eventloop.stop();
}