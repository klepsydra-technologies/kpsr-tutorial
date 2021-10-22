/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*****************************************************************************/
#include <iostream>

#include <klepsydra/core/event_transform_forwarder.h>
#include <klepsydra/high_performance/event_loop_middleware_provider.h>

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