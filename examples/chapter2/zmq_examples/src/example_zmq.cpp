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

#include <zmq.hpp>

#include <klepsydra/zmq_core/from_zmq_middleware_provider.h>
#include <klepsydra/zmq_core/to_zmq_middleware_provider.h>

#include <klepsydra/serialization/json_cereal_mapper.h>

#include "simple_publisher.h"
#include <klepsydra/high_performance/event_loop_middleware_provider.h>

int main(int argv, char **argc)
{
    // Set up zmq specific options.
    std::string serverUrl = "tcp://*:9001";
    std::string topic = "example1";
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind(serverUrl);
    std::string clientUrl = "tcp://localhost:9001";
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    // ZMQ set up over, now our simple example:

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQMiddlewareProvider(nullptr, publisher);
    kpsr::Publisher<std::string> *toZMQPublisher =
        toZMQMiddlewareProvider.getJsonToMiddlewareChannel<std::string>(topic, 0);

    SimplePublisher publisherS(toZMQPublisher);

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    kpsr::zmq_mdlw::FromZmqChannel<std::string> *_jsonFromZMQProvider =
        _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<std::string>(subscriber, 100);
    _jsonFromZMQProvider->start();

    kpsr::high_performance::EventLoopMiddlewareProvider<16> stringDataSafeQueueProvider(nullptr);
    stringDataSafeQueueProvider.start();

    _jsonFromZMQProvider->registerToTopic(
        topic, stringDataSafeQueueProvider.getPublisher<std::string>(topic, 0, nullptr, nullptr));

    stringDataSafeQueueProvider.getSubscriber<std::string>(topic)
        ->registerListener("example", [](std::string const &msg) {
            std::cout << "Received message: " << msg << std::endl;
        });
    std::this_thread::sleep_for(std::chrono::seconds(1));

    publisherS.run();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    stringDataSafeQueueProvider.getSubscriber<std::string>(topic)->removeListener("example");
    stringDataSafeQueueProvider.stop();
    _jsonFromZMQProvider->unregisterFromTopic(topic);
    _jsonFromZMQProvider->stop();
}
