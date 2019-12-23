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

#include <zmq.hpp>

#include <klepsydra/zmq_core/to_zmq_middleware_provider.h>
#include <klepsydra/zmq_core/from_zmq_middleware_provider.h>

#include <klepsydra/serialization/json_cereal_mapper.h>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include "simple_publisher.h"

int main (int argv, char ** argc) {

    // Set up zmq specific options.
    std::string serverUrl = "tcp://*:9001";
    std::string topic = "example1";
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    std::string clientUrl = "tcp://localhost:9001";
    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    // ZMQ set up over, now our simple example:

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQMiddlewareProvider(nullptr, publisher);
    kpsr::Publisher<std::string> * toZMQPublisher = toZMQMiddlewareProvider.getJsonToMiddlewareChannel<std::string>(topic, 0);
    
    SimplePublisher publisherS(toZMQPublisher);
    
    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    kpsr::zmq_mdlw::FromZmqChannel<std::string> * _jsonFromZMQProvider = _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<std::string>(subscriber, 100);
    _jsonFromZMQProvider->start();
    
    kpsr::high_performance::EventLoopMiddlewareProvider<16> stringDataSafeQueueProvider(nullptr);
    stringDataSafeQueueProvider.start();

    _jsonFromZMQProvider->registerToTopic(topic, stringDataSafeQueueProvider.getPublisher<std::string>(topic, 0, nullptr, nullptr));

    stringDataSafeQueueProvider.getSubscriber<std::string>(topic)->registerListener(
        "example", [](std::string const &msg) {
                       std::cout << "Received message: " << msg << std::endl; });
    std::this_thread::sleep_for(std::chrono::seconds(1));

    publisherS.run();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    stringDataSafeQueueProvider.getSubscriber<std::string>(topic)->removeListener("example");
    stringDataSafeQueueProvider.stop();
    _jsonFromZMQProvider->unregisterFromTopic(topic);
    _jsonFromZMQProvider->stop();

}
