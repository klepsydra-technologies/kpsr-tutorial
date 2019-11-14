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
#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include "simple_publisher.h"

int main() {
    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);

    eventloop.start();
    eventloop.getSubscriber<std::string>("example2")->registerListener("listener",
                                                             [](const std::string & message) {
                                                                 std::cout << "Message received: " << message << std::endl;
                                                             });
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Ensures listener has been registered before publisher runs.

    SimplePublisher simplePublisher(eventloop.getPublisher<std::string>("example2", 0, nullptr, nullptr));
    simplePublisher.run();
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Ensures published message ALWAYS reaches subscriber before the listener is removed in next line.

    eventloop.getSubscriber<std::string>("example2")->removeListener("listener");
    eventloop.stop();
}
