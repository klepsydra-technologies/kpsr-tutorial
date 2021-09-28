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
#include "simple_publisher.h"

#include <klepsydra/core/event_emitter_middleware_provider.h>
int main() {
    std::cout << "Main thread ID: " << std::this_thread::get_id() << std::endl;

    kpsr::EventEmitterMiddlewareProvider<std::string> provider(nullptr, "tutorial_app_api_example1", 0, nullptr, nullptr);

    SimplePublisher simplePublisher(provider.getPublisher());

    provider.getSubscriber()->registerListener("example1", [](const std::string & message) {
                                                               std::cout << "Message received: " << message << std::endl;
 							       std::cout << "Provider (subscriber) thread ID: " << std::this_thread::get_id() << std::endl;
                                                           }
        );

    simplePublisher.run();

    provider.getSubscriber()->removeListener("example1");

}
