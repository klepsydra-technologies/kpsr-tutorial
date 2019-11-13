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
#ifndef SIMPLE_SUBSCRIBER_SERVICE_H
#define SIMPLE_SUBSCRIBER_SERVICE_H

#include <klepsydra/core/service.h>
#include <klepsydra/core/subscriber.h>

#include <iostream>

class SimpleSubscriberService : public kpsr::Service {
public:
    SimpleSubscriberService(kpsr::Subscriber<std::string> * subscriber)
    : Service(nullptr, "subcriber_service")
    ,_subscriber(subscriber) {}

protected:
    void start() override {
        _subscriber->registerListener("subscriber_service", [](const std::string & message) {
            std::cout << "Message received: " << message << std::endl;
        });
    }

    void stop() override {
        _subscriber->removeListener("subscriber_service");
    }

    void execute() {} // Empty on purpose.

private:
    kpsr::Subscriber<std::string> * _subscriber;
};
#endif // SIMPLE_SUBSCRIBER_SERVICE_H
