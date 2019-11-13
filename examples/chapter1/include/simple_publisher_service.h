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
#ifndef SIMPLE_PUBLISHER_SERVICE_H
#define SIMPLE_PUBLISHER_SERVICE_H

#include <klepsydra/core/service.h>
#include <klepsydra/core/publisher.h>

class SimplePublisherService : public kpsr::Service
{
public:
    SimplePublisherService(kpsr::Publisher<std::string> * publisher)
        : kpsr::Service(nullptr, "publisher_service")
        , _publisher(publisher) {}

protected:
    void start() {}; // Empty on purpose

    void stop() {}; // Empty on purpose

    void execute () {
        _publisher->publish("Hello world");
    }

private:

    kpsr::Publisher<std::string>* _publisher;
};
#endif // SIMPLE_PUBLISHER_SERVICE_H
