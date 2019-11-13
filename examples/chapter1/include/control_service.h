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

#ifndef CONTROL_SERVICE_H
#define CONTROL_SERVICE_H

#include <klepsydra/core/service.h>
#include <klepsydra/core/subscriber.h>
#include <klepsydra/core/publisher.h>
#include <klepsydra/core/environment.h>
#include <klepsydra/core/system_event.h>
#include <iostream>

namespace kpsr {
class ControlService : public Service
{
public:
    ControlService(kpsr::Subscriber<float> * batterySubscriber,
                   kpsr::Publisher<kpsr::SystemEventData> * statusPublisher,
                   kpsr::Environment * env)
        : Service(env, "control_service")
        , _batterySubscriber(batterySubscriber)
        , _statusPublisher(statusPublisher)
        {}

protected:
    void start() {
        std::function<void(float)> listenerFunction = [this](const float event) {
            float lowThreshold;
            this->_environment->getPropertyFloat("low_threshold", lowThreshold);
            if (lowThreshold >= event) {
                _statusPublisher->publish(kpsr::SystemEventData::Stop);
            }
            float highThreshold;
            this->_environment->getPropertyFloat("high_threshold", highThreshold);
            if (highThreshold <= event) {
                _statusPublisher->publish(kpsr::SystemEventData::Start);
            }
         };
        _batterySubscriber->registerListener("batteryListener", listenerFunction);
    }

    void stop() {
        _batterySubscriber->removeListener("batteryListener");
    }

    void execute() {};
private:
    kpsr::Subscriber<float> * _batterySubscriber;
    kpsr::Publisher<kpsr::SystemEventData> * _statusPublisher;
};
}
#endif
