/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CONTROL_SERVICE_H
#define CONTROL_SERVICE_H

#include <iostream>
#include <klepsydra/core/environment.h>
#include <klepsydra/core/publisher.h>
#include <klepsydra/core/service.h>
#include <klepsydra/core/subscriber.h>
#include <klepsydra/core/system_event.h>

namespace kpsr {
class ControlService : public Service
{
public:
    ControlService(kpsr::Subscriber<float> *batterySubscriber,
                   kpsr::Publisher<kpsr::SystemEventData> *statusPublisher,
                   kpsr::Environment *env)
        : Service(env, "control_service")
        , _batterySubscriber(batterySubscriber)
        , _statusPublisher(statusPublisher)
    {}

protected:
    void start()
    {
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

    void stop() { _batterySubscriber->removeListener("batteryListener"); }

    void execute(){};

private:
    kpsr::Subscriber<float> *_batterySubscriber;
    kpsr::Publisher<kpsr::SystemEventData> *_statusPublisher;
};
} // namespace kpsr
#endif
