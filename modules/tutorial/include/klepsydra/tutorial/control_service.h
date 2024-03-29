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

#include <atomic>

#include <klepsydra/core/system_event.h>
#include <klepsydra/sdk/environment.h>
#include <klepsydra/sdk/publisher.h>
#include <klepsydra/sdk/service.h>
#include <klepsydra/sdk/subscriber.h>

#include "klepsydra/tutorial/battery_state.h"
#include "klepsydra/tutorial/temperature.h"

namespace kpsr {

float const BATTERY_THRESHOLD_LOW = 0.20;
float const BATTERY_THRESHOLD_LOW_START = 0.25;
double const TEMPERATURE_THRESHOLD_HIGH_STOP = 60.0;
double const TEMPERATURE_THRESHOLD_HIGH_START = 55.0;
double const TEMPERATURE_THRESHOLD_LOW_STOP = -15.0;
double const TEMPERATURE_THRESHOLD_LOW_START = -10.0;

class ControlService : public Service
{
public:
    ControlService(Environment *environment,
                   kpsr::Subscriber<kpsr::sensors::BatteryState> *batterySubscriber,
                   kpsr::Subscriber<kpsr::sensors::Temperature> *temperatureSubscriber,
                   kpsr::Publisher<kpsr::SystemEventData> *publisherSysEvent);

    virtual void start();

    virtual void stop();

    virtual void execute(){};

    float getBatteryValue();

    double getTemperatureValue();

private:
    void onBatteryReceived(const kpsr::sensors::BatteryState &batteryData);

    void onTemperatureReceived(const kpsr::sensors::Temperature &temperatureData);

    float _batteryValue;
    double _temperature;
    kpsr::SystemEventData _status;
    kpsr::SystemEventData _batteryOk;
    kpsr::SystemEventData _temperatureOk;
    kpsr::Subscriber<kpsr::sensors::BatteryState> *_batterySubscriber;
    kpsr::Subscriber<kpsr::sensors::Temperature> *_temperatureSubscriber;
    kpsr::Publisher<kpsr::SystemEventData> *_systemEventPublisher;
};
} // namespace kpsr
#endif // CONTROL_SERVICE_H
