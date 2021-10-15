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

#include <atomic>

#include <klepsydra/core/environment.h>
#include <klepsydra/core/publisher.h>
#include <klepsydra/core/service.h>
#include <klepsydra/core/subscriber.h>
#include <klepsydra/core/system_event.h>

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
