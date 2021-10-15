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

#include "control_service_ros.h"

kpsr::ControlServiceRos::ControlServiceRos(
    Environment *environment,
    kpsr::Subscriber<sensor_msgs::BatteryState> *batterySubscriber,
    kpsr::Subscriber<sensor_msgs::Temperature> *temperatureSubscriber,
    kpsr::Publisher<kpsr::SystemEventData> *publisherSysEvent)
    : Service(environment, "control_service")
    , _status(kpsr::SystemEventData::Idle)
    , _batteryOk(kpsr::SystemEventData::Idle)
    , _temperatureOk(kpsr::SystemEventData::Idle)
    , _batterySubscriber(batterySubscriber)
    , _temperatureSubscriber(temperatureSubscriber)
    , _systemEventPublisher(publisherSysEvent)
{}

void kpsr::ControlServiceRos::start()
{
    std::function<void(sensor_msgs::BatteryState)> batteryListenerFn =
        std::bind(&ControlServiceRos::onBatteryReceived, this, std::placeholders::_1);
    _batterySubscriber->registerListener("batteryCacheListener", batteryListenerFn);

    std::function<void(sensor_msgs::Temperature)> temperatureListenerFn =
        std::bind(&ControlServiceRos::onTemperatureReceived, this, std::placeholders::_1);
    _temperatureSubscriber->registerListener("temperatureCacheListener", temperatureListenerFn);
}

void kpsr::ControlServiceRos::stop()
{
    _batterySubscriber->removeListener("batteryCacheListener");
    _temperatureSubscriber->removeListener("temperatureCacheListener");
}

float kpsr::ControlServiceRos::getBatteryValue()
{
    return _batteryValue;
}

double kpsr::ControlServiceRos::getTemperatureValue()
{
    return _temperature;
}

void kpsr::ControlServiceRos::onBatteryReceived(const sensor_msgs::BatteryState &batteryData)
{
    _batteryValue = batteryData.percentage;
    if ((kpsr::BATTERY_THRESHOLD_LOW >= _batteryValue) && (_status != kpsr::SystemEventData::Stop)) {
        _batteryOk = kpsr::SystemEventData::Stop;
        _status = kpsr::SystemEventData::Stop;
        _systemEventPublisher->publish(_status);
    } else if ((kpsr::BATTERY_THRESHOLD_LOW_START <= _batteryValue) &&
               (_status != kpsr::SystemEventData::Start)) {
        _batteryOk = kpsr::SystemEventData::Start;
        if (_temperatureOk != kpsr::SystemEventData::Stop) {
            // This ensures that start is sent only when both battery
            // and temperature are in range.
            _status = kpsr::SystemEventData::Start;
            _systemEventPublisher->publish(_status);
        }
    }
}

void kpsr::ControlServiceRos::onTemperatureReceived(const sensor_msgs::Temperature &temperatureData)
{
    _temperature = temperatureData.temperature;

    bool isStop = (kpsr::TEMPERATURE_THRESHOLD_HIGH_STOP <= _temperature) ||
                  (kpsr::TEMPERATURE_THRESHOLD_LOW_STOP >= _temperature);

    bool isStart = (kpsr::TEMPERATURE_THRESHOLD_HIGH_START >= _temperature) &&
                   (kpsr::TEMPERATURE_THRESHOLD_LOW_START <= _temperature);

    if (isStop && (_status != kpsr::SystemEventData::Stop)) {
        _temperatureOk = kpsr::SystemEventData::Stop;
        _status = kpsr::SystemEventData::Stop;
        _systemEventPublisher->publish(_status);
    } else if (isStart && (_status != kpsr::SystemEventData::Start)) {
        _temperatureOk = kpsr::SystemEventData::Start;
        if (_batteryOk != kpsr::SystemEventData::Stop) {
            _status = kpsr::SystemEventData::Start;
            _systemEventPublisher->publish(_status);
        }
    }
}
