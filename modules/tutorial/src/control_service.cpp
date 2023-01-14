// Copyright 2023 Klepsydra Technologies AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <klepsydra/tutorial/control_service.h>

kpsr::ControlService::ControlService(
    Environment *environment,
    kpsr::Subscriber<kpsr::sensors::BatteryState> *batterySubscriber,
    kpsr::Subscriber<kpsr::sensors::Temperature> *temperatureSubscriber,
    kpsr::Publisher<kpsr::SystemEventData> *publisherSysEvent)
    : Service(environment, "control_service")
    , _status(kpsr::SystemEventData::Idle)
    , _batteryOk(kpsr::SystemEventData::Idle)
    , _temperatureOk(kpsr::SystemEventData::Idle)
    , _batterySubscriber(batterySubscriber)
    , _temperatureSubscriber(temperatureSubscriber)
    , _systemEventPublisher(publisherSysEvent)
{}

void kpsr::ControlService::start()
{
    std::function<void(kpsr::sensors::BatteryState)> batteryListenerFn =
        std::bind(&ControlService::onBatteryReceived, this, std::placeholders::_1);
    _batterySubscriber->registerListener("batteryCacheListener", batteryListenerFn);

    std::function<void(kpsr::sensors::Temperature)> temperatureListenerFn =
        std::bind(&ControlService::onTemperatureReceived, this, std::placeholders::_1);
    _temperatureSubscriber->registerListener("temperatureCacheListener", temperatureListenerFn);
}

void kpsr::ControlService::stop()
{
    _batterySubscriber->removeListener("batteryCacheListener");
    _temperatureSubscriber->removeListener("temperatureCacheListener");
}

float kpsr::ControlService::getBatteryValue()
{
    return _batteryValue;
}

double kpsr::ControlService::getTemperatureValue()
{
    return _temperature;
}

void kpsr::ControlService::onBatteryReceived(const kpsr::sensors::BatteryState &batteryData)
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

void kpsr::ControlService::onTemperatureReceived(const kpsr::sensors::Temperature &temperatureData)
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
