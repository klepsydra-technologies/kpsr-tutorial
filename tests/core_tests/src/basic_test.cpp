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

#include <stdio.h>
#include <thread>

#include "gtest/gtest.h"

#include <klepsydra/tutorial/control_service.h>

#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/event_emitter_publisher.h>
#include <klepsydra/core/event_emitter_subscriber.h>

#include <klepsydra/mem_core/mem_env.h>

#include "klepsydra/tutorial/battery_state.h"
#include "klepsydra/tutorial/temperature.h"

class ControlServiceTest : public ::testing::Test
{
protected:
    std::string batteryTopicName = "testBattery";
    std::string temperatureTopicName = "testTemperature";
    std::string statusTopicName = "testStatus";

    ControlServiceTest()
        : eventEmitter()
        , batterySubscriber(nullptr, eventEmitter, batteryTopicName)
        , temperatureSubscriber(nullptr, eventEmitter, temperatureTopicName)
        , statusPublisher(nullptr, statusTopicName, eventEmitter, 0, nullptr, nullptr)
        , sut(nullptr, &batterySubscriber, &temperatureSubscriber, &statusPublisher)
    {}

    kpsr::EventEmitter eventEmitter;
    kpsr::EventEmitterSubscriber<kpsr::sensors::BatteryState> batterySubscriber;

    kpsr::EventEmitterSubscriber<kpsr::sensors::Temperature> temperatureSubscriber;

    kpsr::EventEmitterPublisher<kpsr::SystemEventData> statusPublisher;
    kpsr::ControlService sut;
};

TEST_F(ControlServiceTest, batteryValueTest)
{
    kpsr::EventEmitterPublisher<kpsr::sensors::BatteryState> batteryPublisher(nullptr,
                                                                              batteryTopicName,
                                                                              eventEmitter,
                                                                              0,
                                                                              nullptr,
                                                                              nullptr);

    kpsr::sensors::BatteryState testBatteryValue;
    testBatteryValue.percentage = 0.55;

    sut.start();
    batteryPublisher.publish(testBatteryValue);

    float receivedValue = sut.getBatteryValue();
    sut.stop();

    ASSERT_EQ(receivedValue, testBatteryValue.percentage);
}

TEST_F(ControlServiceTest, temperatureTest)
{
    kpsr::EventEmitterPublisher<kpsr::sensors::Temperature>
        temperaturePublisher(nullptr, temperatureTopicName, eventEmitter, 0, nullptr, nullptr);

    kpsr::sensors::Temperature testTemperatureValue;
    testTemperatureValue.temperature = 20;

    sut.start();
    temperaturePublisher.publish(testTemperatureValue);

    float receivedValue = sut.getTemperatureValue();
    sut.stop();

    ASSERT_EQ(receivedValue, testTemperatureValue.temperature);
}

TEST_F(ControlServiceTest, systemEventStartTest)
{
    kpsr::EventEmitterPublisher<kpsr::sensors::BatteryState> batteryPublisher(nullptr,
                                                                              batteryTopicName,
                                                                              eventEmitter,
                                                                              0,
                                                                              nullptr,
                                                                              nullptr);

    kpsr::EventEmitterSubscriber<kpsr::SystemEventData> statusSubscriber(nullptr,
                                                                         eventEmitter,
                                                                         statusTopicName);
    kpsr::mem::TestCacheListener<kpsr::SystemEventData> cacheListener(10);
    statusSubscriber.registerListener("cacheListener", cacheListener.cacheListenerFunction);

    kpsr::sensors::BatteryState testBatteryValue;
    testBatteryValue.percentage = 0.55;

    sut.start();
    batteryPublisher.publish(testBatteryValue);
    kpsr::SystemEventData publishedStatus = *cacheListener.getLastReceivedEvent().get();

    ASSERT_EQ(kpsr::SystemEventData::Start, publishedStatus);
    sut.stop();
}

TEST_F(ControlServiceTest, systemEventNoDoublePublishTest)
{
    kpsr::EventEmitterPublisher<kpsr::sensors::BatteryState> batteryPublisher(nullptr,
                                                                              batteryTopicName,
                                                                              eventEmitter,
                                                                              0,
                                                                              nullptr,
                                                                              nullptr);

    kpsr::EventEmitterSubscriber<kpsr::SystemEventData> statusSubscriber(nullptr,
                                                                         eventEmitter,
                                                                         statusTopicName);
    kpsr::mem::TestCacheListener<kpsr::SystemEventData> cacheListener(10);
    statusSubscriber.registerListener("cacheListener", cacheListener.cacheListenerFunction);

    kpsr::sensors::BatteryState testBatteryValue;
    testBatteryValue.percentage = 0.55;

    sut.start();
    batteryPublisher.publish(testBatteryValue);
    testBatteryValue.percentage = 0.65;
    batteryPublisher.publish(testBatteryValue);
    kpsr::SystemEventData publishedStatus = *cacheListener.getLastReceivedEvent().get();

    ASSERT_EQ(kpsr::SystemEventData::Start, publishedStatus);
    ASSERT_EQ(1, cacheListener.counter);
    sut.stop();
}

TEST_F(ControlServiceTest, systemEventTemperatureLimitsTest)
{
    kpsr::EventEmitterPublisher<kpsr::sensors::Temperature>
        temperaturePublisher(nullptr, temperatureTopicName, eventEmitter, 0, nullptr, nullptr);

    kpsr::EventEmitterSubscriber<kpsr::SystemEventData> statusSubscriber(nullptr,
                                                                         eventEmitter,
                                                                         statusTopicName);
    kpsr::mem::TestCacheListener<kpsr::SystemEventData> cacheListener(10);
    statusSubscriber.registerListener("cacheListener", cacheListener.cacheListenerFunction);

    kpsr::sensors::Temperature testTemperatureValue;
    testTemperatureValue.temperature = -20;

    sut.start();
    temperaturePublisher.publish(testTemperatureValue);

    kpsr::SystemEventData publishedStatus = *cacheListener.getLastReceivedEvent().get();

    ASSERT_EQ(kpsr::SystemEventData::Stop, publishedStatus);

    testTemperatureValue.temperature = 0;
    temperaturePublisher.publish(testTemperatureValue);

    publishedStatus = *cacheListener.getLastReceivedEvent().get();
    ASSERT_EQ(kpsr::SystemEventData::Start, publishedStatus);

    testTemperatureValue.temperature = 56;
    temperaturePublisher.publish(testTemperatureValue);

    publishedStatus = *cacheListener.getLastReceivedEvent().get();
    ASSERT_EQ(kpsr::SystemEventData::Start, publishedStatus);

    testTemperatureValue.temperature = 66;
    temperaturePublisher.publish(testTemperatureValue);

    publishedStatus = *cacheListener.getLastReceivedEvent().get();
    ASSERT_EQ(kpsr::SystemEventData::Stop, publishedStatus);

    sut.stop();
}

TEST(ControlServiceEnvironmentTest, checkEnvironmentPropsTest)
{
    std::string batteryTopicName = "testBattery";
    std::string temperatureTopicName = "testTemperature";
    std::string statusTopicName = "testStatus";

    kpsr::EventEmitter eventEmitter;

    kpsr::mem::MemEnv testEnv;
    float const TEMPERATURE_THRESHOLD_HIGH_STOP = 60.0;
    float const TEMPERATURE_THRESHOLD_HIGH_START = 55.0;
    float const TEMPERATURE_THRESHOLD_LOW_STOP = -15.0;
    float const TEMPERATURE_THRESHOLD_LOW_START = -10.0;
    float const BATTERY_THRESHOLD_LOW = 0.20;
    float const BATTERY_THRESHOLD_LOW_START = 0.25;

    // Set environment properties.
    testEnv.setPropertyFloat("TEMPERATURE_THRESHOLD_HIGH_STOP", TEMPERATURE_THRESHOLD_HIGH_STOP);
    testEnv.setPropertyFloat("TEMPERATURE_THRESHOLD_HIGH_START", TEMPERATURE_THRESHOLD_HIGH_START);
    testEnv.setPropertyFloat("TEMPERATURE_THRESHOLD_LOW_STOP", TEMPERATURE_THRESHOLD_LOW_STOP);
    testEnv.setPropertyFloat("TEMPERATURE_THRESHOLD_LOW_START", TEMPERATURE_THRESHOLD_LOW_START);
    testEnv.setPropertyFloat("BATTERY_THRESHOLD_LOW", BATTERY_THRESHOLD_LOW);
    testEnv.setPropertyFloat("BATTERY_THRESHOLD_LOW_START", BATTERY_THRESHOLD_LOW_START);

    kpsr::EventEmitterSubscriber<kpsr::sensors::BatteryState> batterySubscriber(nullptr,
                                                                                eventEmitter,
                                                                                batteryTopicName);
    kpsr::EventEmitterSubscriber<kpsr::sensors::Temperature>
        temperatureSubscriber(nullptr, eventEmitter, temperatureTopicName);
    kpsr::EventEmitterPublisher<kpsr::SystemEventData> statusPublisher(nullptr,
                                                                       statusTopicName,
                                                                       eventEmitter,
                                                                       0,
                                                                       nullptr,
                                                                       nullptr);
    kpsr::ControlService testClass(&testEnv,
                                   &batterySubscriber,
                                   &temperatureSubscriber,
                                   &statusPublisher);

    float value;
    testClass._environment->getPropertyFloat("TEMPERATURE_THRESHOLD_HIGH_STOP", value);
    ASSERT_EQ(TEMPERATURE_THRESHOLD_HIGH_STOP, value);

    testClass._environment->getPropertyFloat("TEMPERATURE_THRESHOLD_HIGH_START", value);
    ASSERT_EQ(TEMPERATURE_THRESHOLD_HIGH_START, value);
    testClass._environment->getPropertyFloat("TEMPERATURE_THRESHOLD_LOW_STOP", value);
    ASSERT_EQ(TEMPERATURE_THRESHOLD_LOW_STOP, value);
    testClass._environment->getPropertyFloat("TEMPERATURE_THRESHOLD_LOW_START", value);
    ASSERT_EQ(TEMPERATURE_THRESHOLD_LOW_START, value);
    testClass._environment->getPropertyFloat("BATTERY_THRESHOLD_LOW", value);
    ASSERT_EQ(BATTERY_THRESHOLD_LOW, value);
    testClass._environment->getPropertyFloat("BATTERY_THRESHOLD_LOW_START", value);
    ASSERT_EQ(BATTERY_THRESHOLD_LOW_START, value);
}
