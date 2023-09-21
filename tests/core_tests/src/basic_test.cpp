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
        : eventEmitterBattery(
              kpsr::EventEmitterFactory::createEventEmitter<
                  std::shared_ptr<const kpsr::sensors::BatteryState>>(kpsr::EventEmitterType::SAFE))
        , batterySubscriber(nullptr, eventEmitterBattery, batteryTopicName)
        , eventEmitterTemperature(
              kpsr::EventEmitterFactory::createEventEmitter<
                  std::shared_ptr<const kpsr::sensors::Temperature>>(kpsr::EventEmitterType::SAFE))
        , temperatureSubscriber(nullptr, eventEmitterTemperature, temperatureTopicName)
        , eventEmitterSys(
              kpsr::EventEmitterFactory::createEventEmitter<
                  std::shared_ptr<const kpsr::SystemEventData>>(kpsr::EventEmitterType::SAFE))
        , statusPublisher(nullptr, statusTopicName, eventEmitterSys, 0, nullptr, nullptr)
        , sut(nullptr, &batterySubscriber, &temperatureSubscriber, &statusPublisher)
    {}

    std::shared_ptr<kpsr::EventEmitterInterface<std::shared_ptr<const kpsr::sensors::BatteryState>>>
        eventEmitterBattery;
    kpsr::EventEmitterSubscriber<kpsr::sensors::BatteryState> batterySubscriber;

    std::shared_ptr<kpsr::EventEmitterInterface<std::shared_ptr<const kpsr::sensors::Temperature>>>
        eventEmitterTemperature;
    kpsr::EventEmitterSubscriber<kpsr::sensors::Temperature> temperatureSubscriber;

    std::shared_ptr<kpsr::EventEmitterInterface<std::shared_ptr<const kpsr::SystemEventData>>>
        eventEmitterSys;
    kpsr::EventEmitterPublisher<kpsr::SystemEventData> statusPublisher;
    kpsr::ControlService sut;
};

TEST_F(ControlServiceTest, batteryValueTest)
{
    kpsr::EventEmitterPublisher<kpsr::sensors::BatteryState> batteryPublisher(nullptr,
                                                                              batteryTopicName,
                                                                              eventEmitterBattery,
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
    kpsr::EventEmitterPublisher<kpsr::sensors::Temperature> temperaturePublisher(
        nullptr, temperatureTopicName, eventEmitterTemperature, 0, nullptr, nullptr);

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
                                                                              eventEmitterBattery,
                                                                              0,
                                                                              nullptr,
                                                                              nullptr);

    kpsr::EventEmitterSubscriber<kpsr::SystemEventData> statusSubscriber(nullptr,
                                                                         eventEmitterSys,
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
                                                                              eventEmitterBattery,
                                                                              0,
                                                                              nullptr,
                                                                              nullptr);

    kpsr::EventEmitterSubscriber<kpsr::SystemEventData> statusSubscriber(nullptr,
                                                                         eventEmitterSys,
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
    kpsr::EventEmitterPublisher<kpsr::sensors::Temperature> temperaturePublisher(
        nullptr, temperatureTopicName, eventEmitterTemperature, 0, nullptr, nullptr);

    kpsr::EventEmitterSubscriber<kpsr::SystemEventData> statusSubscriber(nullptr,
                                                                         eventEmitterSys,
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

    std::shared_ptr<kpsr::EventEmitterInterface<std::shared_ptr<const kpsr::sensors::BatteryState>>>
        eventEmitterBattery = kpsr::EventEmitterFactory::createEventEmitter<
            std::shared_ptr<const kpsr::sensors::BatteryState>>(kpsr::EventEmitterType::SAFE);
    std::shared_ptr<kpsr::EventEmitterInterface<std::shared_ptr<const kpsr::sensors::Temperature>>>
        eventEmitterTemperature = kpsr::EventEmitterFactory::createEventEmitter<
            std::shared_ptr<const kpsr::sensors::Temperature>>(kpsr::EventEmitterType::SAFE);
    std::shared_ptr<kpsr::EventEmitterInterface<std::shared_ptr<const kpsr::SystemEventData>>>
        eventEmitterSys = kpsr::EventEmitterFactory::createEventEmitter<
            std::shared_ptr<const kpsr::SystemEventData>>(kpsr::EventEmitterType::SAFE);

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
                                                                                eventEmitterBattery,
                                                                                batteryTopicName);
    kpsr::EventEmitterSubscriber<kpsr::sensors::Temperature>
        temperatureSubscriber(nullptr, eventEmitterTemperature, temperatureTopicName);
    kpsr::EventEmitterPublisher<kpsr::SystemEventData> statusPublisher(nullptr,
                                                                       statusTopicName,
                                                                       eventEmitterSys,
                                                                       0,
                                                                       nullptr,
                                                                       nullptr);
    kpsr::ControlService testClass(&testEnv,
                                   &batterySubscriber,
                                   &temperatureSubscriber,
                                   &statusPublisher);

    float value;
    testClass.environment->getPropertyFloat("TEMPERATURE_THRESHOLD_HIGH_STOP", value);
    ASSERT_EQ(TEMPERATURE_THRESHOLD_HIGH_STOP, value);

    testClass.environment->getPropertyFloat("TEMPERATURE_THRESHOLD_HIGH_START", value);
    ASSERT_EQ(TEMPERATURE_THRESHOLD_HIGH_START, value);
    testClass.environment->getPropertyFloat("TEMPERATURE_THRESHOLD_LOW_STOP", value);
    ASSERT_EQ(TEMPERATURE_THRESHOLD_LOW_STOP, value);
    testClass.environment->getPropertyFloat("TEMPERATURE_THRESHOLD_LOW_START", value);
    ASSERT_EQ(TEMPERATURE_THRESHOLD_LOW_START, value);
    testClass.environment->getPropertyFloat("BATTERY_THRESHOLD_LOW", value);
    ASSERT_EQ(BATTERY_THRESHOLD_LOW, value);
    testClass.environment->getPropertyFloat("BATTERY_THRESHOLD_LOW_START", value);
    ASSERT_EQ(BATTERY_THRESHOLD_LOW_START, value);
}
