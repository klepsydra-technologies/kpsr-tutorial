#include <iostream>
#include <thread>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <klepsydra/state_machine/sm_factory_impl.h>
#include <klepsydra/state_machine/state_machine_factory.h>
#include <klepsydra/state_machine/state_machine_listener.h>

#include <spdlog/spdlog.h>

#include "simple_state_machine.h"

int main()
{
    const std::string batteryTopicName = "testBattery";
    const std::string temperatureTopicName = "testTemperature";

    static const int BUFFER_SIZE = 8; // Must be a power of 2
    SimpleStateMachine exampleStateMachine;

    kpsr::fsm::SMFactoryImpl smFactory;
    auto stateMachine = smFactory.createStateMachine(exampleStateMachine.configStateMachine);

    kpsr::fsm::StateMachineListener smListener(exampleStateMachine.configStateMachine.id);

    kpsr::high_performance::EventLoopMiddlewareProvider<BUFFER_SIZE> provider(nullptr);
    provider.start();

    // st1 == READY state
    // st2 == IDLE state
    // st3 == ACTIVE state
    auto batteryStateSubscriber = provider.getSubscriber<float>(batteryTopicName);
    auto batteryStatePublisher = provider.getPublisher<float>(batteryTopicName, 0, nullptr, nullptr);
    auto temperaturePublisher = provider.getPublisher<float>(temperatureTopicName,
                                                             0,
                                                             nullptr,
                                                             nullptr);
    auto temperatureSubscriber = provider.getSubscriber<float>(temperatureTopicName);

    batteryStateSubscriber
        ->registerListener("battery_level_events", [&stateMachine](const float &batteryLevel) {
            if (batteryLevel <= 0.2) {
                stateMachine->enqueueEvent("event1"); // Low battery level.
            } else if (batteryLevel >= 0.6) {
                stateMachine->enqueueEvent("event2"); // Full battery
            } else {
                stateMachine->enqueueEvent("event3"); // Medium battery
            }
            stateMachine->update();
        });

    smListener.addAction("st1", [](const std::string &currentState) {
        std::cout << "sensor on and ready" << std::endl;
    });
    smListener.addAction("st2", [](const std::string &currentState) {
        std::cout << " Low battery" << std::endl;
    });
    smListener.addPeriodicAction("st2", [](const std::string &currentState) {
        std::cout << " Waiting for sufficient charge before it can be used" << std::endl;
    });

    smListener.addOneOffAction("st3", [](const std::string &currentState) {
        std::cout << " Calibrating motors" << std::endl;
    });
    smListener.addAction("st3", [](const std::string &currentState) {
        std::cout << "Ready to measure temperature" << std::endl;
    });

    smListener.addPeriodicAction("st3",
                                 [&temperaturePublisher,
                                  &temperatureTopicName](const std::string &currentState) {
                                     temperaturePublisher->publish(22.0f);
                                     std::cout << "Publishing dummy temperature" << std::endl;
                                 });
    stateMachine->registerObserver(smListener.getObserverFunc());
    stateMachine->start();

    std::thread batterySensorThread([&batteryStatePublisher, &batteryTopicName] {
        // first battery discharges
        float percentage = 0.7;
        for (int i = 0; i < 30; i++) {
            percentage -= (0.02);
            batteryStatePublisher->publish(percentage);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        // charging started but interrupted
        for (int i = 0; i < 15; i++) {
            percentage += (0.01);
            batteryStatePublisher->publish(percentage);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        // Discharges again
        for (int i = 0; i < 10; i++) {
            percentage -= (0.02);
            batteryStatePublisher->publish(percentage);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        // full recharge
        for (int i = 0; i < 40; i++) {
            percentage += (0.02);
            batteryStatePublisher->publish(percentage);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    });

    temperatureSubscriber->registerListener("temperature receiver", [](const float &temp) {
        std::cout << "Current temperature is " << temp << std::endl;
    });

    batterySensorThread.join();
    stateMachine->stop();
    spdlog::info("Stopping program");
    provider.stop();
}
