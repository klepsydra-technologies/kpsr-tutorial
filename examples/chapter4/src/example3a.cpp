#include <iostream>
#include <thread>

#include <klepsydra/state_machine/sm_factory_impl.h>
#include <klepsydra/state_machine/state_machine_factory.h>

#include <spdlog/spdlog.h>

#include "simple_state_machine.h"

struct CoutObserver
{
    std::string _currentState;
    void updateCurrentState(const std::string &currentState, bool stateChanged)
    {
        if ("exampleStateMachine:st1" == currentState) {
            std::cout << "CoutObserver::First state " << std::endl;
        } else if ("exampleStateMachine:st2" == currentState) {
            std::cout << "CoutObserver::state 2 reached" << std::endl;
        }
        this->_currentState = currentState;
    }
};

struct LastStateObserver
{
    std::string _currentState;
    void updateCurrentState(const std::string &currentState, bool stateChanged)
    {
        if ("exampleStateMachine:st3" == currentState) {
            if (stateChanged) {
                std::cout
                    << "LastStateObserver: This is the last state. We just changed to this state."
                    << std::endl;
            } else {
                std::cout << "LastStateObserver: This is the last state" << std::endl;
            }
        } else {
            std::cout << "LastStateObserver: Not the last state" << std::endl;
        }
        this->_currentState = currentState;
    }
};

int main()
{
    SimpleStateMachine exampleStateMachine;

    kpsr::fsm::SMFactoryImpl smFactory;
    auto stateMachine = smFactory.createStateMachine(exampleStateMachine.configStateMachine);
    CoutObserver smObserver;
    LastStateObserver lastObserver;
    stateMachine->registerObserver(std::bind(&CoutObserver::updateCurrentState,
                                             smObserver,
                                             std::placeholders::_1,
                                             std::placeholders::_2));
    stateMachine->registerObserver(std::bind(&LastStateObserver::updateCurrentState,
                                             lastObserver,
                                             std::placeholders::_1,
                                             std::placeholders::_2));
    stateMachine->start();

    std::thread updateThread([stateMachine]() {
        for (int i = 0; i < 10; i++) {
            spdlog::info("Updating state machine");
            stateMachine->update();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    });
    std::thread enqueueThread([&stateMachine]() {
        spdlog::info("Enqueuing event1");
        stateMachine->enqueueEvent("event1"); // transition to st2 if allowed
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        spdlog::info("Enqueuing event2");
        stateMachine->enqueueEvent("event2"); // transition to st3 if allowed
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        spdlog::info("Enqueuing event3");
        stateMachine->enqueueEvent("event3"); // transition to st1 if allowed
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        spdlog::info("Enqueuing event2");
        stateMachine->enqueueEvent("event2"); // transition to st3 if allowed
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        spdlog::info("Enqueuing event3");
        stateMachine->enqueueEvent("event3"); // transition to st1 if allowed
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        spdlog::info("Enqueuing event1");
        stateMachine->enqueueEvent("event1"); // transition to st2 if allowed
    });

    enqueueThread.join();
    updateThread.join();
    stateMachine->stop();
}
