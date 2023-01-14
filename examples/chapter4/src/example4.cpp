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

#include <iostream>
#include <thread>

#include <klepsydra/state_machine/sm_factory_impl.h>
#include <klepsydra/state_machine/state_machine_factory.h>
#include <klepsydra/state_machine/state_machine_listener.h>

#include <spdlog/spdlog.h>

#include "simple_state_machine.h"

struct LastStateObserver
{
    std::string _currentState;
    void updateCurrentState(const std::string &currentState, bool stateChanged)
    {
        if ("exampleStateMachine:st3" != currentState) {
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

    kpsr::fsm::StateMachineListener smListener(exampleStateMachine.configStateMachine.id);

    smListener.addPeriodicAction("st1", [](const std::string &currentState) {
        std::cout << "CoutObserver::First state " << std::endl;
    });
    smListener.addPeriodicAction("st2", [](const std::string &currentState) {
        std::cout << "CoutObserver::state 2 reached" << std::endl;
    });
    smListener.addOneOffAction("st2", [](const std::string &currentState) {
        std::cout << "CoutObserver::state 2 reached for the first time" << std::endl;
    });
    smListener.addAction("st3", [](const std::string &currentState) {
        std::cout << "LastStateObserver: This is the last state. We just changed to this state."
                  << std::endl;
    });
    stateMachine->registerObserver(smListener.getObserverFunc());
    LastStateObserver lastObserver;
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
