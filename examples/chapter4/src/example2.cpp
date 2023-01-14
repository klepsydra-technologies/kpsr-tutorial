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

#include <klepsydra/state_machine/sm_factory_impl.h>
#include <klepsydra/state_machine/state_machine_factory.h>

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

    stateMachine->enqueueAndUpdate("event1"); // transition to st2
    stateMachine->enqueueEvent("event3");     // transition to st1
    stateMachine->update();
    stateMachine->enqueueAndUpdate("event2"); // transition to st3
    stateMachine->enqueueAndUpdate("event3"); // no transition

    stateMachine->stop();
}
