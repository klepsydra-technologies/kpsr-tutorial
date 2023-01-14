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

#include <klepsydra/state_machine/config_state_machine.h>

class SimpleStateMachine
{
public:
    SimpleStateMachine()
    {
        // Define transitions
        transition1 = {"st2", "event1"};
        transition2 = {"st3", "event2"};
        transition3 = {"st1", "event3"};

        // Define states
        state1 = {"st1", std::vector<kpsr::fsm::ConfigTransition>{transition1, transition2}};
        state2 = {"st2", std::vector<kpsr::fsm::ConfigTransition>{transition3}};
        state3 = {"st3", std::vector<kpsr::fsm::ConfigTransition>{transition1}};

        configStateMachine.id = "exampleStateMachine";

        configStateMachine.states = std::vector<kpsr::fsm::ConfigState>{state1, state2, state3};
    }

    kpsr::fsm::ConfigTransition transition1;
    kpsr::fsm::ConfigTransition transition2;
    kpsr::fsm::ConfigTransition transition3;

    // Define states
    kpsr::fsm::ConfigState state1;
    kpsr::fsm::ConfigState state2;
    kpsr::fsm::ConfigState state3;

    kpsr::fsm::ConfigStateMachine configStateMachine;
};
