<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# Chapter 4 - State Machine


## Pre-requisites

* Medium level of C++ and basic understanding of C++11 [share pointers](https://en.cppreference.com/w/cpp/memory/shared_ptr) and [lambda functions](https://en.cppreference.com/w/cpp/language/lambda).
* Basic knowledge of [CMAKE](https://cmake.org/)
* Understanding of YAML and JSON formats

## Technical requirements

Installation of Klepsydra Core with YAML support.

## Introduction

Klepsydra Core includes a State Machine. It has been developed so that it can be used along with the high performance eventloop API. We present here the basic usage of the State Machine.


## Components

The State Machine modules defines the following classes for use:

 - Transition:  Provides a destination `state id` and an `event` that triggers the transition
 - State: For a given `state id` defines a series of allowed transitions
 - StateMachine:
    - Allows the user to register Observer Functions.
        Observer Function are functions which take in the current state and whether the state has changed or not, and run some user defined action. 
    - Holds events and allows to enqueue new events. Events can trigger transitions to different states.
    - update(): Processes queued events and updates the state if a transition is permitted. This function calls all registered observers
 - StateMachineFactory:
    - A factory for creating a state machine. 
 - ConfigStateMachine, ConfigState, ConfigTransition
    Simple data classes to help define the state machine, state and transitions. A ConfigStateMachine class can be passed to the state machine factory to create a state machine programmatically.
 - StateMachineListener: A utility class to allow adding actions, one-off actions and periodic actions which are run if the current state of the state machine has changed to a specific state. Provides an observer function that can be registered to the state machine.
    
For convenience, we provide support for defining a state machine with a YAML file. The YAML file can be passed to the StateMachineFactory to create a state machine.


## Example 1: A simple state machine

```cpp
#include <iostream>

#include <klepsydra/state_machine/sm_factory_impl.h>
#include <klepsydra/state_machine/state_machine_factory.h>

struct Observer
{
    std::string _currentState;
    void updateCurrentState(const std::string &currentState, bool stateChanged)
    {
        if ("exampleStateMachine:st2" == currentState) {
            std::cout << "state 2 reached" << std::endl;
        } else if ("exampleStateMachine:st3" == currentState) {
            std::cout << "This is the last state" << std::endl;
        } else {
            std::cout << "Other states" << std::endl;
        }
        this->_currentState = currentState;
    }
};
int main()
{
    // Define transitions
    kpsr::fsm::ConfigTransition transition1 = {"st2", "event1"};
    kpsr::fsm::ConfigTransition transition2 = {"st3", "event2"};
    kpsr::fsm::ConfigTransition transition3 = {"st1", "event3"};

    // Define states
    kpsr::fsm::ConfigState state1 = {"st1",
                                     std::vector<kpsr::fsm::ConfigTransition>{transition1,
                                                                              transition2}};
    kpsr::fsm::ConfigState state2 = {"st2", std::vector<kpsr::fsm::ConfigTransition>{transition3}};
    kpsr::fsm::ConfigState state3 = {"st3", std::vector<kpsr::fsm::ConfigTransition>{transition1}};

    kpsr::fsm::ConfigStateMachine configStateMachine;
    configStateMachine.id = "exampleStateMachine";

    configStateMachine.states = std::vector<kpsr::fsm::ConfigState>{state1, state2, state3};

    kpsr::fsm::SMFactoryImpl smFactory;
    auto stateMachine = smFactory.createStateMachine(configStateMachine);
    Observer smObserver;
    stateMachine->registerObserver(std::bind(&Observer::updateCurrentState,
                                             smObserver,
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
```
This code shows the creation of a simple state machine using the factory implementation provided in the code sdk. We define the transitions and states using ConfigTransition, ConfigState classes and the state machine using these. In this example, there is a single "Observer" which prints a message to the screen depending on the current state.

The `stateMachine` variable is a shared_ptr to the StateMachine, created by the factory.

## Example 1a: State Machine with YAML configuration

The above state machine can also be defined in a YAML file and the configuration can be loaded directly from the file, rather than creating the machine programmatically. We present here the basic example for illustrative purposes. However, all the following examples define the *same* machine programmatically.

The above state machine can be represented in a YAML structure as below:
```yaml
state_machine:
  id: "exampleStateMachine"
  states:
  - state: # the first state is always the initial state
      id: "st1"
      transitions:
        - transition:
            destination: "st2"
            event: "event1"
        - transition:
            destination: "st3"
            event: "event2"
  - state:
      id: "st2" 
      transitions:
        - transition:
            destination: "st1"
            event: "event3"
  - state:
      id: "st3" 
      transitions:
        - transition:
            destination: "st2"
            event: "event1"
```
The only change then to the example 1 would then be adding the following include:

```cpp
#include <klepsydra/state_machine/yaml_config_loader.h>
```

And then the ConfigStateMachine can then be loaded directly from the file as :
```cpp
    kpsr::fsm::YamlConfigLoader configLoader;
    kpsr::fsm::ConfigStateMachine configStateMachine = configLoader.loadConfig(TEST_DATA "/sm1.yaml");
```

## Example 2: Multiple observers

For this case, we add some slightly complex observers to demonstrate how different observers can be used to run different actions depending on the state. The rest of the code for creating the state machine and the events being processed remain the same. To simplify the code, the state machine configuration is moved to a separate header file `simple_state_machine.h`.

```cpp
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
```

## Example 3: Enqueing multiple events

In this example we show how we can decouple enqueuing multiple events and updating the state machine. Since the transitions depend on the current state, the state will change depending on whether the transition is permitted or not. We can, in fact, separate the update cycle to a separate thread if we desire, and use the main thread only to enqueue events to the state machine.

In the code below, we have changed the list of enqueued events slightly compared to previous examples. However, because of the way the state transitions are defined, the state machine still follows the same series of state transitions as in previous examples.

```cpp
#include <iostream>
#include <thread>

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

    stateMachine->enqueueEvent("event1"); // transition to st2 if allowed
    stateMachine->enqueueEvent("event2"); // transition to st3 if allowed
    stateMachine->enqueueEvent("event3"); // transition to st1 if allowed
    stateMachine->enqueueEvent("event2"); // transition to st3 if allowed
    stateMachine->enqueueEvent("event3"); // transition to st1 if allowed
    stateMachine->enqueueEvent("event1"); // transition to st2 if allowed

    for (int i = 0; i < 10; i++) {
        std::cout << " Updating state machine" << std::endl;
        stateMachine->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    stateMachine->stop();
}
```
### Example 3a: Multiple enqueued events and threads
This is a slightly modified example 3, with only the enqueue and update steps being separated out to different threads.

```cpp
    stateMachine->start();

    std::thread updateThread([stateMachine](){
                                 for (int i= 0; i < 10; i++) {
                                     spdlog::info("Updating state machine");
                                     stateMachine->update();
                                     std::this_thread::sleep_for(std::chrono::milliseconds(200));
                                 }
                             });
    std::thread enqueueThread([&stateMachine](){
                                  spdlog::info("Enqueuing event1");
                                  stateMachine->enqueueEvent("event1");// transition to st2 if allowed
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
```
The rest of the code remains the same, and for brevity we do not replicate it here.

## Example 4: Actions using StateMachineListener

The StateMachineListener allows us to add actions easily rather than creating specific Observers. Functionality similar to the previous examples can be easily obtained using the state machine listener. However, the StateMachineListener couples actions (normal, one-off and periodic actions) to specific states. As a result, if you want to have a certain action run in multiple states, then you must use the standard observer as before.

- One-Off actions are only run the first time the state changes to the desired state.
- Periodic actions are run each time the current state is the desired state.
- Actions are run only when the current state has changed to the desired state.


```cpp
#include <iostream>
#include <thread>

#include <klepsydra/state_machine/sm_factory_impl.h>
#include <klepsydra/state_machine/state_machine_factory.h>
#include <klepsydra/state_machine/state_machine_listener.h>

#include <spdlog/spdlog.h>

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
    // Define transitions
    kpsr::fsm::ConfigTransition transition1 = {"st2", "event1"};
    kpsr::fsm::ConfigTransition transition2 = {"st3", "event2"};
    kpsr::fsm::ConfigTransition transition3 = {"st1", "event3"};

    // Define states
    kpsr::fsm::ConfigState state1 = {"st1",
                                     std::vector<kpsr::fsm::ConfigTransition>{transition1,
                                                                              transition2}};
    kpsr::fsm::ConfigState state2 = {"st2", std::vector<kpsr::fsm::ConfigTransition>{transition3}};
    kpsr::fsm::ConfigState state3 = {"st3", std::vector<kpsr::fsm::ConfigTransition>{transition1}};

    kpsr::fsm::ConfigStateMachine configStateMachine;
    configStateMachine.id = "exampleStateMachine";

    configStateMachine.states = std::vector<kpsr::fsm::ConfigState>{state1, state2, state3};

    kpsr::fsm::SMFactoryImpl smFactory;
    auto stateMachine = smFactory.createStateMachine(configStateMachine);

    kpsr::fsm::StateMachineListener smListener(configStateMachine.id);

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
```

## Example 5: State Machine with Klepsydra Publisher/Subscriber

We can use the Klepsydra Publish/Subscribe model easily with the state machine and the state machine listener. For this example, we repurpose the state machine used in the previous examples.

Assume this state machine describes a cubesat with battery and temperature sensors. We repurpose the states to signify the following:

- "st1" : Ready
- "st2" : Idle
- "st3" : Active

The allowed transitions are then Ready <-> Idle, Ready -> Active and Active -> Idle.

The events for this state machine are :
- "event1" -- Low power  (<= 20%)
- "event2" -- Full battery, active mode (>=60%)
- "event3" -- Medium battery, charged but cannot be put into active mode if not already in this mode

We add some actions as follows:
- In Idle state, cubesat should first print a "Low battery" warning, and notify that it is waiting for charge to be high.
- In Active state, the cubesat should measure and send the temperature.
- The first time cubesat is put into "Active", it should print a message to "calibrate the motors"

The battery sensor readings are published using the High Performance Event loop publisher. A subcriber listens to these battery readings and triggers events in the state machine depending on the battery level.

The example5.cpp shows the code for how this is achieved. For running this example, we have set up a dummy thread which simulates the battery first discharging, then being charged for a little while. This charging is interrupted and battery is drained further and finally the battery gets charged fully.
