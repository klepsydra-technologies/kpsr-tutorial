#include <iostream>

#include <klepsydra/state_machine/sm_factory_impl.h>
#include <klepsydra/state_machine/state_machine_factory.h>
#include <klepsydra/state_machine/yaml_config_loader.h>

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
    kpsr::fsm::YamlConfigLoader configLoader;
    kpsr::fsm::ConfigStateMachine configStateMachine = configLoader.loadConfig(TEST_DATA
                                                                               "/sm1.yaml");

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
