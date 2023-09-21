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

#include <zmq.hpp>

#include <klepsydra/zmq_core/zmq_env.h>

int main()
{
    std::string serverUrl = "tcp://*:9001";
    std::string topic = "example2";
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind(serverUrl);
    std::string clientUrl = "tcp://localhost:9001";
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect(clientUrl);
    subscriber.set(zmq::sockopt::subscribe, topic);

    kpsr::zmq_mdlw::ZMQEnv environment("", "test", topic, 100, publisher, subscriber);
    std::string stringPropertyName = "testPropString";
    std::string stringPropertyValue = "dummyValue";
    std::string checkStringPropertyValue;
    environment.setPropertyString(stringPropertyName, stringPropertyValue);
    environment.getPropertyString(stringPropertyName, checkStringPropertyValue);
    std::cout << "String env property " << stringPropertyName << ", written: '"
              << stringPropertyValue << "', read: '" << checkStringPropertyValue << "'"
              << std::endl;

    std::string intPropertyName = "testPropInt";
    int intPropertyValue = 42;
    int checkIntPropertyValue;
    environment.setPropertyInt(intPropertyName, intPropertyValue);
    environment.getPropertyInt(intPropertyName, checkIntPropertyValue);
    std::cout << "Int env property " << intPropertyName << ", written: " << intPropertyValue
              << ", read: " << checkIntPropertyValue << std::endl;

    std::string floatPropertyName = "testPropFloat";
    float floatPropertyValue = 3.145f;
    float checkFloatPropertyValue;
    environment.setPropertyFloat(floatPropertyName, floatPropertyValue);
    environment.getPropertyFloat(floatPropertyName, checkFloatPropertyValue);
    std::cout << "Float env property " << floatPropertyName << ", written: " << floatPropertyValue
              << ", read: " << checkFloatPropertyValue << std::endl;

    std::string boolPropertyName = "testPropBool";
    bool boolPropertyValue = false;
    bool checkBoolPropertyValue;
    environment.setPropertyBool(boolPropertyName, boolPropertyValue);
    environment.getPropertyBool(boolPropertyName, checkBoolPropertyValue);
    std::cout << "Bool env property " << boolPropertyName << ", written: " << boolPropertyValue
              << ", read: " << checkBoolPropertyValue << std::endl;

    return 0;
}
