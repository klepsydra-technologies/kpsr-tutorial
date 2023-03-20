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

#ifndef SIMPLE_LISTENER_H
#define SIMPLE_LISTENER_H

#include <functional>
#include <iostream>
#include <string>
#include <thread>

template<class T>
class SimpleListener
{
public:
    SimpleListener()
        : _className("SimpleListener")
    {
        forwarderListenerFunction = std::bind(&SimpleListener<T>::listenerWithMultipleArgs,
                                              this,
                                              std::placeholders::_1,
                                              _className);
    }

    void listener(const std::string &message)
    {
        std::cout << "Message received in class method \"listener\": " << message << std::endl;
        std::cout << "Provider (subscriber) thread ID: " << std::this_thread::get_id() << std::endl;
    }

    static void staticListener(const std::string &message)
    {
        std::cout << "Message received in class method \"staticListener\": " << message
                  << std::endl;
        std::cout << "Provider (subscriber) thread ID: " << std::this_thread::get_id() << std::endl;
    }

    void listenerWithMultipleArgs(const std::string &message, const std::string className)
    {
        std::cout << "Message received in method \"listenerWithMultipleArgs\" of class "
                  << className << ": " << message << std::endl;
        std::cout << "Provider (subscriber) thread ID: " << std::this_thread::get_id() << std::endl;
    }

    std::function<void(const T &)> forwarderListenerFunction;

private:
    const std::string _className;
};

#endif
