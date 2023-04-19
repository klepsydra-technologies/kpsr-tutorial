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

#ifndef TEST_EVENT_H
#define TEST_EVENT_H

#include <atomic>
#include <string>

class TestEvent
{
public:
    static std::atomic<int> constructorInvocations;
    static std::atomic<int> emptyConstructorInvocations;
    static std::atomic<int> copyInvocations;
    static std::atomic<int> initInvocations;
    static std::atomic<int> destructorInvocations;

    TestEvent() { emptyConstructorInvocations++; }

    TestEvent(int id, const std::string &message)
        : id(id)
        , message(message)
    {
        constructorInvocations++;
    }

    TestEvent(const TestEvent &that)
        : id(that.id)
        , message(that.message)
    {
        copyInvocations++;
    }

    ~TestEvent() { destructorInvocations++; }

    void init() { initInvocations++; }

    int id;
    std::string message;
};

#endif
