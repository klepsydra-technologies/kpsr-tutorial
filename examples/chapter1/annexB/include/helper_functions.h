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

#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <string>

#include "test_event.h"

std::atomic<int> TestEvent::constructorInvocations{0};
std::atomic<int> TestEvent::emptyConstructorInvocations{0};
std::atomic<int> TestEvent::copyInvocations{0};
std::atomic<int> TestEvent::initInvocations{0};
std::atomic<int> TestEvent::destructorInvocations{0};

int currentConstructorInvocations{0};
int currentEmptyConstructorInvocations{0};
int currentCopyInvocations{0};
int currentInitInvocations{0};
int currentDestructorInvocations{0};

void printInvocations(std::string message)
{
    std::cout << "Invocations counters updating after " << message << std::endl;
    if (currentConstructorInvocations == TestEvent::constructorInvocations &&
        currentEmptyConstructorInvocations == TestEvent::emptyConstructorInvocations &&
        currentCopyInvocations == TestEvent::copyInvocations &&
        currentInitInvocations == TestEvent::initInvocations &&
        currentDestructorInvocations == TestEvent::destructorInvocations) {
        std::cout << "\t(none)" << std::endl;
        return;
    }
    if (currentConstructorInvocations != TestEvent::constructorInvocations) {
        std::cout << "\tconstructorInvocations      " << TestEvent::constructorInvocations
                  << std::endl;
        currentConstructorInvocations = TestEvent::constructorInvocations;
    }
    if (currentEmptyConstructorInvocations != TestEvent::emptyConstructorInvocations) {
        std::cout << "\temptyConstructorInvocations " << TestEvent::emptyConstructorInvocations
                  << std::endl;
        currentEmptyConstructorInvocations = TestEvent::emptyConstructorInvocations;
    }
    if (currentCopyInvocations != TestEvent::copyInvocations) {
        std::cout << "\tcopyInvocations             " << TestEvent::copyInvocations << std::endl;
        currentCopyInvocations = TestEvent::copyInvocations;
    }
    if (currentInitInvocations != TestEvent::initInvocations) {
        std::cout << "\tinitInvocations             " << TestEvent::initInvocations << std::endl;
        currentInitInvocations = TestEvent::initInvocations;
    }
    if (currentDestructorInvocations != TestEvent::destructorInvocations) {
        std::cout << "\tdestructorInvocations       " << TestEvent::destructorInvocations
                  << std::endl;
        currentDestructorInvocations = TestEvent::destructorInvocations;
    }
}

void resetInvocationsCounters(void)
{
    TestEvent::constructorInvocations = 0;
    TestEvent::emptyConstructorInvocations = 0;
    TestEvent::copyInvocations = 0;
    TestEvent::initInvocations = 0;
    TestEvent::destructorInvocations = 0;

    currentConstructorInvocations = 0;
    currentEmptyConstructorInvocations = 0;
    currentCopyInvocations = 0;
    currentInitInvocations = 0;
    currentDestructorInvocations = 0;
}

#endif
