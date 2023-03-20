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

#include "simple_listener.h"
#include "simple_publisher.h"

#include <functional>
#include <iostream>
#include <klepsydra/core/event_emitter_middleware_provider.h>

int main()
{
    std::cout << "Main thread ID: " << std::this_thread::get_id() << std::endl;

    kpsr::EventEmitterMiddlewareProvider<std::string> provider(nullptr,
                                                               "tutorial_app_api_example3",
                                                               0,
                                                               nullptr,
                                                               nullptr);

    SimplePublisher simplePublisher(provider.getPublisher());
    SimpleListener<std::string> simpleListener;

    provider.getSubscriber()->registerListener("example3",
                                               std::bind(&SimpleListener<std::string>::listener,
                                                         &simpleListener,
                                                         std::placeholders::_1));

    simplePublisher.run();

    provider.getSubscriber()->removeListener("example3");
}
