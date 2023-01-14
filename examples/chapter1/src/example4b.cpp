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

#include "sum_vector_data.h"

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

int main()
{
    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);
    kpsr::Publisher<std::vector<float>> *vectorPublisher = eventloop.getPublisher<std::vector<float>>(
        "vector", 10, [](std::vector<float> &element) { element.resize(10); }, nullptr);

    eventloop.start();
    {
        SumVectorData sumVectorData(eventloop.getSubscriber<std::vector<float>>("vector"),
                                    eventloop.getPublisher<float>("sum", 0, nullptr, nullptr));

        eventloop.getSubscriber<float>("sum")->registerListener("sum", [](const float &message) {
            std::cout << "Sum received: " << message << std::endl;
        });

        std::thread t([&vectorPublisher]() {
            for (int i = 0; i < 100; i++) {
                std::vector<float> vector(10);
                for (int j = 0; j < 10; j++) {
                    vector[j] = static_cast<float>(j);
                }
                vectorPublisher->publish(vector);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        });

        t.join();
    }

    eventloop.stop();
}
