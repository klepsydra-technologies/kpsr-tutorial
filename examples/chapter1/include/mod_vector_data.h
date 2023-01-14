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

#ifndef MOD_VECTOR_DATA_H
#define MOD_VECTOR_DATA_H

#include <klepsydra/core/publisher.h>
#include <klepsydra/core/subscriber.h>

#include <cmath>
#include <numeric>

class ModuleVectorData
{
public:
    ModuleVectorData(kpsr::Subscriber<std::vector<float>> *subscriber,
                     kpsr::Publisher<float> *publisher)
        : _subscriber(subscriber)
        , _publisher(publisher)
    {
        _subscriber->registerListener("mod_vector", [this](const std::vector<float> &event) {
            float module = calculateModule(event);
            _publisher->publish(module);
            std::cout << "ModuleVectorDataClass (subscriber - mod) thread ID: "
                      << std::this_thread::get_id() << std::endl;
        });
    }

    ~ModuleVectorData() { _subscriber->removeListener("mod_vector"); }

private:
    kpsr::Subscriber<std::vector<float>> *_subscriber;
    kpsr::Publisher<float> *_publisher;

    float calculateModule(const std::vector<float> &event)
    {
        return sqrt(std::inner_product(event.begin(), event.end(), event.begin(), 0.0f));
    }
};
#endif //  MOD_VECTOR_DATA_H
