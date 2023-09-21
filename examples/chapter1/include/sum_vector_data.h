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

#ifndef SUM_VECTOR_DATA_H
#define SUM_VECTOR_DATA_H

#include <klepsydra/sdk/publisher.h>
#include <klepsydra/sdk/subscriber.h>
#include <numeric>
#include <thread>
#include <vector>

class SumVectorData
{
public:
    SumVectorData(kpsr::Subscriber<std::vector<float>> *subscriber,
                  kpsr::Publisher<float> *publisher)
        : _subscriber(subscriber)
        , _publisher(publisher)
    {
        _subscriber->registerListener("sum_vector", [this](const std::vector<float> &event) {
            float sum = calculateSum(event);
            _publisher->publish(sum);
            std::cout << "SumVectorDataClass (subscriber) thread ID: " << std::this_thread::get_id()
                      << std::endl;
        });
    }

    ~SumVectorData() { _subscriber->removeListener("sum_vector"); }

private:
    kpsr::Subscriber<std::vector<float>> *_subscriber;
    kpsr::Publisher<float> *_publisher;

    float calculateSum(const std::vector<float> &event)
    {
        return std::accumulate(event.begin(), event.end(), 0.0f);
    }
};

#endif // SUM_VECTOR_DATA
