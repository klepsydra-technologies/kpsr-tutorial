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

#include "gtest/gtest.h"

#include "sum_vector_data.h"
#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>

TEST(SumVectorDataTest, NominalPoolTest)
{
    kpsr::EventEmitterMiddlewareProvider<std::vector<float>> vectorProvider(
        nullptr,
        "vector_provider",
        10,
        [](std::vector<float> &element) { element.resize(10); },
        nullptr);

    kpsr::EventEmitterMiddlewareProvider<float> sumProvider(
        nullptr, "sum_provider", 10, [](float &element) { element = 0.0f; }, nullptr);

    {
        SumVectorData sut(vectorProvider.getSubscriber(), sumProvider.getPublisher());

        kpsr::mem::CacheListener<float> eventListener;

        sumProvider.getSubscriber()->registerListener("cacheListener",
                                                      eventListener.cacheListenerFunction);

        std::vector<float> vector(10);
        for (int i = 0; i < 10; i++) {
            vector[i] = static_cast<float>(i + 1);
        }

        vectorProvider.getPublisher()->publish(vector);

        ASSERT_FLOAT_EQ(*eventListener.getLastReceivedEvent(), 55);
    }
}
