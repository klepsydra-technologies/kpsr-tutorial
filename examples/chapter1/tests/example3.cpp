/****************************************************************************
 *
 *                           Klepsydra Core Modules
 *              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *****************************************************************************/

#include "gtest/gtest.h"

#include "sum_vector_data.h"
#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>

TEST(SumVectorDataTest, NominalTest)
{
    kpsr::EventEmitterMiddlewareProvider<std::vector<float>> vectorProvider(nullptr,
                                                                            "vector_provider",
                                                                            0,
                                                                            nullptr,
                                                                            nullptr);

    kpsr::EventEmitterMiddlewareProvider<float> sumProvider(nullptr,
                                                            "sum_provider",
                                                            0,
                                                            nullptr,
                                                            nullptr);

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
