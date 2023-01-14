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

#include <dds/dds.hpp>

#include <klepsydra/dds_core/from_dds_middleware_provider.h>
#include <klepsydra/dds_core/to_dds_middleware_provider.h>
#include <klepsydra/dds_serialization/primitive_type_dds_mapper.h>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

#include "simple_publisher.h"

int main(int argc, char **argv)
{
    dds::domain::DomainParticipant dp(0);
    dds::pub::Publisher publisher(dp);
    dds::sub::Subscriber subscriber(dp);

    std::string topicName("stringData");
    dds::topic::Topic<kpsr_dds_serialization::StringData> stringDataTopic(dp, topicName);
    dds::pub::DataWriter<kpsr_dds_serialization::StringData> stringDataWriter(publisher,
                                                                              stringDataTopic);
    dds::sub::DataReader<kpsr_dds_serialization::StringData> stringDataReader(subscriber,
                                                                              stringDataTopic);

    kpsr::dds_mdlw::ToDDSMiddlewareProvider provider(nullptr);
    kpsr::dds_mdlw::FromDDSMiddlewareProvider ddsProvider;

    kpsr::Publisher<std::string> *stringDataPublisher =
        provider.getToMiddlewareChannel<std::string, kpsr_dds_serialization::StringData>(
            topicName, 0, nullptr, &stringDataWriter);

    SimplePublisher simplePublisher(stringDataPublisher);

    kpsr::high_performance::EventLoopMiddlewareProvider<16> stringDataSafeQueueProvider(nullptr);
    stringDataSafeQueueProvider.start();
    ddsProvider.registerToTopic(topicName,
                                &stringDataReader,
                                true,
                                stringDataSafeQueueProvider.getPublisher<std::string>("string",
                                                                                      0,
                                                                                      nullptr,
                                                                                      nullptr));

    stringDataSafeQueueProvider.getSubscriber<std::string>("string")
        ->registerListener("example", [](std::string const &msg) {
            std::cout << "Received message: " << msg << std::endl;
        });

    simplePublisher.run();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    stringDataSafeQueueProvider.stop();
    ddsProvider.unregisterFromTopic(topicName, &stringDataReader);
    return 0;
}
