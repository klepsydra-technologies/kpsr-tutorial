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
#include <iostream>

#include <dds/dds.hpp>

#include <klepsydra/dds_core/to_dds_middleware_provider.h>
#include <klepsydra/dds_core/from_dds_middleware_provider.h>
#include <klepsydra/dds_serialization/primitive_type_dds_mapper.h>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

#include "simple_publisher.h"

int main(int argc, char **argv) {
    dds::domain::DomainParticipant dp(0);
    dds::pub::Publisher publisher(dp);
    dds::sub::Subscriber subscriber(dp);

    std::string topicName("stringData");
    dds::topic::Topic<kpsr_dds_serialization::StringData> stringDataTopic(dp, topicName);
    dds::pub::DataWriter<kpsr_dds_serialization::StringData> stringDataWriter(publisher, stringDataTopic);
    dds::sub::DataReader<kpsr_dds_serialization::StringData> stringDataReader(subscriber, stringDataTopic);

    kpsr::dds_mdlw::ToDDSMiddlewareProvider provider(nullptr);
    kpsr::dds_mdlw::FromDDSMiddlewareProvider ddsProvider;

    kpsr::Publisher<std::string> * stringDataPublisher =
            provider.getToMiddlewareChannel<std::string, kpsr_dds_serialization::StringData>(topicName, 0, nullptr, &stringDataWriter);

    SimplePublisher simplePublisher(stringDataPublisher);

    kpsr::high_performance::EventLoopMiddlewareProvider<16> stringDataSafeQueueProvider(nullptr);
    stringDataSafeQueueProvider.start();
    ddsProvider.registerToTopic(topicName, &stringDataReader, true, stringDataSafeQueueProvider.getPublisher<std::string>("string", 0, nullptr, nullptr));
   
    stringDataSafeQueueProvider.getSubscriber<std::string>("string")->registerListener("example", [](std::string const& msg) {
        std::cout << "Received message: " << msg << std::endl; });

    simplePublisher.run();

    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    stringDataSafeQueueProvider.stop();
    ddsProvider.unregisterFromTopic(topicName, &stringDataReader);
    return 0;
}
