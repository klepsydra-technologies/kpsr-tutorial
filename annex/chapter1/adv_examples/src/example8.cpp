/****************************************************************************
*    1 event loop
*    Publish and subscriber for other types of data: my own struct Position(x, y)
*    Distance to origin computation
*****************************************************************************/

#include <iostream>
#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <thread>
#include <chrono>
#include "distance_calculation_service.h"
#include "position_publisher_service.h"

int main() {
    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);

    PositionPublisherService positionPublisherService(eventloop.getPublisher<Position>("positionCheck", 0, nullptr, nullptr));

    eventloop.start();

    {
        positionPublisherService.startup();

        DistanceCalculationService distanceCalculationService(eventloop.getSubscriber<Position>("positionCheck"), eventloop.getPublisher<float>("distanceOrigin", 0, nullptr, nullptr));

        distanceCalculationService.startup();

        eventloop.getSubscriber<float>("distanceOrigin")->registerListener("distanceOrigin", [](const float & message) {
            std::cout << "Distance to origin : " << message << std::endl;
            std::thread::id this_id = std::this_thread::get_id();
            std::cout << "the distanceListenerThread id is : " << this_id << std::endl;

        });

        distanceCalculationService.runOnce();

        std::thread positionProducerThread([&positionPublisherService]() {
            for (int i = 0; i<100; i ++) {

                positionPublisherService.runOnce();

                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }

        });

        positionProducerThread.join();

        distanceCalculationService.shutdown();
        positionPublisherService.shutdown();

    }

    eventloop.stop();
}
