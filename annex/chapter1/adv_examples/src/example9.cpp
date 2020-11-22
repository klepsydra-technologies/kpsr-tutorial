/****************************************************************************
*    1 event loop
*    Listener to print the content of the event and the thread id
*    Publish and subscriber for other types of data: int, my own strct, etc.
*    Number as the output
*****************************************************************************/

#include <iostream>
#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <thread>
#include <chrono>
#include "position.h"
#include "temperature_publisher_service.h"
#include "temperature_subscriber_service.h"
#include "position_publisher_service.h"
#include "position_subscriber_service.h"

int main() {
    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);

    eventloop.start();
    // main thread id to print here
    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "the main thread id is : " << this_id << std::endl;

    TemperaturePublisherService temperaturePublisherService(eventloop.getPublisher<int>("temperatureCheck", 0, nullptr, nullptr));
    TemperatureSubscriberService temperatureSubscriberService(eventloop.getSubscriber<int>("temperatureCheck"));

    PositionPublisherService positionPublisherService(eventloop.getPublisher<Position>("positionCheck", 0, nullptr, nullptr));
    PositionSubscriberService positionSubscriberService(eventloop.getSubscriber<Position>("positionCheck"));

    temperatureSubscriberService.startup();
    temperaturePublisherService.startup();

    positionSubscriberService.startup();
    positionPublisherService.startup();

    temperaturePublisherService.runOnce();

    positionPublisherService.runOnce();

    std::thread dataProducerThread_2([&positionPublisherService, &temperaturePublisherService]() {
        // print the thread id here as well
        std::thread::id this_id = std::this_thread::get_id();
        std::cout << "the dataProducerThread_2 id is : " << this_id << std::endl;

        temperaturePublisherService.runOnce();
        positionPublisherService.runOnce();
    });

    std::thread dataProducerThread_3([&temperaturePublisherService]() {
        // print the thread id here as well
        std::thread::id this_id = std::this_thread::get_id();
        std::cout << "the dataProducerThread_3 id is : " << this_id << std::endl;

        temperaturePublisherService.runOnce();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // message reaching subscriber before removing the listener afterwards.

    dataProducerThread_2.join();
    dataProducerThread_3.join();

    temperaturePublisherService.shutdown();
    temperatureSubscriberService.shutdown();

    positionPublisherService.shutdown();
    positionSubscriberService.shutdown();

    eventloop.stop();
}
