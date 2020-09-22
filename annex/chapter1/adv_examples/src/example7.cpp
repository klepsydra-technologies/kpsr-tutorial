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

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    temperaturePublisherService.shutdown();
    temperatureSubscriberService.shutdown();

    positionPublisherService.shutdown();
    positionSubscriberService.shutdown();

    eventloop.stop();
}
