/****************************************************************************
*	1 event loop
*	Listener to print the content of the event and the thread id
*	Publish and subscriber for other types of data: int, my own strct, etc.
*	Number as the output
*****************************************************************************/

#include <iostream>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

#include <thread>

#include <chrono>

# include "int_publisher.h"

# include "position_publisher.h"

int main() {
	kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);

	eventloop.start();
	// main thread id to print here
	eventloop.getSubscriber<int>("temperature receiver")->registerListener("listener", [](const int & message) {
		std::cout <<"temperature received t : " << message << std::endl;
		std::thread::id this_id = std::this_thread::get_id();
		std::cout << "the temperatureListener id is : " << this_id << std::endl;
	});

	eventloop.getSubscriber<Position>("position receiver")->registerListener("listener", [](const Position & message) {
		std::cout <<"position received x : " << message.x << " y : " << message.y << std::endl;
		std::thread::id this_id = std::this_thread::get_id();
		std::cout << "the positionListener id is : " << this_id << std::endl;
	});

	IntPublisher intPublisher(eventloop.getPublisher<int>("temperature receiver", 0, nullptr, nullptr));
	intPublisher.run();
	
	PositionPublisher positionPublisher(eventloop.getPublisher<Position>("position receiver", 0, nullptr, nullptr));
	std::thread t2([&positionPublisher, &intPublisher]() {
	// print the thread id here as well
	positionPublisher.run();
	intPublisher.run();
	});

	std::thread t3([&positionPublisher, &intPublisher]() {
	// print the thread id here as well
	intPublisher.run();
	});
	
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // message reaching subscriber before removing the listener afterwards.

	eventloop.getSubscriber<int>("temperature receiver")->removeListener("listener");

	eventloop.getSubscriber<Position>("position receiver")->removeListener("listener");

	t2.join();
	t3.join();

	eventloop.stop();
}
