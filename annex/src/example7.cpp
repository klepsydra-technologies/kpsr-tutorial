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

void foo()
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

int main() {
	kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);

	eventloop.start();
	
	std::thread t1(foo);
	std::thread::id t1_id = t1.get_id();
	std::cout << "t1's id: " << t1_id << '\n';
	eventloop.getSubscriber<int>("temperature receiver")->registerListener("listener", [](const int & message) {
		std::cout <<"temperature received t : " << message << std::endl;
	});

	eventloop.getSubscriber<Position>("position receiver")->registerListener("listener", [](const Position & message) {
		std::cout <<"position received x : " << message.x << " y : " << message.y << std::endl;
	});

	std::this_thread::sleep_for(std::chrono::milliseconds(1)); // listener registration before publisher runs.
	
	IntPublisher intPublisher(eventloop.getPublisher<int>("temperature receiver", 0, nullptr, nullptr));
	intPublisher.run();
	
	PositionPublisher positionPublisher(eventloop.getPublisher<Position>("position receiver", 0, nullptr, nullptr));
	positionPublisher.run();
	
	std::this_thread::sleep_for(std::chrono::milliseconds(1)); // message reaching subscriber before removing the listener afterwards.

	eventloop.getSubscriber<int>("temperature receiver")->removeListener("listener");

	eventloop.getSubscriber<Position>("position receiver")->removeListener("listener");

	eventloop.stop();
}
