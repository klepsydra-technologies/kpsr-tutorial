/****************************************************************************
*	1 event loop
*	Publish and subscriber for other types of data: my own struct Position(x, y)
*	Distance to origin computation
*****************************************************************************/

#include <iostream>
#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <thread>
#include <chrono>
#include "distance_data.h"

int main() {
	kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);

	kpsr::Publisher<Position> * positionPublisher = eventloop.getPublisher<Position>("whereabout", 0, nullptr, nullptr);
	

	eventloop.start();

	{
		DistanceData distanceData(eventloop.getSubscriber<Position>("whereabout"), eventloop.getPublisher<float>("distanceOrigine", 0, nullptr, nullptr));
	
		eventloop.getSubscriber<float>("distanceOrigine")->registerListener("distanceOrigine", [](const float & message) {
			std::cout << "Distance to origin : " << message << std::endl;

			});

		std::thread t([&positionPublisher]() {
		  for (int i = 0; i<100; i ++) {
		    Position whereabout(static_cast<float>(1*i), static_cast<float>(2*i));
		    positionPublisher->publish(whereabout);
		    std::this_thread::sleep_for(std::chrono::milliseconds(5));
		  }	
			
		});

		t.join();

	}

	eventloop.stop();
}
