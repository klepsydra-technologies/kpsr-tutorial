#ifndef INT_PUBLISHER_H
#define INT_PUBLISHER_H

#include <klepsydra/core/subscriber.h>
#include <klepsydra/core/publisher.h>
#include <numeric>
#include <thread>


class IntPublisher {
public:
	explicit IntPublisher(kpsr::Publisher<int> *publisher) : _publisher(publisher) {}

	void run() {
		_publisher->publish(1000000);
		std::thread::id this_id = std::this_thread::get_id();
		std::cout << "the intPublisher id is : " << this_id << std::endl;
	}
private:
	kpsr::Publisher<int> * _publisher;
};

#endif
