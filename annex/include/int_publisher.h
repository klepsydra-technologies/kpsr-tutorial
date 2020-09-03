#ifndef INT_PUBLISHER_H
#define INT_PUBLISHER_H

#include <klepsydra/core/publisher.h>


class IntPublisher {
public:
	explicit IntPublisher(kpsr::Publisher<int> *publisher) : _publisher(publisher) {}

	void run() {
		_publisher->publish(1000000);
	}
private:
	kpsr::Publisher<int> * _publisher;
};
#endif
