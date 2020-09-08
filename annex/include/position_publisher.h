#ifndef POSITION_PUBLISHER_H
#define POSITION_PUBLISHER_H

#include <klepsydra/core/subscriber.h>
#include <klepsydra/core/publisher.h>
#include <numeric>

class Position {
public:
	float x;
	float y;
	Position(float _x, float _y) {
		x = _x;
		y = _y;
	}
	Position() {
		x = 0;
		y = 0;
	}
};
class PositionPublisher {
public:
	explicit PositionPublisher(kpsr::Publisher<Position> *publisher) : _publisher(publisher) {}

	void run() {
		Position position(1, 2);
		_publisher->publish(position);
		std::thread::id this_id = std::this_thread::get_id();
		std::cout << "the positionPublisher id is : " << this_id << std::endl;
	}
private:
	kpsr::Publisher<Position> * _publisher;
};

#endif
