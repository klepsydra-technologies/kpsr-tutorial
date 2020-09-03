#ifndef POSITION_PUBLISHER_H
#define POSITION_PUBLISHER_H

#include <klepsydra/core/publisher.h>

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
	}
private:
	kpsr::Publisher<Position> * _publisher;
};
#endif
