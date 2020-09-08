#ifndef POSITION_DATA_H
#define POSITION_DATA_H

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

class DistanceData {
public:
	DistanceData(kpsr::Subscriber<Position> * subscriber, kpsr::Publisher<float> * publisher) : _subscriber(subscriber), _publisher(publisher) {
		_subscriber->registerListener("position_coordonnate", [this](const Position & event){
		  float distanceOrigine = computeDistance(event);
		  std::cout << "Position x : " << event.x << " | y : " << event.y << std::endl;
			_publisher->publish(distanceOrigine);
		});
	}
	~DistanceData() {
		_subscriber->removeListener("position_coordonnate");
	}	

private:
	kpsr::Subscriber<Position> * _subscriber;
	kpsr::Publisher<float> * _publisher;

  float computeDistance(const Position & event) {
    return sqrt(event.x*event.x + event.y*event.y);
  }
};
#endif
