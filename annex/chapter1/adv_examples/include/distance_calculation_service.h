#ifndef DISTANCE_CALCULATION_SERVICE_H
#define DISTANCE_CALCULATION_SERVICE_H

#include <numeric>
#include <klepsydra/core/service.h>
#include <klepsydra/core/subscriber.h>
#include <klepsydra/core/publisher.h>
#include "position.h"

class DistanceCalculationService : public kpsr::Service
{
public:
    DistanceCalculationService(kpsr::Subscriber<Position> * subscriber, kpsr::Publisher<float> * publisher) :
        kpsr::Service(nullptr, "publisher_service"),
        _subscriber(subscriber), _publisher(publisher) {}

protected:
    void start() {
        _subscriber->registerListener("position_coordonnate", [this](const Position & event){
            float distanceOrig = computeDistance(event);
            std::cout << "Position x : " << event.x << " | y : " << event.y << std::endl;
            _publisher->publish(distanceOrig);
            std::thread::id this_id = std::this_thread::get_id();
            std::cout << "the positionListenerThread id and the distancePublisherThread id are : " << this_id << std::endl;
        });
    }

    void stop() {
        _subscriber->removeListener("position_coordonnate");
    }

    void execute() {}

private:
    kpsr::Subscriber<Position> * _subscriber;
    kpsr::Publisher<float> * _publisher;

    float computeDistance(const Position & event) {
        return sqrt(event.x*event.x + event.y*event.y);
    }
};
#endif // DISTANCE_CALCULATION_SERVICE_H
