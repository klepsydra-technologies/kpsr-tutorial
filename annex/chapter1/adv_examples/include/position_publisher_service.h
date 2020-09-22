#ifndef POSITION_PUBLISHER_SERVICE_H
#define POSITION_PUBLISHER_SERVICE_H

#include <numeric>
#include <klepsydra/core/service.h>
#include <klepsydra/core/publisher.h>
#include "position.h"

class PositionPublisherService : public kpsr::Service
{
public:
    PositionPublisherService(kpsr::Publisher<Position> *publisher) :
        kpsr::Service(nullptr, "publisher_service")
        , _publisher(publisher)
        , _count(0) {}

protected:
    void start() {};

    void stop() {};

    void execute() {
        _count++;
        Position position(1 * _count, 2 * _count);
        _publisher->publish(position);
        std::thread::id this_id = std::this_thread::get_id();
        std::cout << "the positionPublisherThread id is : " << this_id << std::endl;
    }
private:
    kpsr::Publisher<Position> * _publisher;
    int _count;
};
#endif // POSITION_PUBLISHER_SERVICE_H
