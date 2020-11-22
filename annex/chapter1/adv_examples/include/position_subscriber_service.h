#ifndef POSITION_SUBSCRIBER_SERVICE_H
#define POSITION_SUBSCRIBER_SERVICE_H

#include <klepsydra/core/service.h>
#include <klepsydra/core/subscriber.h>
#include <numeric>
#include <iostream>
#include "position.h"

class PositionSubscriberService : public kpsr::Service {
public:
    PositionSubscriberService(kpsr::Subscriber<Position> *subscriber)
    : Service(nullptr, "subscriber_service")
    , _subscriber(subscriber) {}

protected:
    void start() override {
        _subscriber->registerListener("subscriber_service", [](const Position & message) {
            std::cout <<"position received x : " << message.x << " y : " << message.y << std::endl;
            std::thread::id this_id = std::this_thread::get_id();
            std::cout << "positionlistener id: " << this_id << '\n';
        });
    }

    void stop() override {
        _subscriber->removeListener("subscriber_service");
    }

    void execute() {}
private:
    kpsr::Subscriber<Position> * _subscriber;
};
#endif // POSITION_SUBSCRIBER_SERVICE_H
