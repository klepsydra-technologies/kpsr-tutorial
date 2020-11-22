#ifndef TEMPERATURE_SUBSCRIBER_SERVICE_H
#define TEMPERATURE_SUBSCRIBER_SERVICE_H

#include <klepsydra/core/service.h>
#include <klepsydra/core/subscriber.h>
#include <numeric>
#include <iostream>

class TemperatureSubscriberService : public kpsr::Service {
public:
    TemperatureSubscriberService(kpsr::Subscriber<int> *subscriber)
    : Service(nullptr, "subscriber_service")
    , _subscriber(subscriber) {}

protected:
    void start() override {
        _subscriber->registerListener("subscriber_service", [](const int & message) {
            std::cout <<"temperature received t : " << message << std::endl;
            std::thread::id this_id = std::this_thread::get_id();
            std::cout << "temperatureListenerThread id: " << this_id << '\n';
        });
    }

    void stop() override {
        _subscriber->removeListener("subscriber_service");
    }

    void execute() {}
private:
    kpsr::Subscriber<int> * _subscriber;
};
#endif // TEMPERATURE_SUBSCRIBER_SERVICE_H
