#ifndef TEMPERATURE_PUBLISHER_SERVICE_H
#define TEMPERATURE_PUBLISHER_SERVICE_H

#include <numeric>
#include <klepsydra/core/service.h>
#include <klepsydra/core/publisher.h>

class TemperaturePublisherService : public kpsr::Service
{
public:
    TemperaturePublisherService(kpsr::Publisher<int> *publisher)
    : kpsr::Service(nullptr, "publisher_service")
    , _publisher(publisher) {}

protected:
    void start() {};

    void stop() {};

    void execute() {
        _publisher->publish(20);
        std::thread::id this_id = std::this_thread::get_id();
        std::cout << "the temperaturePublisherThread id is : " << this_id << std::endl;
    }
private:
    kpsr::Publisher<int> * _publisher;
};
#endif // TEMPERATURE_PUBLISHER_SERVICE_H
