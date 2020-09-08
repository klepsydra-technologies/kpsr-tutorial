#ifndef TEMPERATURE_DATA_H
#define TEMPERATURE_DATA_H

#include <klepsydra/core/subscriber.h>
#include <klepsydra/core/publisher.h>
#include <numeric>

class TemperatureData {
public:
        TemperatureData(kpsr::Subscriber<int> * subscriber, kpsr::Publisher<int> * publisher) : _subscriber(subscriber), _publisher(publisher) {
                _subscriber->registerListener("temperature_measured", [this](const int & event) {
                        int measuredTemperature = event;
                        _publisher->publish(measuredTemperature);
                });
        }
        ~TemperatureData() {
                _subscriber->removeListener("temperature_measured");
        }

private:
        kpsr::Subscriber<int> * _subscriber;
        kpsr::Publisher<int> * _publisher;
};
#endif
