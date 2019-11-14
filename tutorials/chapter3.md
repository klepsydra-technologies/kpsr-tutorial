<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# Chapter 3. Code generation

## Introduction ##

This tutorial considers an example case of a cubesat with two
sensors - Battery monitor and a Temperature sensor. The functioning of
the cubesat depends on whether the battery and temperature values are
within operating ranges. In case they are not, the cubesat should be
shut down (or take other appropriate action). We show how Klepsydra
can be used to program for such a case.

## The Problem to solve:

We need to define a service which will receive messages from the
battery and temperature sensors and take appropriate action based on
the sensor values. For example, let's say that a low battery, extreme
low or extreme high temperature should trigger the cubesat to stop and
stay in idle mode.

If the battery level goes below 20%, then the cubesat stays idle until
the battery has charged back up to at least 25%. Similarly, if the
temperature goes below -15 C or above 60 C, the cubesat stops until
the temperature is back in the -10 C to 55 C range.

# The solution:

## Write the service classes

### The `ControlService` class

The Service class in Klepsydra core (kpsr-core) can be used to perform
regular tasks and is the interface we implement. The start method of
the Service class will start the subscribers to the battery and
temperature messages. The idea is that the class should be independent
of middlewares. So the class holds pointers to subscribers and publishers.

The `ControlService` class inherits from the Service class to perform
this task. It holds the subscribers for the battery and temperature
sensor messages, and a publisher for SystemEventData data type where it
publishes what action it desires should be taken.

We define functions that define what actions need to be done when
battery values are received. For example, the function on receiving
battery values is:

```
void kpsr::ControlService::onBatteryReceived(const kpsr::sensors::BatteryState & batteryData) {
    _batteryValue = batteryData.percentage;
    if ((kpsr::BATTERY_THRESHOLD_LOW >= _batteryValue) &&
        (_status != kpsr::SystemEventData::Stop)) {
        _status = kpsr::SystemEventData::Stop;
            _systemEventPublisher->publish(_status);
        }
    else if ((kpsr::BATTERY_THRESHOLD_LOW_START <= _batteryValue) &&
             (_status != kpsr::SystemEventData::Start)) {
        _status = kpsr::SystemEventData::Start;
        _systemEventPublisher->publish(_status);
    }
}
```

This "Listener" function is bound to the subscriber using std::bind and the `registerListener` method of the `Subscriber` class. For example:

```
std::function<void(kpsr::sensors::BatteryState)> batteryListenerFn = std::bind(
    &ControlService::onBatteryReceived, this, std::placeholders::_1);
_batterySubscriber->registerListener("batteryCacheListener", batteryListenerFn);
```

Thus, whenever the batterySubscriber receives a message, it will
execute the `onBatteryReceived` function.

### The `ApplicationService` class

To simplify the tasks done by each class, the `ControlService` class
is designed to only publish the actions it wants to be done. The
`ControlService` class deals only with the sensor data. The actions
that need to be performed should be handled by a different class which
has access to the cubesat controllers.

For this, we use the `ManagedService` abstract class which subscribes
to the `SystemEventData` messages. We define the `ApplicationService`
class to inherit from the `ManagedService` class, and define the
start, stop and execute methods specific to the cubesat.

In this example, these functions are empty, except for log messages to
let us know that the functions were called correctly. Note that the
two classes `ApplicationService` and `ControlService` are completely
independent of ROS, DDS or any middleware and deal with the
sensors/controllers of the cubesat.


## Specific tutorials for each case:

- [Tutorial for basic case, using no middlewares](./chapter3_part1.md)
- [Tutorial for basic case, ensuring subscribers are single threaded & thread-safe](./chapter3_part2.md)
- [Tutorial with ROS](./chapter3_part3.md)
- [Tutorial with DDS](./chapter3_part4.md)

