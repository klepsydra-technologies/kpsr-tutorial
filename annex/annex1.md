<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# Chapter 1 Advanced examples

This annex is the follow-up of examples related to chapter 1 in this tutorial. 

## example7.cpp

The example7.cpp is about:

* Klepsydra Service class used to print the content of an event and its thread id

* Publisher and subscriber for other types of data: an int and a customized struct such as Position(float _x, float _y)

```cpp
    PositionPublisherService positionPublisherService(eventloop.getPublisher<Position>("positionCheck", 0, nullptr, nullptr));
    PositionSubscriberService positionSubscriberService(eventloop.getSubscriber<Position>("positionCheck"));

    positionSubscriberService.startup();
    positionPublisherService.startup();

    positionPublisherService.runOnce();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    positionPublisherService.shutdown();
    positionSubscriberService.shutdown();
```

Klepsydra Service class is used for Position publisher / subscriber.

## example8.cpp

The example8.cpp is an extension of example2.cpp from chapter1 of the tutorial. The goal of this new example is to publish a customized struct of data and then to apply some computations on them in order to publish another type of data. We have: 

* 1 event loop

* Publisher and subscriber for other type of data: in this case Position(float _x, float _y)

```cpp
  float computeDistance(const Position & event) {
    return sqrt(event.x*event.x + event.y*event.y);
  }
```

Distance to origin is computed and displayed by the subscriber based on the position received.

## example9.cpp

The example9.cpp is to test the event loop multithread behavior. Indeed, we have 2 threads, one that publishes and one that consumes the data. The goal of this example is to verify the multithreading capability of the event loop process. The goal is also to show that all the listeners of the event loop run in a single thread irrespective of the threading behaviour of the publishers. We have:

* 1 Publisher and subscriber for each type of data: in this case Temperature (int t) and Position(float _x, _y)

* Print thread ids of the publisher and the subscriber in different configurations (either from the main thread, or within the same thread for both position and temperature, or temperature in its own thread)

* It comes out that the listener always has the same thread id. Within the same thread, both position and temperature have identical thread ids which are however different from the listener's thread id). When the temperature alone is included in its own thread, then its thread id returns a completely new value. 
```cpp
    std::thread dataProducerThread_2([&positionPublisherService, &temperaturePublisherService]() {
    // print the thread id here as well
    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "the dataProducerThread_2 id is : " << this_id << std::endl;  

    temperaturePublisherService.runOnce();
    positionPublisherService.runOnce();
    });
```

Here-above illustrates when both position publisher (```positionPublisherService.runOnce()```) and temperature publisher (```temperaturePublisherService.runOnce()```) are in the same thread ```dataProducerThread_2``` for instance. 