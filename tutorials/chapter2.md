# Chapter 2: Klepsydra middleware

<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

- [Introduction](#introduction)
- [ZMQ middleware](#zmq-middleware)
    - [The Hello World example](#the-hello-world-example)
    - [ZMQ Environment](#zmq-environment)

## Introduction

Klepsydra's API for ZMQ is part of the composition API presented in [chapter 1](chapter1.md). The functionality offered
for each of these middlewares includes:

- **Publication to Middleware**. Fulfilling the `kpsr::Publisher` API, Klepsydra offers an efficient way to publish to
  the middleware.
- **Subscription from Middleware**. The most powerful element of Klepsydra is the reception of incoming data from the
  middleware and passing it to the application. The associated API is explained in detail in this chapter.
- **Environment**. Klepsydra offers distributed configuration data by fulfilling the `kpsr::Environment` API based on
  middleware-specific data sharing capabilities.

The original goal of these three features of the middleware integration of Klepsydra was to isolate the application code
from any middleware dependencies, so that applications can:

- Have transparency over the underlying middleware.
- Develop unit tests and integrations tests agnostic of the underlying middleware.

This is the recommendation of several middleware providers
including [ROS](http://wiki.ros.org/Quality/Tutorials/UnitTesting#line-85).

## ZMQ middleware

Klepsydra compiled with ZMQ support generates the `kpsr_zmq_core` library which is available to be linked by
applications. In order to send or receive data using ZMQ, Klepsydra provides three ways of serializing data: JSON
encoding, binary encoding, or raw data. When dealing with primitive data types, no additional steps are required, and we
show below how the previous simple example would be used.

### The Hello World example

Going back to the fist example in chapter 1, `simple_publisher.h`:

```cpp
#include <klepsydra/sdk/publisher.h>

class SimplePublisher
{
public:
    explicit SimplePublisher(kpsr::Publisher<std::string> *publisher)
        : _publisher(publisher)
    {}

    void run() { _publisher->publish("Hello World!"); }

private:
    kpsr::Publisher<std::string> *_publisher;
};
```

Let's build a simple publisher for ZMQ:

```cpp
#include <zmq.hpp>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <klepsydra/serialization/json_cereal_mapper.h>
#include <klepsydra/zmq_core/from_zmq_middleware_provider.h>
#include <klepsydra/zmq_core/to_zmq_middleware_provider.h>

#include "simple_publisher.h"

int main()
{
    // Set up ZMQ specific options
    std::string serverUrl = "tcp://*:9001";
    std::string topic = "example1";
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind(serverUrl);
    std::string clientUrl = "tcp://localhost:9001";
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect(clientUrl);
    subscriber.set(zmq::sockopt::subscribe, topic);

    // ZMQ set up over, now our simple example
    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQMiddlewareProvider(nullptr, publisher);
    kpsr::Publisher<std::string> *toZMQPublisher =
        toZMQMiddlewareProvider.getJsonToMiddlewareChannel<std::string>(topic, 0);

    SimplePublisher publisherS(toZMQPublisher);

    // Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider fromZmqMiddlewareProvider;
    auto jsonFromZmqProvider = fromZmqMiddlewareProvider
                                    .getJsonFromMiddlewareChannel<std::string>(subscriber, 100);
    jsonFromZmqProvider->start();

    kpsr::high_performance::EventLoopMiddlewareProvider<16> stringDataSafeQueueProvider(nullptr);
    stringDataSafeQueueProvider.start();

    jsonFromZmqProvider->registerToTopic(
        topic, stringDataSafeQueueProvider.getPublisher<std::string>(topic, 0, nullptr, nullptr));

    stringDataSafeQueueProvider.getSubscriber<std::string>(topic)
        ->registerListener("example", [](std::string const &msg) {
            std::cout << "Received message: " << msg << std::endl;
        });
    std::this_thread::sleep_for(std::chrono::seconds(1));

    publisherS.run();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    stringDataSafeQueueProvider.getSubscriber<std::string>(topic)->removeListener("example");
    stringDataSafeQueueProvider.stop();
    jsonFromZmqProvider->unregisterFromTopic(topic);
    jsonFromZmqProvider->stop();
}
```

- The first block of code deals with the ZMQ set up. We declare the ZMQ publisher and subscribers here.
- Next we define the factory for the publisher `kpsr::zmq_mdlw::ToZMQMiddlewareProvider`. This takes in the ZMQ
  publisher as an input parameter using the Cereal library.

    ```cpp
    kpsr::Publisher<std::string> *toZMQPublisher =
        toZMQMiddlewareProvider.getJsonToMiddlewareChannel<std::string>(topic, 0);
    ```

  We could also use a binary serializer using the `getBinaryToMiddlewareChannel` or use raw data bytes using
  the `getVoidCasterToMiddlewareChannel` functions.
- Next we set up the `SimplePublisher`, our middleware-agnostic class which uses `kpsr::Publisher`: the internal mapper
  calls automatically convert the Klepsydra's messages to the ZMQ messages (both `std::string`).
- The next two lines show how to set up a subscriber factory which will receive JSON strings:

    ```cpp
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider fromZmqMiddlewareProvider;
    auto jsonFromZmqProvider = fromZmqMiddlewareProvider
                                    .getJsonFromMiddlewareChannel<std::string>(subscriber, 100);
    ```

  The subscriber factory is the `kpsr::zmq_mdlw::FromZmqMiddlewareProvider`, but depending on whether the serialization
  used was json, binary, or raw data, we choose to use the appropriate type of provider. Here, since the json
  serialization is used, we use the `getJsonFromMiddlewareChannel` function, with the ZMQ subscriber and a poll period
  in milliseconds as input parameters.
- The next step is to start the polling loop of the provider we obtained in the previous loop with a call to the `start`
  command.
- We next set up the event loop for the internal publisher/subscriber pairs and start it.
- We can connect a Klepsydra publisher/subscriber pair to the ZMQ subscriber factory provider using
  the `registerToTopic` function. The polling loop of the ZMQ factory provider parses the data received by ZMQ and
  publishes it to the Klepsydra publisher based on the topic name.
- The required callback is attached the Klepsydra subscriber (and is thus independent of ZMQ). In this example, we have
  used the high-performance event loop publisher and subscribers to attach to the ZMQ subscriber. However, for testing
  program logic and whether the callbacks are being executed, one could attach an event emitter publisher/subscriber or
  even the basic in-memory publisher/subscriber (from kpsr_mem_core module).
- The final four lines provide us the steps required to stop Klepsydra objects safely.

### ZMQ Environment

The `kpsr::zmq_mdlw::ZMQEnv` holds the configuration for the environment (or global data parameters to be shared over
the ZMQ network), along with an ability to share this data over the network. It allows reading and writing global
parameters, knowing the type of the parameter: string, int, float, or bool. The changes to the parameters are visible
immediately to all Klepsydra applications that are on the ZMQ network. As it inherits from the `kpsr::Environment`
class, the interface is the same as other middlewares.

```cpp
#include <iostream>
#include <zmq.hpp>
#include <klepsydra/zmq_core/zmq_env.h>

int main()
{
    std::string serverUrl = "tcp://*:9001";
    std::string topic = "example2";
    [...]

    kpsr::zmq_mdlw::ZMQEnv environment("", "test", topic, 100, publisher, subscriber);

    std::string stringPropertyName = "testPropString";
    std::string stringPropertyValue = "dummyValue";
    std::string checkStringPropertyValue;
    environment.setPropertyString(stringPropertyName, stringPropertyValue);
    environment.getPropertyString(stringPropertyName, checkStringPropertyValue);
    std::cout << "String env property " << stringPropertyName << ", written: '" << stringPropertyValue << "', read: '" << checkStringPropertyValue << "'" << std::endl;

    [...]

    std::string boolPropertyName = "testPropBool";
    bool boolPropertyValue = false;
    bool checkBoolPropertyValue;
    environment.setPropertyBool(boolPropertyName, boolPropertyValue);
    environment.getPropertyBool(boolPropertyName, checkBoolPropertyValue);
    std::cout << "Bool env property " << boolPropertyName << ", written: " << boolPropertyValue << ", read: " << checkBoolPropertyValue << std::endl;

    return 0;
}
```
