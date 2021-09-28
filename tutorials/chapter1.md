<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# Chapter 1

## Pre-requisites

* Medium level of C++ and basic understanding of C++11 [share pointers](https://en.cppreference.com/w/cpp/memory/shared_ptr) and [lambda functions](https://en.cppreference.com/w/cpp/language/lambda).
* Understanding of the publisher/subscriber pattern [example](https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern)
* Basic knowledge of [CMAKE](https://cmake.org/)
* [Unit testing](https://en.wikipedia.org/wiki/Unit_testing) knowledge and [google test](https://github.com/google/googletest) framework
* Although not required beforehand, there will be use of [YAML language](https://yaml.org/).
* Klepsydra recommends the use of the [composition root](https://medium.com/@cfryerdev/dependency-injection-composition-root-418a1bb19130) and [dependency injection](https://en.wikipedia.org/wiki/Dependency_injection) patterns. These two concepts appear repeatedly throughout this tutorial.

For the ROS tutorial, basic knowledge of ROS is required and similarly for the DDS tutorial, RTI DDS understanding is required.

## Technical requirements

Installation of Klepsydra Core, Optionally ROS, DDS.

Klepsydra is installed (by default) to /usr/local by CMake. To develop an application using Klepsydra use the cmake command `find_package(Klepsydra)` in the CMakeLists file.. With this command, the Klepsydra includes are available in the variable `KLEPSYDRA_INCLUDE_DIRS` and the libraries are available using the variable `KLEPSYDRA_CORE_LIBRARIES`. The CMakeLists file provided for compiling the examples in the tutorial demonstrate how these two variables are used.

## Introduction

Klepsydra API is split into main groups:

* **Application API**. A simple Publisher / Subscriber API, this API is used by the application or customer code to build the functionality.
* **Composition API**. This API is used to create and assembly publisher subscriber instances. Examples are: EventLoop, ROS Connector, etc.

This tutorial does not intend to cover the complete API of Klepsydra Core, but to explain the key programming features of it. For a full comprehensive description of the API, please refer to the [api-doc](https://github.com/klepsydra-technologies/kpsr-core/tree/master/api-doc) folder of the kpsr-core repository.

## Example 1: The "Hello world!" example.

Lets start with a simple example. First, let's create a class that publishes a string:

```cpp
#include <klepsydra/core/publisher.h>

class SimplePublisher {
public:
   SimplePublisher(kpsr::Publisher<std::string> * publisher)
      : _publisher(publisher)
   {}
   
   void run() {
      _publisher->publish("Hello World!");
   }

private:
   kpsr::Publisher<std::string> * _publisher;
}
```

There are two important elements in this class that are always required when programming with Klepsydra:

* The ```publish``` method expects a const reference to the publishing object. All publisher implementations make a copy of this object before proceeding to the actual publication. There is another API for publication that skip the copy, that we will cover later.
* Application code always use pointers to ```kpsr::Publisher```. This is done in order to keep the application independent of the implementation of the publisher. It is not only a good practice, but also necessary when building unit tests for classes like this example. The same principle applies to the ```kpsr::Subscriber``` and ```kpsr::Environment```.

Let's now build simple main that makes use of this example class:

```cpp
#include <iostream>
#include "simple_publisher.h"
#include <klepsydra/core/event_emitter_middleware_provider.h>

int main() {
   kpsr::EventEmitterMiddlewareProvider<std::string> provider(nullptr, "tutorial_app_api_example1", 0, nullptr, nullptr);
   
   SimplePublisher simplePublisher(provider.getPublisher());

   provider.getSubscriber()->registerListener("example1", [](const std::string & message) {
         std::cout << "Message received: " << message << std::endl;
      }
   );
   
   simplePublisher.run();

   provider.getSubscriber()->removeListener("example1");
   
}
```

This examples shows several features of Klepsydra already. Let's have a look at them line by line:

* ```kpsr::EventEmitterMiddlewareProvider<std::string> provider(nullptr, "tutorial_app_api_example1", 0, nullptr, nullptr);```. This line creates the most basic memory sharing provider available in Klepsydra: the Event Emitter. It is a synchronous, single event publisher/subscriber implementation. It is used solely for testing purposes, however it works similarly to the rest of the memory sharing providers: 
   * creates a paired publisher/subscriber
   * the api gives pointers to ```kpsr::Publisher``` and ```kpsr::Subscriber```

*    ```provider.getSubscriber()->registerListener``` is the API to register a consumer for events in Klepsydra. A name and a lambda function are the arguments.
*    ```provider.getSubscriber()->removeListener``` is the API to remove a consumer from the ```Subcriber```. In order to avoid unexpected behaviour at shutting down applications, it is stringly recommended to remove listeners before stopping threads or middleware providers.

## Example 2: The eventloop

As suggested before, the ```EventEmitterMiddlewareProvider``` is a test only provider, so let's change it for one of the two main high performance providers in Klepsydra: **The Event Loop**:

```cpp
#include <iostream>
#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include "simple_publisher.h"

int main() {
    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);

    eventloop.start();
    eventloop.getSubscriber<std::string>("example2")->registerListener("listener",
                                                             [](const std::string & message) {
                                                                 std::cout << "Message received: " << message << std::endl;
                                                             });
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Ensures listener has been registered before publisher runs.

    SimplePublisher simplePublisher(eventloop.getPublisher<std::string>("example2", 0, nullptr, nullptr));
    simplePublisher.run();
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Ensures published message ALWAYS reaches subscriber before the listener is removed in next line.

    eventloop.getSubscriber<std::string>("example2")->removeListener("listener");
    eventloop.stop();
}
```

This version of the main is slighlty different from the previous one in the following elements:

* The instatiation of the ```EventLoopMiddlewareProvider``` does not take any template, nor the ```std::function``` pair for the smart object pool. The reason for that is that the ```EventLoop``` is a general memory sharing system that can handle data of any type, it only requires the size of the ring buffer as template parameter. It is only when invoking ```getPublisher``` and ```getSubscriber``` that types need to be specified. The ```EventEmitter``` can only handle one publisher / subcsriber pair, this is why the templating and smart pool configuration happen at the declaration and construction level.
* Another important difference is that the event loop requires starting and stopping. The start will create and start the listening thread, which is the only thread that will handle all the listeners associated to all subscribers. It is important that, before stopping, all listeners have been removed.

## Example 3: Unit testing. 

Now, lets take a step further in the concept of separation of the application API and composition API. In asynchronous programming, methods will publish data instead of returning data:

```cpp
class SumVectorData {
public:
   SumVectorData(kpsr::Subcriber<std::vector<float>> * subscriber,
                  kpsr::Publisher<float> * publisher)
      : _subcriber(subscriber)
      , _publisher(publisher)
      {
         _subscriber->registerListener("sum_vector", [this](const std::vector<float> & event) {
            float sum = calculateSum(event);
            _publisher->publish(sum);
         });
      }
      
   ~SumVectorData() {
         _subscriber->removeListener("sum_vector");
      }
      
private:
   kpsr::Subcriber<std::vector<float>> * _subscriber;
   kpsr::Publisher<float> * _publisher;
   
   float calculateSum(const std::vector<float> & event) {
      return std::accumulate(event.begin(), event.end(), 0.0f);
   }
}
```

This class performs a simple sum of all elements in a vector of floats. The main difference with tradition synchronous programming, is that the public API of this method are a publisher and a subscriber, instead of a tradition method API.

Although this example is quite simple, testing of asynchronous application can become quite cumbersome. Klepsydra offers a simple solution for testing by using the ```EventEmitterProvider```:

```cpp
#include "gtest/gtest.h"

#include "sum_vector_data.h"
#include <klepsydra/core/event_emitter_middleware_provider.h>

TEST(SumVectorDataTest, NominalTest) {
    kpsr::EventEmitterMiddlewareProvider<std::vector<float>> vectorProvider(nullptr, "vector_provider", 0, nullptr, nullptr);
    
    kpsr::EventEmitterMiddlewareProvider<float> sumProvider(nullptr, "sum_provider", 0, nullptr, nullptr);

   {
       SumVectorData sut(vectorProvider.getSubscriber(), sumProvider.getPublisher());
   
       kpsr::mem::CacheListener<float> eventListener;
   
       sumProvider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);
   
      std::vector<float> vector(10);
      for (int i = 0; i < 10; i++) {
         vector[i] = static_cast<float>(i);
      }
      
      vectorProvider.getPublisher()->publish(vector);
      
      ASSERT_FLOAT_EQ(* eventListener.getLastReceivedEvent(), 45);
   }
}

```

Explanation:

* Each publisher subscriber pair in the event emitter requires a new instance as explained earlier. In this example, the class ```SumVectorData``` makes use of a subscriber and a publisher of different types.
* We are introducing a new element of Klepsydra API: ```kpsr::mem::CacheListener```. This is a special type of listener that simply store the data in an internal variable. In this example, this listener is used to store and validate the data published to the sum publisher.
* The rest of the test is a matter of building controlled data to publish and check the correct functioning of the class under test.
* The use of the curly brackets block inside the test is to ensure that the destruction of the instance of ```SumVectorData``` happens before the destructors of event emitter instances are called. This is because the listener removal happens in the destructor code. As this can make the code somewhat obscure, Klepsydra has the ```kpsr::Service``` API to avoid this issue. More on that later in this tutorial.

Let us build now a running application for this class:

```cpp
#include <iostream>

#include "sum_vector_data.h"

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

int main() {
   kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);
   
   kpsr::Publisher<std::vector<float>> * vectorPublisher = eventloop.getPublisher<std::vector<float>>("vector", 0, nullptr, nullptr);
   
   eventloop.start();
      
   {
      SumVectorData sumVectorData(
            eventloop.getSubscriber<std::vector<float>>("vector"),
            eventloop.getPublisher<float>("sum", 0, nullptr, nullptr));
   
      eventloop.getSubscriber<float>()->registerListener("sum", [](const float & message) {
            std::cout << "Sum received: " << message << std::endl;
         }
      );
      
       std::thread t([&vectorPublisher]() {
          for (int i = 0; i < 100; i ++) {
            std::vector<float> vector(10);
            for (int j = 0; j < 10; j++) {
               vector[j] = static_cast<float>(j);
            }
            vectorPublisher->publish(vector);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
         }
       });
   
       t.join();
    }

   eventloop.stop();
}
```

This example shows a slightly more advanced use of the event loop. In this case, there are two threads, one that publishes vector data, this somehow resembles an external source of data. Then there is the event loop thread that consumes the data, but in the case, the sum vector class also publishes data to the eventloop. This is one of the main features of the event loop: being able to receive data from multiple sources, including the consumer thread. This behaviour, instrumental in asyc programming, can be found in other event loops, like the Event Emitter in NodeJs (TODO LINK).

Another important point of this example is that we have shown the use of the same class in two different middleware provider setups: unit test and real application. This one of the main programmatic concepts in Klepsydra. The main principle behind is to have complete separation of application and composition code.

## Example 4: A note on performance

By default, the ```publish(T & event)``` method in all implementations makes a copy of the ```event``` object. In some cases, this copy might have to be prevented, or at least, ensure that no memory allocations happen during execution of applications. There are two ways to make this more efficient:

### Publication API

By using the ```publish(std::shared_ptr<const T> & event)``` API instead. This will make direct use of the provided object instead of creating a copy. For example, we could change the code of the main block in example 3:

```cpp
   {
       SumVectorData sut(vectorProvider.getSubscriber(), sumProvider.getPublisher());

       kpsr::mem::CacheListener<float> eventListener;

       provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);
       std::shared_ptr<std::vector<float>> vector(new std::vector<float>(10));
       for (int i = 0; i < 10; i++) {
           (*vector)[i] = static_cast<float>(i);
       }

       vectorProvider.getPublisher()->publish(vector);
       ASSERT_FLOAT_EQ(* newEventListener.getLastReceivedEvent(), 45);
   }
```

This will prevent any copies of the vector when publishing.

### Smart Memory Pool

The above code is simple and efficient, but in a sensor data publication scenario, it might not be possible. In those cases, performance and number of allocations can also be improved by using the smart object pool in Klepsydra. ```kpsr:: SmartObjectPool``` is the main API for this and i ensures that during the publication copy, no memory allocation happens.

Let's look again at the example 3 main file:

```cpp
...
int main() {

    kpsr::Publisher<std::vector<float>> * vectorPublisher = provider.getPublisher<std::vector<float>>("vector", 0, nullptr, nullptr);
...
}
```

The last 3 arguments of the ```getPublisher``` method are for the ```kpsr::SmartPool``` configuration. When the provided size is 0, like in this example, the pool is disabled. Let's now enable it:

```cpp
...
int main() {

    kpsr::Publisher<std::vector<float>> * vectorPublisher = eventloop.getPublisher<std::vector<float>>("vector", 4, nullptr, nullptr);
...
}
```

Now, we have a memory pool of 4 elements which will make the application run faster, but only after a "warm-up" period. This "warm-up" consists of using all 4 elements in the pool by the publisher, so that all vectors are sized to 10 elements. This warm-up can be prevented by providing an initialization function as the second-last argument:

```cpp
...
int main() {

    kpsr::Publisher<std::vector<float>> * vectorPublisher = eventloop.getPublisher<std::vector<float>>("vector", 4, [](std::vector<float> & element) { 
        element.resize(10);
   }, nullptr);
...
}
```

With this code, we have ensured that any vector published into the eventloop will produce no memory allocations.

The last argument, a clone function that the publisher will use instead of standard copy, is also to increase performance. This is covered in depth in the [vision tutorial](https://github.com/klepsydra-technologies/kpsr-vision-ocv-tutorial).

## Example 5: The data multiplexer.

Let's look now at the second high performance API in Klepsydra: ```kpsr::high_perf::DataMultiplexerMiddlewareProvider```. This is a single-producer, multiple-consumer API for Klepsydra. We are going to extend example 3, so that we will have a second consumer of the vector that will calculate the module of the vector:

```cpp
class ModuleVectorData {
public:
    ModuleVectorData(kpsr::Subcriber<std::vector<float>> * subscriber,
                        kpsr::Publisher<float> * publisher)
       : _subcriber(subscriber)
       , _publisher(publisher)
       {
          _subscriber->registerListener("mod_vector", [this](const std::vector<float> & event) {
             float module = calculateModule(event);
             _publish->publish(module);
          });
       }

    ~ ModuleVectorData() {
         _subscriber->removeListener("mod_vector");
      }

private:
    kpsr::Subcriber<std::vector<float>> * _subscriber;
    kpsr::Publisher<float> * _publisher;

    float calculateModule(const std::vector<float> & event) {
      return sqrt(std::inner_product(event.begin(), event.end(),event.begin(), 0.0f));
   }
}
```

For the sake of brevity, we do not include the unit test for this class, but one is provided in the correspoding module of the tutorial repository.

Now, the assemblying in the main class:

```cpp
#include <iostream>

#include "sum_vector_data.h"
#include "mod_vector_data.h"

#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <klepsydra/high_performance/data_multiplexer_middleware_provider.h>

int main() {
    kpsr::high_performance::EventLoopMiddlewareProvider<16>eventloop(nullptr);

    kpsr::high_performance::DataMultiplexerMiddlewareProvider<std::vector<float>, 4> dataMultiplexer(nullptr, "example5", nullptr, nullptr);

    kpsr::Publisher<std::vector<float>> * vectorPublisher = dataMultiplexer.getPublisher();

    eventloop.start();

    {
        SumVectorData sumVectorData(
            dataMultiplexer.getSubscriber(),
            eventloop.getPublisher<float>("sum", 0, nullptr, nullptr));
        eventloop.getSubscriber<float>()->registerListener("sum", [](const float & message) {
            std::cout << "Sum received: " << message << std::endl;
         }
      );

      ModuleVectorData moduleVectorData(
            dataMultiplexer.getSubscriber(),
            eventloop.getPublisher<float>("mod", 0, nullptr, nullptr));

      eventloop.getSubscriber<float>()->registerListener("mod", [](const float & message) {
            std::cout << "Module received: " << message << std::endl;
         }
      );
      std::thread t([&vectorPublisher]() {
          for (int i = 0; i < 100; i ++) {
            std::vector<float> vector(10);
            for (int j = 0; j < 10; j++) {
               vector[i] = static_cast<float>(j);
            }
            vectorPublisher->publish(vector);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
         }
      });

       t.join();
    }

    eventloop.stop();
}
```

As it can be seen in this code, the ```kpsr::high_perf::DataMultiplexer``` has a similar API to other providers we have covered, however, it behaves very differently to other providers:

* The templating occurs at the declaration level, similar to the event emitter. The data multiplexer is built to accelerate consumption of specific data, as opposed to the eventloop which is for general purpose.
* This is the only provider that does not make use of the smart pool. It uses, instead, the internal ring-buffer itself as the pool of data. This makes it much more efficient. The initializer and clone functions though can be passed in a similar manner to the event emitter..
* There is one subscriber in the provider, but listeners are the drivers of the processing:
	* Registering a listener will start its own thread of execution. 
	* Listeners in data multiplexer are invoked independently of each other.
	* Data needed by a listener will be kept in the buffer until all listeners  are done processing it
	* Listener will have access to the latest data only. If a listener is slow in processing, data multiplexer will skip old data and only provide it with the latest one.
* There is no start or stop methods. The start is implicitly triggered when a listener is registered in the ```Subscriber```.

There is an extensive use of the DataMultiplexer in the [vision tutorial](https://github.com/klepsydra-technologies/kpsr-vision-ocv-tutorial).

## Example 6: Working with services

We are going to introduce now the API for ```kpsr::Service```, ```kpsr::Environment``` and ```kpsr::ManagedService```. These classes are not really necessary to benefit from the high performance API of Klepsydra, but might be of help.

### Service

The ```kpsr::Service``` allows one to define a task which will run in the background, which can be administered remotely and/or which need to be run at regular intervals. The ```kpsr::Service``` class is used by means of inheritance. I.e., custom services must extend from this class. ```kpsr::Service``` has three pure virtual methods:

* ```start()```, 
* ```stop()```, 
* ```execute()``` 

That custom services need to implement. These functions are never invoked directly, but via their public counterparts or public API functions:

* ```startup()```, 
* ```shutdown()``` and 
* ```runOnce()```.

Very important to know is that the ```runOnce``` invokes the ```execute``` method when the service is started, i.e., once the ```startup``` function have been called.

Let's start with the most basic publisher as a service.

```cpp
class SimplePublisherService : public Service {
public:
    SimplePublisherService(kpsr::Publisher<std::string> * publisher)
    : Service(nullptr, "publisher_service")
    , _publisher(publisher) {}

protected:
    void start() {}; // Empty on purpose

    void stop() {}; // Empty on purpose

    void execute () {
        _publisher->publish("Hello world");
    }

private:

    kpsr::Publisher<std::string>* _publisher;
}
```

Now this publisher service can be started using its startup() command, stopped using shutdown() command, and the runOnce() command will publish the intended message. Note that the publisher does not need any setup procedure and that is why the start and stop functions are empty.

Notice the constructor call of the service. The Service class needs three parameters: a pointer to kpsr::Environment, a std::string containing the name of the service, and a bool (default to true) to signify if service is master or not. The Environment class will be explained in the next section. In these example we don't use any environments and the `nullptr` is passed instead.

Similar to the publisher service we can have a basic subscriber as a service.
```cpp
class SimpleSubscriberService : public Service {
public
    SimpleSubscriberService(kpsr::Subscriber<std::string> * subscriber)
    : Service(nullptr, "subcriber_service")
    ,_subscriber(subscriber) {}

protected:
    void start() override {
        _subscriber->registerListener("subcriber_service", [](const std::string & message) {
            std::cout << "Message received: " << message << std::endl;
        });
    }

    void stop() override {
        _subscriber->removeListener("subscriber_service");
    }

    void execute() {} // Empty on purpose.

private:
    kpsr::Subscriber<std::string> * _subscriber;
};

```
A complete example using these two services is given below:
```cpp
#include <iostream>
#include "simple_publisher_service.h"
#include "simple_subscriber_service.h"
#include <klepsydra/high_performance/event_loop_middleware_provider.h>

int main() {
    kpsr::high_performance::EventLoopMiddlewareProvider<16> provider(nullptr);

    SimplePublisherService publisherService(provider.getPublisher<std::string>("example2", 0, nullptr, nullptr));
    SimpleSubscriberService subscriberService(provider.getSubscriber<std::string>("example2"));

    publisherService.startup();
    subscriberService.startup();

    publisherService.runOnce();

    publisherService.shutdown();
    subscriberService.shutdown();
}
```

This example has the same functionality as example 1, but the Services api allows us to have much cleaner and readable code. 

The service api can also be used effectively when we have slightly involved problems. Consider the following example where we have an application running along with a battery sensor publishing the battery levels. We want the application to stop when the battery level goes low. Let's call the service that will monitor the battery status as the Control Service. The most basic implementation of such a service is given below:

```cpp

namespace kpsr {
class ControlService : public Service
{

public:

    ControlService(kpsr::Subscriber<float> * batterySubscriber)
    : Service(nullptr, "control_service")
    , _batterySubscriber(batterySubscriber)
    {}

protected:

    void ControlService::start() {
        std::function<void(float)> listenerFunction = [this](const float event) {
            if (LOW_THRESHOLD >= event) {
                std::cout << "Stop the application" << std::endl;
            }
        }
        _batterySubscriber->registerListener("batteryListener", listenerFunction);
    }

    void ControlService::stop() {
        _batterySubscriber->removeListener("batteryListener");
    }

private:
    kpsr::Subscriber<float> * _batterySubscriber;
};
}
```

Now, this service can be started using the public function ```startup``` and it will start monitoring the battery topic and if the battery levels go below the threshold, it will print the "Stop the application" command. 

### Environment

If we see the previous example, it becomes clear that the threshold value for the battery must be hard coded into the code. However, in real cases, it would be preferable to have such data depend on the real environment. The code for services should not have to be modified according to the environment in which the application is running. For example, one should be able to provide a configuration file to set these values, or for example, in case of ROS system provide run time values via command line.

Klepsydra provides a `kpsr::Environment` API that provides this facility. It allows us to isolate the services and other code from the environment in which it will run. The `kpsr::mem::MemEnv` is one such implementation providing a in-memory environment used specially for unit testing. Also, the `kpsr::YamlEnvironment` (available when Klepsydra is compiled with YAML support), allows setting up an environment using a YAML file. With this, the application can be run in different environments just by providing a different YAML configuration file.

Using the environment api, the relevant ControlService code changes as shown below. We only present the functions that change, and also add another constraint by providing an upper threshold.

```cpp

    ControlService(kpsr::Subscriber<float> * batterySubscriber, kpsr::Environment * env)
    : Service(env, "control_service")
    , _batterySubscriber(batterySubscriber)
    {}

    void ControlService::start() {
        std::function<void(float)> listenerFunction = [this](const float event) {
            float lowThreshold;
            this->environment->getFloatProperty("low_threshold", lowThreshold);
            if (lowThreshold >= event) {
                std::cout << "Stop the application" << std::endl;
            }
            float highThreshold;
            this->environment->getFloatProperty(“high_threshold”, highThreshold);
            if (highThreshold <= event) {
                std::cout << "Start the application" << std::endl;
            }
        }
        _batterySubscriber->registerListener("batteryListener", listenerFunction);
    }
```

### Managed Services

Klepsydra provides a `kpsr::ManagedService` class that starts or stops a service depending on the `kpsr::SystemEventData` value that is sent. This allows us to connect one service to another. For example, in the previous example, the actual application code would be a managed service, and it will respond to the messages sent by the control service appropriately.

#### A message type to control services

Klepsydra offers a special datatype ```kpsr::SystemEventData``` for the specific purpose of communicating if a certain service or application has to be started or stopped. We can thus improve the previous control by using it as shown below::

```cpp

namespace kpsr {
class ControlService : public Service
{

public:

    ControlService(kpsr::Subscriber<float> * batterySubscriber,
                   kpsr::Publisher<kpsr::SystemEventData> * statusPublisher, kpsr::Environment * env)
    : Service(env, "control_service")
    , _batterySubscriber(batterySubscriber)
    , _statusPublisher(statusPublisher) {}

protected:

    void ControlService::start() {
        std::function<void(float)> listenerFunction = [this](const float event) {
            float lowThreshold;
            this->environment->getFloatProperty("low_threshold", lowThreshold);
            if (lowThreshold >= event) {
                _statusPublisher->publish(kpsr::SystemEventData::Stop);
            }
            float lowThreshold;
            this->environment->getFloatProperty("low_threshold", lowThreshold);
            if (lowThreshold >= event) {
                _statusPublisher->publish(kpsr::SystemEventData::Start);
            }
        }
        _batterySubscriber->registerListener("batteryListener", listenerFunction);
    }

    void ControlService::stop() {
        _batterySubscriber->removeListener("batteryListener");
    }

private:
    kpsr::Subscriber<float> * _batterySubscriber;
    kpsr::Publisher<kpsr::SystemEventData> * _statusPublisher;
};
}
```
The application that needs to be stopped can now by handled by using a subscriber to the status publisher. This also makes testing the Control Service easier. The unit test `example6b.cpp` in the tests folder provides an example of usage of the environment and how to test the internal logic of the Control Service.

#### Managed Service Example

The managed service class constructor needs as input parameters the environment pointer, a subscriber of SystemEventData messages, and a name for the class. A simple managed service could be as follows:

```cpp
namespace kpsr {
class ApplicationService: public ManagedService {
public:
    ApplicationService(kpsr::Environment * environment,
        kpsr::Subscriber<SystemEventData> * systemStatusEventSubscriber,
        std::string const& serviceName)
        : ManagedService(environment, systemStatusEventSubscriber, serviceName) {}

protected:
    void start() {
        std::cout <<"Started cubesat";
    }

    void stop() {
        std::cout << "Stopping cubesat";
    }

    void execute() {}
};
}
```

We provide an example (using the previously defined ControlService) along with a unit test (example6c.cpp)
