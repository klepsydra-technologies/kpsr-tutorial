## Chapter 2

### Introduction

Klepsydra's API for DDS, ZMQ and ROS are part of the composition API presented in [chapter 1](chapter1.md). The functionality offered for each of these middlewares include:

* **Publication to Middleware**. Fulfilling the ```kpsr::Publisher``` API, Klepsydra offers an efficient way to publish to the middleware.
* **Subscription from Middleware**. The most powerful element of Klepsydra is the reception of incoming data from the middleware and passing it to the application. The associated API is explained in detail in this chapter.
* **Environment**. Klepsydra offers distributed configuration data by fulfilling the ```kpsr::Environment``` API based on middleware specific data sharing capabilities.

The original goal of these three features of the middleware integration of Klepsydra was to isolate the application code from any middleware dependencies, so that applications can:

* Have transparency over the underlying middleware
* Develop unit tests and integrations tests agnostic of the underlying middleware.

This is the recommendation of several middleware providers including [ROS](http://wiki.ros.org/Quality/Tutorials/UnitTesting#line-85).
 
### ROS

To integrate Klepsydra with ROS, we provide three packages: `kpsr_ros_core`, `kpsr_ros_serialization` and `kpsr_ros_codegen`. These three packages are in the ros module of the folders `core`, `serialization` and `code_generation` in the kpsr-core repository. The functionality provided by the first two packages is demonstrated here while the functionality provided by `kpsr_ros_codegen` is explained in Chapter 3.

The following examples show the Klepsydra - ROS connector.

#### Example 1: The "Hello world!" example.

Going back to the fist example in chapter1:

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


Let's build a simple publisher for ROS:

```cpp
#include <kpsr_ros_core/to_ros_middleware_provider.h>
#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>
#include "simple_publisher.h"

void callbackFunction(const std_msgs::String& message) {
    std::cout << "Message received: " << message.data<< std::endl;
}

int main(int argc, char ** argv) {
   ros::init(argc, argv, "kpsr_ros_publisher");
   ros::NodeHandle nodeHandle;

   std::string topicName = "tutorial_api_example1";
   ros::Publisher stringPublisher = nodeHandle.advertise<std_msgs::String>(topicName, 1);

   ros::Subscriber sub = nodeHandle.subscribe(topicName, 1000, callbackFunction);

   kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

   kpsr::Publisher<std::string> * kpsrPublisher = toRosProvider.getToMiddlewareChannel<std::string, std_msgs::String>(topicName, 1, nullptr, stringPublisher);
   SimplePublisher publisher(kpsrPublisher);
   
   simplePublisher.run();
   
   ros::SpinOnce();
}
```

This examples shows several features of Klepsydra already. Let's have a look at them line by line:

* The first five lines in `main` are standard lines in a ROS application to set up the node and define a publisher/subscriber pair.
* The `kpsr::ros_mdlw::ToRosMiddlewareProvider` is the base factory to associate the ROS middleware publisher to a Klepysydra publisher. 
* The associated Klepsydra publisher is obtained by calling the class template function `getToMiddlewareChannel`. 
```cpp
   kpsr::Publisher * kpsrPublisher = toRosProvider.getToMiddlewareChannel<std::string, std_msgs::String>(topicName, 1, nullptr, stringPublisher);

```
As ROS defines specific message types for different primitive data types in C++, we call template function with the template parameters being the primitive C++ object type and the associated ROS object type of the publisher.  The input parameters to the function are the topic name, the pool size and initializer function (in case the smart pool is to be used), and the original ROS publisher. Klepsydra has a mapper that allows for conversion between the two data types interchangeably.
* We use the `kpsr::Publisher` obtained in the previous step to create a SimplePublisher instance. With this, we will now be able to publish to ROS topics using our middleware-agnostic class SimplePublisher.
* Because of the internal mapper calls in the publisher obtained from `ToRosMiddlewareProvider`, the `std::string` message that is intended to be published by the SimplePublisher automatically gets converted to the `std_msgs::String` type.
* This example shows publishing a simple (primitive) data type for which equivalent ROS types already exists. In case you want to publish more complex C++ data structures, Klepsydra provides a code generator which can generate appropriate ROS msg types along with mapper classes. This is covered in Chapter 3(TODO: add link) of the tutorial.
* This example is like a typical ROS application. Thus, it must be compiled using `catkin`. 

#### Example 2: Reception of data

The following example shows how data coming from the middleware is placed in the eventloop. We will use the same publisher as in example 1.

```cpp
#include <kpsr_ros_core/from_ros_middleware_provider.h>
#include <kpsr_ros_core/to_ros_middleware_provider.h>
#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

int main(int argc, char ** argv) {
   ros::init(argc, argv, "kpsr_ros_subscriber");
   ros::NodeHandle nodeHandle;

   std::string topicName = "tutorial_api_example2";
   ros::Publisher stringPublisher = nodeHandle.advertise<std_msgs::String>(topicName, 1);

   kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
   kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);
   
   fromRosProvider.registerToTopic<std::string, std_msgs::String>(topicName.c_str(), 1, eventLoop.getPublisher<std::string>(topicName, 0, nullptr, nullptr));
   
   eventloop.getSubscriber<std::string>(topicName)->registerListener("example2", [](const std::string & message) {
         std::cout << "Message received: " << message << std::endl;
      }
   );
   
   eventloop.start();
   stringPublisher.publish("Hello World!");
   std::this_thread::sleep_for(std::chrono::milliseconds(100));
   
   ros::SpinOnce();
   std::this_thread::sleep_for(std::chrono::milliseconds(100));
   
   eventloop.getSubscriber<std::string>(topicName)->removeListener("example2");
   std::this_thread::sleep_for(std::chrono::milliseconds(10));
   eventloop.stop();
}
```

Let's look at the lines in the main function:
* The first four lines set up a normal ros node and a publisher.
* Next we set up a factory of incoming data connectors using the `kpsr::ros_mdlw::FromRosMiddlewareProvider`. This takes the ROS nodehandle as the input parameter:
```cpp
   kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
```
* The FromRosMiddlewareProvider provides an API to create a connector to the ROS middleware. 
```cpp
   fromRosProvider.registerToTopic<std::string, std_msgs::String>(topicName.c_str(), 1, eventLoop.getPublisher<std::string>(topicName, 0, nullptr, nullptr));
```
The `registerToTopic` expects two template parameters - one corresponding to the C++ data type and one corresponding to the respective ROS msg type. It also needs three input parameters: the topic name to subscribe to, the ROS Subscriber option of queue size, and an internal Klepsydra Publisher. We use the high performance event loop here to get access to a publisher.

Klepsydra uses it own event loop to replicate the message sent by the publisher and then, the subscriber obtained from this event loop is the one which executes the desired callback function. Once we start the event loop, a separate thread is started. The event loop with its internal ring buffer allows us to execute complicated/heavy callbacks without affecting the publisher. Internally there is a standard ROS subscriber with a callback function that just transforms the ROS msg data to corresponding C++ data type, which is a fast operation, and thus the subscriber is always ready to accept new data.

* Registering the Listener with the desired callback function:
```cpp
   eventloop.getSubscriber<std::string>(topicName)->registerListener("example2", [](const std::string & message) {
         std::cout << "Message received: " << message << std::endl;
      }
   );
```
* Start the event loop and then publish the desired message.
* Since the Klepsydra event loop has its own internal thread for handling messages, we can use the `ros::SpinOnce()` function.
* As noted in the previous chapter, we must remove the listener before stopping the event loop.
* The sleep calls are included because as this is a multi-threaded application, the sleep ensures that sufficient time is given to the function calls to terminate. If we don't give enough time to the removeListener call, the event loop may get blocked in its stop() call.

#### Example 3: A more complex example

The previous example is intended to show the most basic use, and thus might appear too counter-intuitive in demonstrating the usage of Klepsydra. As in the previous chapter, we show the example with vector data used in this case. We reuse the class SumVectorData (with modifications to use ROS Float32MultiArray type). Ideally, we should separate our application from the middleware. However for simplicity, we use the existing ROS message type in this example. Klepsydra includes an `IdentityMapper` class which allows the use of middleware-specific data types in Klepsydra.

As mentioned above, good programming practices encourage the application specific code to be independent of the middleware being used. The chapter 3 will show how you can use klepsydra to publish a data type by automatically mapping it to equivalent middleware-specific data types.


```cpp
#include <iostream>

#include "sum_vector_data.h"

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

int main() {
   ros::init(argc, argv, "kpsr_ros_subscriber");
   ros::NodeHandle nodeHandle;

   // Declare the eventloop and vector publisher
   kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);
   eventloop.start();

   // Set up the klepsydra class to publish ROS vectors.
   auto vectorPublisher = toRosProvider.getToMiddlewareChannel<std_msgs::Float32MultiArray, std_msgs::Float32MultiArray>(
       "vector", 10, nullptr, rosVectorPublisher);
      
   // Declare the ROS publisher that will publish the vector sum
   std::string vectorSum("VectorSum");
   ros::Publisher rosPublisher = nodeHandle.advertise<std_msgs::Float32>(vectorSum, 1);
   kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);
   kpsr::Publisher * sumPublisher = toRosProvider.getToMiddlewareChannel<float, std_msgs::Float32>(vectorSum, 1, nullptr, rosPublisher);

   kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
   // Register ROS subscriber for the ROS vector.
   fromRosProvider.registerToTopic<std_msgs::Float32MultiArray, std_msgs::Float32MultiArray>(
       "vector", 10, eventloop.getPublisher<std_msgs::Float32MultiArray>("vector", 10, nullptr, nullptr));
   
   {
      SumVectorData sumVectorData(
            eventloop.getSubscriber<std::vector<float>>("vector"),
            sumPublisher);

      // Define the ROS subscriber to receive the vector sum
      fromRosProvider.registerToTopic<float, std_msgs::Float32>(vectorSum.c_str(), 1, event.getPublisher<float>(vectorSum, 0, nullptr, nullptr);
      eventloop.getSubscriber<float>(vectorSum)->registerListener("sum", [](const float & message) {
            std::cout << "Sum received: " << message << std::endl;
         }
      );

      // Set up sample publisher.
      std::thread t([&vectorPublisher]() {
                          std_msgs::Float32MultiArray msg;
                          int const vectorLength(10);
                          msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  
                          msg.layout.dim[0].size = vectorLength;
                          msg.layout.dim[0].stride = 1;
                          for (int i = 0; i < 100; i ++) {
                              msg.data.clear();
                              std::vector<float> vector(vectorLength);
                              for (int j = 0; j < vectorLength; j++) {
                                  vector[j] = static_cast<float>(j+1);
                              }
                              msg.data.insert(msg.data.end(), vector.begin(), vector.end());
                              vectorPublisher->publish(msg);
                              ros::spinOnce();
                          }
                      });
      t.join();
   }
   eventloop.getSubscriber<float>(vectorSum)->removeListener("sum");
   eventloop.stop();
}

```
#### Example 4: ROS Env. 

The ROS Env class is the Environment class built to function as the wrapper to the ROS parameter server. The ROSEnv allows us to access the parameters defined by the ROS parameter server. The following example shows how read data from a ROS middleware. 

```cpp
#include <kpsr_ros_core/ros_env.h>

int main(int argc, char **argv) {
   ros::init(argc, argv, "kpsr_ros_environment");
   ros::NodeHandle nodeHandle;

   kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

   std::string rosversion;
   environment.getPropertyString("rosversion", rosversion);

   std::cout <<" The ros version is " << rosversion << std::endl;

   float paramFloat(1.5f);
   environment.setPropertyFloat("paramFloat", paramFloat);
   
   return 0;
}

```
* This example shows usage of the ros environment with default parameters that always exist and show how a new parameter can be set.
* The ROS Env is constructed using the pointer to the NodeHandle as the only required input parameter. The rest of the code is straightforward. 
* The Environment class interface can be used to get and set values. When getting/setting a parameter, you must know the type of the parameter and use the relevant getter/setter. 
* After running this example, you can verify that a new parameter `/paramFloat` exists with a value 1.5 using the `rosparam` command-line utility.

### DDS

When Klepsydra is compiled and installed with DDS support, three DDS specific libraries are installed: `kpsr_dds_serialization_datamodel`, `kpsr_dds_core` and `kpsr_dds_core_datamodel`, and are available to be linked against using the environment variable `KLEPSYDRA_DDS_LIBRARIES`. The `kpsr_dds_serialization` provides DDS compatible types for primitive data types. As DDS requires message data types to be specified in an *.idl file, Klepsydra provides the primitive data types in wrapped formats which also include the sequence number field. The `kpsr_dds_serialization` also provides mapping functions to conveniently transform data from the primitive type to the dds specific type. 

#### Sending / receiving data - the HelloWorld example:

The HelloWorld example for DDS can then use the same SimplePublisher class as shown for ROS and the complete example would be as shown below:

```cpp

int main(int argc, char **argv) {
    dds::domain::DomainParticipant dp(0);
    dds::pub::Publisher publisher(dp);
    dds::sub::Subscriber subscriber(dp);

    dds::topic::Topic<kpsr_dds_serialization::StringData> stringDataTopic(dp, "stringData");
    dds::pub::DataWriter<kpsr_dds_serialization::StringData> stringDataWriter(publisher, stringDataTopic);
    dds::sub::DataReader<kpsr_dds_serialization::StringData> stringDataReader(subscriber, stringDataTopic);

    kpsr::dds_mdlw::ToDDSMiddlewareProvider provider(nullptr);
    kpsr::dds_mdlw::FromDDSMiddlewareProvider ddsProvider;

    kpsr::Publisher<std::string> * stringDataPublisher =
            provider.getToMiddlewareChannel<std::string, kpsr_dds_serialization::StringData>("stringData", 0, nullptr, &stringDataWriter);

    SimplePublisher publisher(stringDataPublisher);

    kpsr::high_performance::EventLoopMiddlewareProvider<16> stringDataSafeQueueProvider(nullptr);
    stringDataSafeQueueProvider.start();
    ddsProvider.registerToTopic("stringData", &stringDataReader, true, stringDataSafeQueueProvider.getPublisher<std::string>("string", 0, nullptr, nullptr));

    stringDataSafeQueueProvider.getSubscriber<std::string>("string")->registerListener("example", [](std::string &msg) {
        std::cout << "Received message: " << msg << std::endl; });

    simplePublisher.run();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    stringDataSafeQueueProvider.getSubscriber<std::string>("string")->removeListener("example");
    stringDataSafeQueueProvider.stop();
    ddsProvider.unregisterFromTopic("stringData", &stringDataReader);
}
```

* The first 6 lines show the standard method for setting up a DDS domain participant with its publisher, subscriber, topic, writer and  reader.
* Next, we set up the ```kpsr::dds_mdlw::ToDDSMiddlewareProvider``` base factory for the Klepsydra publishers and the ```kpsr::dds_mdlw::FromDDSMiddlewareProvider``` base factory for the Klepsydra subscribers.
* The publisher factory gives us the Klepsydra string publisher which will map to the DDS publisher. We can use this to create the SimplePublisher class instance.
* Next we set up the Klepsydra event loop, in this case the ```kpsr::high_performance::EventLoopMiddlewareProvider``` as we are only running a simple test. Similar to examples in Chapter 1 of this tutorial, we can also use other high performance APIs that Klepsydra provides.
* ```kpsr::dds_mdlw::FromDDSMiddlewareProvider``` allows us to connect Klepsydra to DDS subscribers using the `registerToTopic` function, similar to the ROS case. While the function is a templated function, we do not need to provide the template types (i.e the C++ data type, and the DDS specific type), since those can be deduced from the input parameters. The ```kpsr::dds_mdlw::FromDDSMiddlewareProvider::registerToTopic``` function requires 4 parameters - the topic name, the associated DDS Reader, the boolean to specify whether to remove read messages from DDS queue or not, and the internal klepsydra publisher. In this case, the internal publisher is obtained from the event loop.
* Next, we attach the listener callback to the subscriber obtained from the event loop. With this, the klepsydra set up is done.
As you can see from the code, the application we intended (publish "Hello World") remained unaffected by the middleware type. The class used in the ROS example was reused in the DDS example and the Klepsydra API allowed us to easily use it with DDS. Additionally, since the event loop used is the same (whether in ROS or DDS), we get the same advantages of the high performance API.

#### DDS Env

The ```kpsr::dds_mdlw::DDSEnv``` class is the Environment class built to share and access global data parameters over the DDS distributed environment. The Environment class defines the interface used by the ```kpsr::dds_mdlw::DDSEnv``` to access the parameters. These parameters are loaded from a YAML file into a YamlEnvironment class, and are made available to the DDS environment via a specific pair of DDS Reader/Writers. As a result, these parameters are accessible over the entire DDS environment, and any changes to these parameters are also visible to Klepsydra applications.

### ZMQ
