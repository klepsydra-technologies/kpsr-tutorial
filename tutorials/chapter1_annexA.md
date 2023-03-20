<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# Chapter 1. Annex A

## Table of contents
* [Introduction](#introduction)
* [Usual function approach](#usual-function-approach)
  * [Example 1: Passing a free function](#example-1)
  * [Example 2: Passing a class member function](#example-2)
  * [Example 3: Passing a bound function](#example-3)
  * [Example 4: Passing a class function member which is bound to another one with more arguments](#example-4)
* [Lambda function approach](#lambda-function-approach)
  * [Example 5: Passing a lambda function which captures a variable by const value](#example-5)
  * [Example 6: Passing a mutable lambda function which captures a variable by value](#example-6)
  * [Example 7: Passing a lambda function which captures a variable by reference](#example-7)
  * [Example 8: Passing a lambda function which has been stored in a std::function](#example-8)

<a name="introduction"></a>
## Introduction

This annex is intended to show different examples of how to pass the listener function in the `registerListener` function. It is divided in two parts: the first one follows a usual function approach, and the second one follows a lambda function approach.

<a name="usual-function-approach"></a>
## Usual function approach

This section shows different techniques to pass a usual function as the listener function. First, a free function is passed, and later the following [`SimpleListener`](../../examples/chapter1/annexA/include/simple_listener.h) class has been used:

```cpp
template<class T>
class SimpleListener
{
public:
    SimpleListener()
        : _className("SimpleListener")
    {
        forwarderListenerFunction = std::bind(&SimpleListener<T>::listenerWithMultipleArgs,
                                              this,
                                              std::placeholders::_1,
                                              _className);
    }

    void listener(const std::string &message)
    {
        std::cout << "Message received in class method \"listener\": " << message << std::endl;
        std::cout << "Provider (subscriber) thread ID: " << std::this_thread::get_id() << std::endl;
    }

    static void staticListener(const std::string &message)
    {
        std::cout << "Message received in class method \"staticListener\": " << message
                  << std::endl;
        std::cout << "Provider (subscriber) thread ID: " << std::this_thread::get_id() << std::endl;
    }

    void listenerWithMultipleArgs(const std::string &message, const std::string className)
    {
        std::cout << "Message received in method \"listenerWithMultipleArgs\" of class "
                  << className << ": " << message << std::endl;
        std::cout << "Provider (subscriber) thread ID: " << std::this_thread::get_id() << std::endl;
    }

    std::function<void(const T &)> forwarderListenerFunction;

private:
    const std::string _className;
};
```

<a name="example-1"></a>
### Example 1: Passing a free function

The easiest form to pass a function is passing a free function. In this example the `listener` free function has been used:

```cpp
#include "simple_publisher.h"

#include <iostream>
#include <klepsydra/core/event_emitter_middleware_provider.h>

void listener(const std::string &message)
{
    std::cout << "Message received in free function \"listener\": " << message << std::endl;
    std::cout << "Provider (subscriber) thread ID: " << std::this_thread::get_id() << std::endl;
}

int main()
{
    std::cout << "Main thread ID: " << std::this_thread::get_id() << std::endl;

    kpsr::EventEmitterMiddlewareProvider<std::string> provider(nullptr,
                                                               "tutorial_app_api_example1",
                                                               0,
                                                               nullptr,
                                                               nullptr);

    SimplePublisher simplePublisher(provider.getPublisher());

    provider.getSubscriber()->registerListener("example1", listener);

    simplePublisher.run();

    provider.getSubscriber()->removeListener("example1");
}
```

This example is the equivalent free function approach of Example 1 of the chapter 1, where a lambda function is used.

<a name="example-2"></a>
### Example 2: Passing a class member function

A class member function can be passed as the listener function too. In this example, the `staticListener` method from the `SimpleListener` class has been used:

```cpp
#include "simple_listener.h"
#include "simple_publisher.h"

#include <iostream>
#include <klepsydra/core/event_emitter_middleware_provider.h>

int main()
{
    std::cout << "Main thread ID: " << std::this_thread::get_id() << std::endl;

    kpsr::EventEmitterMiddlewareProvider<std::string> provider(nullptr,
                                                               "tutorial_app_api_example2",
                                                               0,
                                                               nullptr,
                                                               nullptr);

    SimplePublisher simplePublisher(provider.getPublisher());

    provider.getSubscriber()->registerListener("example2",
                                               SimpleListener<std::string>::staticListener);

    simplePublisher.run();

    provider.getSubscriber()->removeListener("example2");
}
```

<a name="example-3"></a>
### Example 3: Passing a bound function

Functions can be bound to another ones in cases where, for example, the functionality will be the same but the number of arguments differ. In this example, the listener is registered with a function bound to the `listener` method from the `SimpleListener` class:

```cpp
#include "simple_listener.h"
#include "simple_publisher.h"

#include <functional>
#include <iostream>
#include <klepsydra/core/event_emitter_middleware_provider.h>

int main()
{
    std::cout << "Main thread ID: " << std::this_thread::get_id() << std::endl;

    kpsr::EventEmitterMiddlewareProvider<std::string> provider(nullptr,
                                                               "tutorial_app_api_example3",
                                                               0,
                                                               nullptr,
                                                               nullptr);

    SimplePublisher simplePublisher(provider.getPublisher());
    SimpleListener<std::string> simpleListener;

    provider.getSubscriber()->registerListener("example3",
                                               std::bind(&SimpleListener<std::string>::listener,
                                                         &simpleListener,
                                                         std::placeholders::_1));

    simplePublisher.run();

    provider.getSubscriber()->removeListener("example3");
}
```

The relevant control flow of this approach is as follows:

1. The listener is registered with an implicit function bound to the `listener` method of the `SimpleListener` class. This is achieved by using the `std::bind` function with the following parameters:
    * `&SimpleListener<std::string>::listener`: Reference to the `listener` method.
    * `&simpleListener`: Pointer to the implicit `SimpleListener` object, which is the hidden first parameter of the non-static member functions.
    * `std::placeholders::_1`: First argument passed to the `listener` method, which is a placeholder that will be replaced by the first argument passed to the implicit function of the listener.

<a name="example-4"></a>
### Example 4: Passing a class function member which is bound to another one with more arguments

As stated previously, a function can be bound to another one even if they differ in the number of arguments. In this example, the `forwarderListenerFunction` function from the `SimpleListener` class (with 1 argument), which is bound to the `listenerWithMultipleArgs` method of the same class (with 2 arguments), has been used:

```cpp
#include "simple_listener.h"
#include "simple_publisher.h"

#include <iostream>
#include <klepsydra/core/event_emitter_middleware_provider.h>

int main()
{
    std::cout << "Main thread ID: " << std::this_thread::get_id() << std::endl;

    kpsr::EventEmitterMiddlewareProvider<std::string> provider(nullptr,
                                                               "tutorial_app_api_example4",
                                                               0,
                                                               nullptr,
                                                               nullptr);

    SimplePublisher simplePublisher(provider.getPublisher());
    SimpleListener<std::string> simpleListener;

    provider.getSubscriber()->registerListener("example4", simpleListener.forwarderListenerFunction);

    simplePublisher.run();

    provider.getSubscriber()->removeListener("example4");
}
```

The relevant control flow of this approach is as follows:

1. The `SimpleListener` class is instantiated with the `simpleListener` object and its constructor is executed. 
2. In the `SimpleListener` constructor, the `forwarderListenerFunction` function is bound to the `listenerWithMultipleArgs` method. This is achieved by using the `std::bind` function with the following parameters:
    * `&SimpleListener<T>::listenerWithMultipleArgs`: Reference to the `listenerWithMultipleArgs` method.
    * `this`: Pointer to the implicit `SimpleListener` object, which is the hidden first parameter of the non-static member functions.
    * `std::placeholders::_1`: First argument passed to the `listenerWithMultipleArgs` method, which is a placeholder that will be replaced by the first argument passed to the `forwarderListenerFunction` function.
    * `_className`: The private member of the class used to show that a function can be bound to another one with different number of arguments.
3. The listener is registered with the `forwarderListenerFunction` function.
4. When the listener function is called, the `forwarderListenerFunction` function acts as a wrapper and calls the `listenerWithMultipleArgs` method.

<a name="lambda-function-approach"></a>
## Lambda function approach

This section shows different techniques to pass a lambda function as the listener function, focusing mostly on how variables can be captured.

Lambda functions live in the scope they are defined, but by default do not have access to the variables of its scope. To allow the access, they need to explicitly specify the particular variables in the **captures** list, which is surrounded by square brackets [ ].

<a name="example-5"></a>
### Example 5: Passing a lambda function which captures a variable by const value

By default, lambda functions capture the variables by constant value:

```cpp
#include "simple_publisher.h"
#include <iostream>

#include <klepsydra/core/event_emitter_middleware_provider.h>
int main()
{
    std::cout << "Main thread ID: " << std::this_thread::get_id() << std::endl;

    kpsr::EventEmitterMiddlewareProvider<std::string> provider(nullptr,
                                                               "tutorial_app_api_example4",
                                                               0,
                                                               nullptr,
                                                               nullptr);

    SimplePublisher simplePublisher(provider.getPublisher());
    std::string captured_message = "Hello World! I am a captured message!";

    provider.getSubscriber()
        ->registerListener("example4", [captured_message](const std::string &message) {
            std::cout << "Message captured by the lambda: " << captured_message << std::endl;
            std::cout << "Message received: " << message << std::endl;
            std::cout << "Provider (subscriber) thread ID: " << std::this_thread::get_id()
                      << std::endl;
        });

    captured_message = "New value of captured message";
    std::cout << captured_message << std::endl;
    simplePublisher.run();

    provider.getSubscriber()->removeListener("example4");
}
```

The relevant data flow of this approach is as follows:

1. The `captured_message` variable is initialized with the `"Hello World! I am a captured message!"` value.
2. The listener is registered with a lambda function which captures the `captured_message` variable. Since no additional specifiers have been provided, by default it is captured by constant value and therefore can not be modified inside the lambda function.
3. Outside the lambda function, the `captured_message` is updated with the `"New value of captured message"` value and printed to standard output.
4. The `simplePublisher` is run. Since the lambda function has captured the `captured_message` variable before its modification at point 3, it will print its old value to standard output when it is executed.


<a name="example-6"></a>
### Example 6: Passing a mutable lambda function which captures a variable by value

To remove the default constant qualifier from the variables captured by value by the lambda functions, the `mutable` specifier can be used. The following example is similar to the previous one but adding the `mutable` specifier:

```cpp
#include "simple_publisher.h"
#include <iostream>

#include <klepsydra/core/event_emitter_middleware_provider.h>
int main()
{
    std::cout << "Main thread ID: " << std::this_thread::get_id() << std::endl;

    kpsr::EventEmitterMiddlewareProvider<std::string> provider(nullptr,
                                                               "tutorial_app_api_example5",
                                                               0,
                                                               nullptr,
                                                               nullptr);

    SimplePublisher simplePublisher(provider.getPublisher());
    std::string captured_message = "Hello World! I am a captured message!";

    provider.getSubscriber()
        ->registerListener("example5", [captured_message](const std::string &message) mutable {
            std::cout << "Message captured by the lambda: " << captured_message << std::endl;
            captured_message = "Since I am in a mutable lambda, my message can be modified, but "
                               "only in the lambda scope!";
            std::cout << "Message captured by the lambda: " << captured_message << std::endl;
            std::cout << "Message received: " << message << std::endl;
            std::cout << "Provider (subscriber) thread ID: " << std::this_thread::get_id()
                      << std::endl;
        });

    simplePublisher.run();

    provider.getSubscriber()->removeListener("example5");
    std::cout << "Outside the lambda, the captured message keeps its original value: "
              << captured_message << std::endl;
}
```

The relevant data flow of this approach is as follows:

1. The `captured_message` variable is initialized with the `"Hello World! I am a captured message!"` value.
2. The listener is registered with a lambda function which captures the `captured_message` variable by value. Since the `mutable` specifier has been provided, the constant qualifier of `captured_message` is removed and therefore the lambda function can modify its value, but it will have effect only inside the lambda function's scope.
3. The `simplePublisher` is run and the `captured_message` variable is printed to standard output inside the lambda function with the updated value. 
4. Finally, outside the lambda function the `captured_message` variable is printed to standard output with its old value, showing that its update done in the lambda function is bound to the lambda's scope.

<a name="example-7"></a>
### Example 7: Passing a lambda function which captures a variable by reference

Lambda functions can capture variables by reference by adding the **&** symbol. Therefore, any modifications of these variables will be present outside the lambda's scope too. The following example illustrates this behaviour:

```cpp
#include "simple_publisher.h"
#include <iostream>

#include <klepsydra/core/event_emitter_middleware_provider.h>
int main()
{
    std::cout << "Main thread ID: " << std::this_thread::get_id() << std::endl;

    kpsr::EventEmitterMiddlewareProvider<std::string> provider(nullptr,
                                                               "tutorial_app_api_example6",
                                                               0,
                                                               nullptr,
                                                               nullptr);

    SimplePublisher simplePublisher(provider.getPublisher());
    std::string captured_message = "Hello World! I am a captured message!";

    provider.getSubscriber()
        ->registerListener("example6", [&captured_message](const std::string &message) {
            std::cout << "Message captured by the lambda: " << captured_message << std::endl;
            captured_message = "Since I am reference captured in the lambda, my message is "
                               "modified in the outside scope too!";
            std::cout << "Message captured by the lambda: " << captured_message << std::endl;
            std::cout << "Message received: " << message << std::endl;
            std::cout << "Provider (subscriber) thread ID: " << std::this_thread::get_id()
                      << std::endl;
        });

    simplePublisher.run();

    provider.getSubscriber()->removeListener("example6");
    std::cout << "Outside the lambda, the captured message has been modified too: "
              << captured_message << std::endl;
}
```

The relevant data flow of this approach is as follows:

1. The `captured_message` variable is initialized with the `"Hello World! I am a captured message!"` value.
2. The listener is registered with a lambda function which captures the `captured_message` variable by reference. 
3. The `simplePublisher` is run and the `captured_message` variable is printed to standard output inside the lambda function with the updated value. 
4. Finally, outside the lambda function the `captured_message` variable is printed to standard output with its new value, showing that the update done in the captured variable is propagated to the original variable.

<a name="example-8"></a>
### Example 8: Passing a lambda function which has been stored in a std::function

As a further step, lambda functions can have a name by storing them in lambda variables. This can be useful to make the code more readable, or in the cases where the lambda function will be used more than once. The following example is similar to the Example 5, but the lambda function has been stored in a std::function:

```cpp
#include "simple_publisher.h"
#include <functional>
#include <iostream>

#include <klepsydra/core/event_emitter_middleware_provider.h>
int main()
{
    std::cout << "Main thread ID: " << std::this_thread::get_id() << std::endl;

    kpsr::EventEmitterMiddlewareProvider<std::string> provider(nullptr,
                                                               "tutorial_app_api_example7",
                                                               0,
                                                               nullptr,
                                                               nullptr);

    SimplePublisher simplePublisher(provider.getPublisher());
    const std::string captured_message = "Hello World! I am a captured message!";

    const std::function<void(const std::string)> listener_function{[captured_message](
                                                                       const std::string &message) {
        std::cout << "Message captured by the lambda: " << captured_message << std::endl;
        std::cout << "Message received: " << message << std::endl;
        std::cout << "Provider (subscriber) thread ID: " << std::this_thread::get_id() << std::endl;
    }};

    provider.getSubscriber()->registerListener("example7", listener_function);
    simplePublisher.run();

    provider.getSubscriber()->removeListener("example7");
}
```

