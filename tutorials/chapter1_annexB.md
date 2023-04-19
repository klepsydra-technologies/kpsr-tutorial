<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# Chapter 1 - Annex B: Publisher and subscriber arguments

- [Example B1a: Concurrent publisher and slow listener](#example-b1a-concurrent-publisher-and-slow-listener)
    - [Source code](#source-code)
    - [Sample output](#sample-output)
- [Example B1b: Publishing by reference](#example-b1b-publishing-by-reference)
    - [Sample output](#sample-output-1)
- [Example B1c: Adding the smart loop](#example-b1c-adding-the-smart-loop)
    - [Sample output](#sample-output-2)
- [Example B1d: Adding an initializer](#example-b1d-adding-an-initializer)
    - [Sample output](#sample-output-3)
- [Example B1e: Adding a cloner](#example-b1e-adding-a-cloner)
    - [Sample output](#sample-output-4)
- [Example B2a: Small pool size](#example-b2a-small-pool-size)
    - [Source code](#source-code-1)
    - [Sample output](#sample-output-5)
- [Example B2b: Small buffer size](#example-b2b-small-buffer-size)
    - [Sample output](#sample-output-6)
- [Example B3: Data multiplexer](#example-b3-data-multiplexer)
    - [Source code](#source-code-2)
    - [Sample output](#sample-output-7)

This chapter explains different event loop behaviors depending on the publisher's options. Additional examples about publishing by reference, small loop size, and small buffer size effects. Finally, an example of the differences and features of the data multiplexer.

## Example B1a: Concurrent publisher and slow listener

The publisher thread, just after the message has been published, modifies the message values. In the meanwhile, the listener takes a long time (simulated by a 1-second sleep) to process its own _copy_ of the message.

### Source code

See `kpsr-tutorial/examples/chapter1/annexB/src/annexB1.cpp` and the `annexB/include/` folder.
A simple publisher sends, from its own thread, 3x `TestEvent` to a slow listener. The class `TestEvent` counts each constructor type invocation.

```C++
#include <klepsydra/high_performance/event_loop_middleware_provider.h>

#include "helper_functions.h"
#include "simple_event_publisher.h"
#include "slow_listener.h"
#include "test_event.h"

const int bufferSize{16};
const int poolSize{0};
const auto initializerFunction{nullptr};
const auto eventClonerFunction{nullptr};
const int publishIterations = 3;

int main(int argc, char *argv[])
{
    resetInvocationsCounters();

    {
        std::cout << "Main thread " << std::this_thread::get_id() << std::endl;

        kpsr::high_performance::EventLoopMiddlewareProvider<bufferSize> eventLoop(nullptr);
        printInvocations("event loop creation");

        kpsr::Publisher<TestEvent> *kpsrPublisher =
            eventLoop.getPublisher<TestEvent>("TestEvent",
                                              poolSize,
                                              initializerFunction,
                                              eventClonerFunction);
        printInvocations("getPublisher");

        kpsr::Subscriber<TestEvent> *kpsrSubscriber = eventLoop.getSubscriber<TestEvent>(
            "TestEvent");
        printInvocations("getSubscriber");

        SimpleEventPublisher testPublisher(kpsrPublisher);
        printInvocations("testPublisher");

        SlowListener testListener(*kpsrSubscriber, "slowListener");
        printInvocations("testListener");

        eventLoop.start();
        std::thread concurrentDataUpdaterThread([&testPublisher]() {
            for (int i = 1; i <= publishIterations; i++) {
                TestEvent event(i, "hello");
                std::cout << "SimpleEventPublisher is about to receive TestEvent(" << event.id
                          << "," << event.message << ")\n\tat addr " << &event << "\n\ton thread "
                          << std::this_thread::get_id() << std::endl;
                testPublisher.run(event);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                event.id = -i;
                event.message = "bye";
            }
        });
        concurrentDataUpdaterThread.join();
        printInvocations("publishing");

        while (kpsrSubscriber->getSubscriptionStats("slowListener")->totalProcessed <
               publishIterations) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        printInvocations("slow listeners finished");

        eventLoop.stop();
        printInvocations("stopping event loop");
    }
    printInvocations("program finished");

    return 0;
}
```

### Sample output

The console shows:

- the running thread,
- the number of constructors' invocations after every step, and
- the messages data, `TestEvent(<int>, <string>)`, and its storage address.

```text
$ bin/kpsr_tutorial_chapter1_annexB_1a

Main thread 139959112914816
Invocations counters updating after event loop creation
        (none)
Invocations counters updating after getPublisher
        (none)
Invocations counters updating after getSubscriber
        (none)
Invocations counters updating after testPublisher
        (none)
Invocations counters updating after testListener
        (none)
SimpleEventPublisher is about to receive TestEvent(1,hello)
        at addr 0x7f4ac4b4fd70
        on thread 139959104505600
SimpleEventPublisher publishes TestEvent(1,hello)
        at addr 0x7f4ac4b4fd70
        on thread 139959104505600
SlowListener (slowListener) received TestEvent(1,hello)
        at addr 0x7f4ab8000b70
        on thread 139959112898304
SimpleEventPublisher is about to receive TestEvent(2,hello)
        at addr 0x7f4ac4b4fd70
        on thread 139959104505600
SimpleEventPublisher publishes TestEvent(2,hello)
        at addr 0x7f4ac4b4fd70
        on thread 139959104505600
SimpleEventPublisher is about to receive TestEvent(3,hello)
        at addr 0x7f4ac4b4fd70
        on thread 139959104505600
SimpleEventPublisher publishes TestEvent(3,hello)
        at addr 0x7f4ac4b4fd70
        on thread 139959104505600
Invocations counters updating after publishing
        constructorInvocations      3
        copyInvocations             3
        destructorInvocations       3
SlowListener (slowListener) finished, now TestEvent(1,hello)
SlowListener (slowListener) received TestEvent(2,hello)
        at addr 0x7f4ab8000bb0
        on thread 139959112898304
SlowListener (slowListener) finished, now TestEvent(2,hello)
SlowListener (slowListener) received TestEvent(3,hello)
        at addr 0x7f4ab8000bf0
        on thread 139959112898304
SlowListener (slowListener) finished, now TestEvent(3,hello)
Invocations counters updating after slow listeners finished
        destructorInvocations       6
[2023-03-27 13:42:03.486] [info] Halting the batchEventProcessor
Invocations counters updating after stopping event loop
        (none)
Invocations counters updating after program finished
        (none)
```

**Threads.**

- One for the main program (139959112914816).
- One executed for publishing 3x messages in a row (139959104505600).
- One for the slow listener (139959112898304).

**TestEvent constructors.**

None of the first event loop steps modify the event data. Just after the publishing of `(1,hello)`, `(2,hello)`, and `(3,hello)`:

- 3x `constructorInvocations`, one for every message. Note that the loop reuses the same address (0x7f4ac4b4fd70).
- 3x `copyInvocations`, when the data was passed _by value_ to the listener (note the different address 0x7f4ab8000b70, etc.).
As such, the publisher changing of its data does not affect to the listeners'.\
Note this is a kpsr internal copy, don't be confused with the listener registration (reference `&event`) at `include/slow_listener.h`

```C++
_subscriber.registerListener(listenerName, [](const TestEvent &event) {
```

- 3x `destructorInvocations`, one for each `constructorInvocations` after the publishing.

When the listeners finish,

- 6x `destructorInvocations`: the number was increased with the 3x `copyInvocations`.
So, in this example:

<p align="center">
constructorInvocations + copyInvocations = destructorInvocations
</p>

## Example B1b: Publishing by reference

This example shares most of the code with example B1a:

```diff
- TestEvent event(i, "hello");
- testPublisher.run(event);
+ std::shared_ptr<TestEvent> eventPtr = std::make_shared<TestEvent>(i, "hello");
+ testPublisher.runWithoutCopy(eventPtr);
```

The message is published as a shared pointer to the event data, instead of copying it. This will increase the performance but introduces the risk of data corruption while the listeners are running:

### Sample output

Changes compared with example B1a:

```text
$ bin/kpsr_tutorial_chapter1_annexB_1b

SimpleEventPublisher publishes TestEvent(1,hello) without copy
        at addr 0x7f2c6c000b70
        on thread 139828901443328
SlowListener (slowListener) received TestEvent(1,hello)
        at addr 0x7f2c6c000b70
        on thread 139828979021568
SimpleEventPublisher publishes TestEvent(2,hello) without copy
        at addr 0x7f2c6c000bb0
        on thread 139828901443328
SimpleEventPublisher publishes TestEvent(3,hello) without copy
        at addr 0x7f2c6c000bf0
        on thread 139828901443328

Invocations counters updating after publishing
        constructorInvocations      3
SlowListener (slowListener) finished, now TestEvent(-1,bye)

SlowListener (slowListener) received TestEvent(-2,bye)
        at addr 0x7f2c6c000bb0
        on thread 139828979021568
SlowListener (slowListener) finished, now TestEvent(-2,bye)
SlowListener (slowListener) received TestEvent(-3,bye)
        at addr 0x7f2c6c000bf0
        on thread 139828979021568
SlowListener (slowListener) finished, now TestEvent(-3,bye)
Invocations counters updating after slow listeners finished
        destructorInvocations       3
```

**Threads.**

Same 3 threads as in the previous example: main, publisher, and listener.

**TestEvent constructors.**

Just after the publishing:

- 3x `constructorInvocations` as before. Not any other constructor or destructor until,

When the listeners finish:

- 3x `destructorInvocations`.

<p align="center">
constructorInvocations = destructorInvocations
</p>

**Data update.**

- Every published data has its own address, which is _shared_ by the listener.
- `TestEvent(1,hello)`, always at 0x7f2c6c000b70, is modified by the publisher and its listener ends with `TestEvent(-1,bye)`.
- `TestEvent(2,hello)` are changed even before the next listener starts, so `TestEvent(-2,bye)` if processed from begin to end.
- Same for `TestEvent(3,hello)`and `TestEvent(-3,bye)`.
  Note that the events do not share their address, so their values (even by reference) do not interfere with each other.

## Example B1c: Adding the smart loop

The code difference with example B1a is minimal:

```diff
- const int poolSize{0};
+ const int poolSize{8};
```

```C++
    kpsrPublisher = eventLoop.getPublisher<TestEvent>("TestEvent",
                                                      poolSize,
                                                      initializerFunction,
                                                      eventClonerFunction);
```

Any non-zero pool size when getting the publisher enables the Klepsydra smart pool: this is a memory allocation that improves publish efficiency. See the [Smart Memory Pool](./chapter1.md#smart-memory-pool) on the main tutorial for additional details.

### Sample output

These are the main differences compared with example B1a:

```text
$ bin/kpsr_tutorial_chapter1_annexB_1c

Invocations counters updating after getPublisher
        emptyConstructorInvocations 8
Invocations counters updating after publishing
        constructorInvocations      3
        destructorInvocations       3
Invocations counters updating after slow listeners finished
        (none)
Invocations counters updating after program finished
        destructorInvocations       11
```

**Threads.**

Same 3 threads as in the previous example: main, publisher, and listener.

**TestEvent constructors.**

When the publisher is created:

- 8x `emptyConstructorInvocations`: the smart memory pool are populated at the selected size.

Just after the publishing:

- same 3x `constructorInvocations` and 3x `destructorInvocations` but,
- 0x `copyInvocations`, because listener data was already allocated at the 8x pool.
Go to examples [B2a](#example-b2a-small-pool-size) and [B2b](#example-b2b-small-buffer-size) to study a smaller pool.

When the listeners finish,

- `destructorInvocations` keeps the same.

When the program finished,

- 11x `destructorInvocations`, the 8 elements in the pool are freed.

<p align="center">
emptyConstructorInvocations + constructorInvocations = destructorInvocations
</p>

## Example B1d: Adding an initializer

One step further example B1c:

```diff
- const auto initializerFunction{nullptr};
+ const auto initializerFunction{[](TestEvent &event) { event.init(); }};
```

```C++
    kpsrPublisher = eventLoop.getPublisher<TestEvent>("TestEvent",
                                                      poolSize,
                                                      initializerFunction,
                                                      eventClonerFunction);
```

Once the smart pool allocates all the objects, the `init()` function can allocate the required memory for every object. In this example `init()` is a dummy function but could be used e.g. to resize a `std::vector` as explained in [Smart Memory Pool](./chapter1.md#smart-memory-pool). This prevents memory allocation on every publish event, increasing the application performance.

### Sample output

Differences with example B1c:

```text
$ bin/kpsr_tutorial_chapter1_annexB_1d

Invocations counters updating after getPublisher
        emptyConstructorInvocations 8
        initInvocations             8
```

**TestEvent constructors.**

When the publisher is created:

- 8x `emptyConstructorInvocations`: the smart memory pool is populated as before and,
- 8x `initInvocations`, every object is initialized to allocate its required memory.

## Example B1e: Adding a cloner

One additional step from example B1d:

```diff
- const auto eventClonerFunction{nullptr};
+ const auto eventClonerFunction{[](const TestEvent &src, TestEvent &dst) { dst = TestEvent(src); }};
```

```C++
    kpsrPublisher = eventLoop.getPublisher<TestEvent>("TestEvent",
                                                      poolSize,
                                                      initializerFunction,
                                                      eventClonerFunction);
```

Klepsydra, by default, copies the publisher objects to the subscriber with the equal (=) operator. This makes a copy _by value_ for simple objects and vectors, but not for complex data structures such as images or `cv::Mat`.\
If the data you are publishing needs a different operation for deep copies, prevent undesired effects by:

1. providing an equal (=) operator for the custom class which do deep copies, or
2. provide a cloner function. In this example, the copy constructor at `include/test_event.h` performs this operation (although not required for this simple object).

```C++
    TestEvent(const TestEvent &that)
```

Refer to the [Klepsydra vision tutorial](https://github.com/klepsydra-technologies/kpsr-vision-ocv-tutorial) for additional information.

### Sample output

Differences with example B1d:

```text
$ bin/kpsr_tutorial_chapter1_annexB_1e

Invocations counters updating after publishing
        constructorInvocations      3
        copyInvocations             3
        destructorInvocations       6
Invocations counters updating after program finished
        destructorInvocations       14
```

**TestEvent constructors.**

Just after the publishing:

- same 3x `constructorInvocations` but,
- 3x `copyInvocations`, as in example B1a; these copies are explicit.
- 6x `destructorInvocations`, equals the last invocations.

When the program finished,

- 14x `destructorInvocations`, after the 8 elements in the pool freeing.

<p align="center">
emptyConstructorInvocations + constructorInvocations + copyInvocations = destructorInvocations
</p>

## Example B2a: Small pool size

This example shows the behavior of the smart data pool when the published events exceed its size.

### Source code

```C++
#include <klepsydra/high_performance/event_loop_middleware_provider.h>

#include "helper_functions.h"
#include "simple_event_publisher.h"
#include "slow_listener.h"
#include "test_event.h"

const int bufferSize{16};
const int poolSize{4};
const auto initializerFunction{nullptr};
const auto eventClonerFunction{nullptr};
const int publishIterations = 6;

int main(int argc, char *argv[])
{
    resetInvocationsCounters();

    {
        std::cout << "Main thread " << std::this_thread::get_id() << std::endl;

        kpsr::high_performance::EventLoopMiddlewareProvider<bufferSize> eventLoop(nullptr);
        printInvocations("event loop creation");

        kpsr::Publisher<TestEvent> *kpsrPublisher =
            eventLoop.getPublisher<TestEvent>("TestEvent",
                                              poolSize,
                                              initializerFunction,
                                              eventClonerFunction);
        auto eventLoopPublisher =
            static_cast<kpsr::high_performance::EventLoopPublisher<TestEvent, bufferSize> *>(
                kpsrPublisher);
        printInvocations("getPublisher");

        kpsr::Subscriber<TestEvent> *kpsrSubscriber = eventLoop.getSubscriber<TestEvent>(
            "TestEvent");
        printInvocations("getSubscriber");

        SimpleEventPublisher fastPublisher(kpsrPublisher);
        printInvocations("fastPublisher");

        SlowListener testListener(*kpsrSubscriber, "slowListener");
        printInvocations("testListener");

        eventLoop.start();
        int publishIter{1};
        do {
            std::thread fastThread([&fastPublisher, &publishIter]() {
                for (; publishIter <= publishIterations; publishIter++) {
                    TestEvent event(publishIter, "hello");
                    std::cout << "SimpleEventPublisher is about to receive TestEvent(" << event.id
                              << "," << event.message << ")\n\tat addr " << &event
                              << "\n\ton thread " << std::this_thread::get_id() << std::endl;
                    fastPublisher.run(event);
                }
            });
            fastThread.join();
            printInvocations("publishing");

            auto publishDiscarded = eventLoopPublisher->_discardedMessages;
            while (kpsrSubscriber->getSubscriptionStats("slowListener")->totalProcessed +
                       publishDiscarded <
                   publishIterations) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            printInvocations("slow listeners finished");

            std::cout << "\nDiscarded messages were " << publishDiscarded;
            if (publishDiscarded > 0) {
                std::cout << ", retrying";
            }
            std::cout << std::endl;

            publishIter -= publishDiscarded;
            eventLoopPublisher->_discardedMessages = 0;
        } while (publishIter <= publishIterations);

        eventLoop.stop();
        printInvocations("stopping event loop");
    }
    printInvocations("program finished");

    return 0;
}
```

- The `eventLoopPublisher` is a publisher cast with additional information about the discarded messages in the event loop, and
- a do-while loop retries these discarded messages (if any) after all the fulfilled listeners finish.

### Sample output

```text
$ bin/kpsr_tutorial_chapter1_annexB_2a

Main thread 140218436257664
Invocations counters updating after event loop creation
        (none)
Invocations counters updating after getPublisher
        emptyConstructorInvocations 4
Invocations counters updating after getSubscriber
        (none)
Invocations counters updating after fastPublisher
        (none)
Invocations counters updating after testListener
        (none)
SimpleEventPublisher is about to receive TestEvent(1,hello)
        at addr 0x7f8725952d80
        on thread 140218427848448
SimpleEventPublisher publishes TestEvent(1,hello)
        at addr 0x7f8725952d80
        on thread 140218427848448
SimpleEventPublisher is about to receive TestEvent(2,hello)
        at addr 0x7f8725952d80
        on thread 140218427848448
SimpleEventPublisher publishes TestEvent(2,hello)
        at addr 0x7f8725952d80
        on thread 140218427848448
SlowListener (slowListener) received TestEvent(1,hello)
        at addr 0x561177ff3de0
        on thread 140218436241152
SimpleEventPublisher is about to receive TestEvent(3,hello)
        at addr 0x7f8725952d80
        on thread 140218427848448
SimpleEventPublisher publishes TestEvent(3,hello)
        at addr 0x7f8725952d80
        on thread 140218427848448
SimpleEventPublisher is about to receive TestEvent(4,hello)
        at addr 0x7f8725952d80
        on thread 140218427848448
SimpleEventPublisher publishes TestEvent(4,hello)
        at addr 0x7f8725952d80
        on thread 140218427848448
SimpleEventPublisher is about to receive TestEvent(5,hello)
        at addr 0x7f8725952d80
        on thread 140218427848448
[2023-03-28 21:07:08.412] [error] Cannot acquire object from an empty pool TestEvent.
[2023-03-28 21:07:08.412] [info] ObjectPoolPublisher::internalPublish. Object Pool failure. TestEvent
SimpleEventPublisher publishes TestEvent(5,hello)
        at addr 0x7f8725952d80
        on thread 140218427848448
SimpleEventPublisher is about to receive TestEvent(6,hello)
        at addr 0x7f8725952d80
        on thread 140218427848448
[2023-03-28 21:07:08.412] [error] Cannot acquire object from an empty pool TestEvent.
[2023-03-28 21:07:08.412] [info] ObjectPoolPublisher::internalPublish. Object Pool failure. TestEvent
SimpleEventPublisher publishes TestEvent(6,hello)
        at addr 0x7f8725952d80
        on thread 140218427848448
Invocations counters updating after publishing
        constructorInvocations      6
        emptyConstructorInvocations 6
        destructorInvocations       6
SlowListener (slowListener) finished, now TestEvent(1,hello)
SlowListener (slowListener) received TestEvent(2,hello)
        at addr 0x561177ff3db0
        on thread 140218436241152
SlowListener (slowListener) finished, now TestEvent(2,hello)
SlowListener (slowListener) received TestEvent(3,hello)
        at addr 0x561177ff3d80
        on thread 140218436241152
SlowListener (slowListener) finished, now TestEvent(3,hello)
SlowListener (slowListener) received TestEvent(4,hello)
        at addr 0x561177ff3d50
        on thread 140218436241152
SlowListener (slowListener) finished, now TestEvent(4,hello)
SlowListener (slowListener) received TestEvent(5,hello)
        at addr 0x7f8718000fe0
        on thread 140218436241152
SlowListener (slowListener) finished, now TestEvent(5,hello)
SlowListener (slowListener) received TestEvent(6,hello)
        at addr 0x7f8718001020
        on thread 140218436241152
SlowListener (slowListener) finished, now TestEvent(6,hello)
Invocations counters updating after slow listeners finished
        destructorInvocations       8

Discarded messages were 0
[2023-03-28 21:07:14.413] [info] Halting the batchEventProcessor
Invocations counters updating after stopping event loop
        (none)
Invocations counters updating after program finished
        destructorInvocations       12
```

**Threads.**

Same 3 threads as in previous examples: main, publisher, and listener.

**TestEvent constructors.**

When the publisher is created:

- 4x `emptyConstructorInvocations`: the smart memory pool are populated at the selected size.

Just after the publishing of `(1,hello)`, ..., `(6,hello)`:

- 6x `constructorInvocations`, one for every message.
- 6x `emptyConstructorInvocations`, 2 additional for the loop buffer (not in the pool).
- 6x `destructorInvocations`, again one for every published message.

When the listeners finish,

- 8x `destructorInvocations`, +2 on the buffer.

When the program finished,

- 12x `destructorInvocations`, +4 from the event loop.

**Data update.**

- Fourth and fifth events overflow the 4-sized data pool, and 2x error messages rise.
- Since the pool _buffer_ size is 16, these events are allocated and correctly sent to the listeners as soon as pool slots are freed.\
  Note that no data is mixed, overwritten, or lost even after the smart data pool is full.

## Example B2b: Small buffer size

The event buffer size is reduced, while the event loop size stays small:

```diff
- const int bufferSize{16};
+ const int bufferSize{4};
```

```C++
const int poolSize{4};
const int publishIterations = 6;

kpsr::high_performance::EventLoopMiddlewareProvider<bufferSize> eventLoop(nullptr);
kpsrPublisher = eventLoop.getPublisher<TestEvent>("TestEvent",
                                                  poolSize,
                                                  initializerFunction,
                                                  eventClonerFunction);
```

### Sample output

Differences with example B2a:

```text
$ bin/kpsr_tutorial_chapter1_annexB_2b

SimpleEventPublisher is about to receive TestEvent(5,hello)
[2023-03-30 11:48:38.994] [error] Cannot acquire object from an empty pool TestEvent.
[2023-03-30 11:48:39.143] [info] ObjectPoolPublisher::internalPublish. Object Pool failure. TestEvent
[2023-03-30 11:48:39.143] [info] EventLoopPublisher::internalPublish. no more capacity.TestEvent

SimpleEventPublisher is about to receive TestEvent(6,hello)
[2023-03-30 11:48:39.143] [error] Cannot acquire object from an empty pool TestEvent.
[2023-03-30 11:48:39.143] [info] ObjectPoolPublisher::internalPublish. Object Pool failure. TestEvent
[2023-03-30 11:48:39.143] [info] EventLoopPublisher::internalPublish. no more capacity.TestEvent

Invocations counters updating after publishing
        constructorInvocations      6
        emptyConstructorInvocations 6
        destructorInvocations       8
Invocations counters updating after slow listeners finished
        (none)
Discarded messages were 2, retrying

Invocations counters updating after publishing
        constructorInvocations      8
        destructorInvocations       10
Invocations counters updating after slow listeners finished
        (none)
Discarded messages were 0

Invocations counters updating after program finished
        destructorInvocations       14
```

**TestEvent constructors.**

After the publishing:

- Same `constructorInvocations` and `emptyConstructorInvocations`.
- 8x `destructorInvocations`, increased with the 2 discarded messages.\
  This increment is propagated to the second attempt to publish the pending messages.

When the listeners finish:

- No additional `destructorInvocations` as both buffer and pool data were fully allocated.

**Data update.**

- Both the data pool and the buffer overflow at the same time, logging a third info line.
- The last 2 messages are _lost_ while the slow listeners are running.\
  As before, no data is mixed or overwritten in the smart pool.
- Once the listeners finish, the 2 pending messages are published again and processed successfully as usual.

## Example B3: Data multiplexer

The `EventLoopMiddlewareProvider` from previous examples is replaced with `DataMultiplexerMiddlewareProvider`. This implies several differences and advantages; see [Example 5: The data multiplexer](./chapter1.md#example-5-the-data-multiplexer) for a detailed explanation.

### Source code

```C++
#include <klepsydra/high_performance/data_multiplexer_middleware_provider.h>

#include "helper_functions.h"
#include "simple_event_publisher.h"
#include "slow_listener.h"
#include "test_event.h"

const int bufferSize{16};
const auto initializerFunction{nullptr};
const auto eventClonerFunction{nullptr};
const int publishIterations = 3;

int main(int argc, char *argv[])
{
    resetInvocationsCounters();

    {
        std::cout << "Main thread " << std::this_thread::get_id() << std::endl;

        kpsr::high_performance::DataMultiplexerMiddlewareProvider<TestEvent, bufferSize>
            dataMultiplexer(nullptr, "TestEvent", initializerFunction, eventClonerFunction);
        printInvocations("data multiplexer creation");

        kpsr::Publisher<TestEvent> *kpsrPublisher = dataMultiplexer.getPublisher();
        printInvocations("getPublisher");

        kpsr::Subscriber<TestEvent> *kpsrSubscriber1 = dataMultiplexer.getSubscriber("subscriber1");
        printInvocations("the first getSubscriber");
        kpsr::Subscriber<TestEvent> *kpsrSubscriber2 = dataMultiplexer.getSubscriber("subscriber2");
        printInvocations("the second getSubscriber");

        SimpleEventPublisher testPublisher(kpsrPublisher);
        printInvocations("testPublisher");

        SlowListener testListener1(*kpsrSubscriber1, "slowListener1");
        printInvocations("testListener1");
        SlowListener testListener2(*kpsrSubscriber2, "slowListener2");
        printInvocations("testListener2");

        std::thread concurrentDataUpdaterThread([&testPublisher]() {
            for (int i = 1; i <= publishIterations; i++) {
                TestEvent event(i, "hello");
                std::cout << "SimpleEventPublisher is about to receive TestEvent(" << event.id
                          << "," << event.message << ")\n\tat addr " << &event << "\n\ton thread "
                          << std::this_thread::get_id() << std::endl;
                testPublisher.run(event);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                event.id = -i;
                event.message = "bye";
            }
        });
        concurrentDataUpdaterThread.join();
        printInvocations("publishing");
    }
    printInvocations("program finished");

    return 0;
}
```

- Data multiplexer accepts multiple subscribers which run independently. This example shows 2 subscribers, each with only 1 listener (the same previous slow listener for simplicity).
- Data multiplexer uses an internal ring-buffer, more efficient than the smart pool.\
  The initializer and clone functions work the same.
- There are no start or stop methods.
- Data multiplexer keeps the buffer for, at least, one subscriber and its listener/s. (A wait loop for the listeners finishing is not required.)

### Sample output

```text
$ bin/kpsr_tutorial_chapter1_annexB_3

Main thread 140177296181120
Invocations counters updating after data multiplexer creation
        emptyConstructorInvocations 16
Invocations counters updating after getPublisher
        (none)
Invocations counters updating after the first getSubscriber
        (none)
Invocations counters updating after the second getSubscriber
        (none)
Invocations counters updating after testPublisher
        (none)
Invocations counters updating after testListener1
        (none)
Invocations counters updating after testListener2
        (none)
SimpleEventPublisher is about to receive TestEvent(1,hello)
        at addr 0x7f7d90f15d70
        on thread 140177279379200
SimpleEventPublisher publishes TestEvent(1,hello)
        at addr 0x7f7d90f15d70
        on thread 140177279379200
SlowListener (slowListener1) received TestEvent(1,hello)
        at addr 0x7f7d88000b70
        on thread 140177296164608
SlowListener (slowListener2) received TestEvent(1,hello)
        at addr 0x7f7d88000b70
        on thread 140177287771904
SimpleEventPublisher is about to receive TestEvent(2,hello)
        at addr 0x7f7d90f15d70
        on thread 140177279379200
SimpleEventPublisher publishes TestEvent(2,hello)
        at addr 0x7f7d90f15d70
        on thread 140177279379200
SimpleEventPublisher is about to receive TestEvent(3,hello)
        at addr 0x7f7d90f15d70
        on thread 140177279379200
SimpleEventPublisher publishes TestEvent(3,hello)
        at addr 0x7f7d90f15d70
        on thread 140177279379200
Invocations counters updating after publishing
        constructorInvocations      3
        copyInvocations             3
        destructorInvocations       6
SlowListener (slowListener2) finished, now TestEvent(1,hello)
SlowListener (slowListener1) finished, now TestEvent(1,hello)
SlowListener (slowListener1) received TestEvent(3,hello)
        at addr 0x55965f201f50
        on thread 140177296164608
SlowListener (slowListener1) finished, now TestEvent(3,hello)
Invocations counters updating after program finished
        destructorInvocations       22
```

**Threads.**

- One for the main program (140177296181120).
- One executed for publishing 3x messages in a row (140177279379200).
- One for every subscriber and its listener (140177296164608 and 140177287771904).

**TestEvent constructors.**

After the data multiplexer creation:

- 16x `emptyConstructorInvocations`, according to the buffer size.

After the events publishing:

- 3x `constructorInvocations`, one for every message and sharing the same address (0x7f7d90f15d70).
- 3x `copyInvocations`, when the data was passed _by value_ to the subscriber and shared by the listener (0x7f7d88000b70, etc.).
- 6x `destructorInvocations`, equals the last invocations.

When the program finished,

- 22x `destructorInvocations`, adding the 16 initial ring-buffered elements.

<p align="center">
emptyConstructorInvocations + constructorInvocations + copyInvocations = destructorInvocations
</p>

**Data update.**

Data multiplexer only provides the last data to the subscribers:

- `TestEvent(1,hello)` is received by both subscribers and its listeners.\
  Since the listeners are slow in processing:
- `TestEvent(2,hello)` is old data and so discarded.
- The last `TestEvent(3,hello)` is kept in memory until it is processed by _one_ of the subscribers and its listener.
