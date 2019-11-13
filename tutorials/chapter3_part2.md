<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

[Previous Tutorial: Core functionality](./chapter3_part1.md)

# Chapter 3. Part II Event Loop

In this case we deal with a slightly complex case where several
threads can contain publishers whereas the subscribers are all
restricted to one thread (ensuring no thread deadlocks). We still
restrict ourselves to the case without middlewares.

In this case, the headers generated in example 1 for the message
objects are still valid and are reused.

## Klepsydra API

In order to ensure all the subscribers run in the same thread, we use
the `EventLoopMiddlewareProvider` from the `klepsydra high
performance` repository. This single provider uses templated functions
to provide access to any type of publisher/subscriber. In the previous
case, we had to define a separate provider for each
publisher/subscriber type. Thus, for example, the line below is
sufficient to get the Battery subscriber:

```
provider.getSubscriber<kpsr::sensors::BatteryState>(batteryTopicName)
```
while to get a Temperature subscriber, you only have to write:
```
provider.getSubscriber<kpsr::sensors::Temperature>(temperatureTopicName)
```
where the arguments to the getter are strings containing the topic name.

The EventLoopMiddlewareProvider will run all the subscribers
sequentially, ensuring that no thread conflicts arise.

## Final file

Overall, the code to write this example is quite similar to that of
the first example, except for the changes brought about by using
`EventLoopMiddlewareProvider`. The file `example2.cpp` shows what code
changes are necessary (compared to example1) to implement this. As can
be seen, the changes are quite minimal.

[Next Tutorial: Performance Monitoring](./chapter3_part3.md)

