<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# Chapter 3. Part I Core functionality

In this case we deal with the case involving no middlewares. We deal
with simple C++ objects being used to transmit and receive
messages.


## Generate headers for message objects

The kpsr-code-generation package, if installed correctly, has the
`kpsr_codegen` utility to generate header files based on a YAML file
describing the structure of the message. We use the BatteryState and
Temperature message used by ROS as a template to define the
message. (Note, ROS is not necessary. We are only using their message
types as a base to start off from.)

Corresponding YAML files are in the `kidl`
folder. `kpsr_codegen` application should take this folder as input
and generate header files that we can use.

The CMakeLists file in this repository has been configured to run the
following command:

```
kpsr_codegen -i kidl -o gen klepsydra/tutorial -r True -z True -d True
```

The header files generated should then be included where the messages
are processed. The CMakeLists has been configured to include the
header files in the folder `gen`.

Since we pass the `klepsydra/tutorial` as prefix to the codgen command
(via the CMakeLists file), the generated files are in
`gen/poco/include/klepsydra/tutorial` folder. The CMakeLists file has
been configured to include the correct necessary paths.

## Klepsydra Core API

The aim of this tutorial is to now use the generated messages and the
service classes with the klepsydra core (and mem_core) API to make a
working example where a low battery value causes the cubesat to stop.

For this, we look at the Middle Provider classes available to use. The
`mem_core` module provides a `BasicMiddlewareProvider` class which
holds a pair of publisher and subscribers, which are each attached to
a topic (given by its name during instantiation).

** Note: `BasicMiddlewareProvider` is used in the simplest cases where
we assume there is only one thread for the subscriber and no other
subscribers are used in the `ControlService` class. In this example, we
will only use the battery subscriber and thus we can safely assume all
operations are thread-safe. In case you use `BasicMiddlewareProvider`
with multiple subscribers, you MUST add a mutex to the
`ControlService` class to ensure operations are thread-safe.**

The templated class `BasicMiddlewareProvider` gives us access to each
respective sensor publisher/subscriber pair.

## Bringing everything together

We bring all these ideas together to write the first example using our
custom service classes and the API provided by klepsydra core. (The
final files for these tutorials are in the `examples/core/` folder.)

Using the middleware provider `BasicMiddlewareProvider` from the
`mem_core` module, we can get a subscriber/publisher pair for each
sensor and use these for declaring a local instance of the
`ControlService` and `ApplicationService` classes each. A separate
thread is declared to publish dummy battery values (normally, in real
production code, this thread is not necessary as real values are being
published).

We just need to ensure that the providers have started polling by
calling the start() methods before starting the publisher thread (so
that we don't miss any data).

The example1.cpp can be compiled using cmake and the binary is
available as `kpsr_tutorial_example_1`.

## Note on threading

This case describes a simple case where we don't have to worry about
multiple threads and its related problems. To actually use this in a
real case, we must make sure that there are sufficient locks to ensure
that two subscribers are not triggered at the same time. Since the
`_status` member of the `ControlService` class can be modified by any
of the subscriber, we must ensure that there is a mutex lock
protecting it, as we cannot guarantee that two subscribers will never
be triggered simultaneously.

[Next Tutorial: Event Loop](./chapter3_part2.md)

