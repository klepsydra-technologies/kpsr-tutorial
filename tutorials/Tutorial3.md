[![klepsydra logo](../images/klepsydra_logo.jpg)](http://www.klepsydra.org)

[Previous Tutorial: Event Loop](./Tutorial2.md)

# Performance Monitoring

Next, we consider the case where we want to be able to monitor the
performance of the services. The logging facility can be obtained by
using the `kpsr-admin` library while the results can be viewed using
the `kpsr-openmct` library (which provides resources for integration
of Klepsydra with NASA's OpenMCT).

## Environment

We can also additionally pass a non-default environment (to the
 service classes). The idea behind passing an
environment is to be able to externally define the environment
properties. For example, in this case we could define different
allowable temperature limits than the ones defined in the problem
statement. This makes the code more flexible. Of course, the control
service class until now has been used with limits defined by static
constants and needs to be updated to consider the case where the
environment provides these values.

[Next Tutorial: ROS connector](./TutorialROS.md)

