# ROS

This section is dedicated to ROS only processes.

## annex_tutorials_ros

In this folder, the talker publishes 2 different types of data: a position (float x, float y) and a temperature (int t). The thread id of the talker is also printed out. 

On the listener side, the data received are printed out. The thread id of the listener is also printed out. 

http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning

According to the different scenarios described in the link here-abore (section 1 and section 2 only), ```spin()``` and ```spinOnce()``` are really meant for single-threaded applications, and are not optimized for being called from multiple threads at once. 

For spinning from multiple threads, here-after is an example related to AsyncSpinner API :

```cpp
ros::AsyncSpinner spinner(4); // Use 4 threads
spinner.start();
ros::waitForShutdown();
```

Please note that the ros::waitForShutdown() function does not spin on its own, so the example above will spin with 4 threads in total. 
