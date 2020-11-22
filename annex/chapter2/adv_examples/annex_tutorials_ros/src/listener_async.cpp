#include "ros/ros.h"
#include "annex_tutorials_ros/Num.h"
#include "annex_tutorials_ros/temperature.h"
#include <thread>
#include <sstream>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
*/
void chatterNumCallback(const annex_tutorials_ros::Num::ConstPtr& tmsg)
{
    std::thread::id this_id = std::this_thread::get_id();

    std::stringstream ll;
    ll << "Position (x : " << tmsg->x << ", y: " << tmsg->y << ") - Thread id : " << this_id;

    ROS_INFO("I heard: [%s]", ll.str().c_str());
}

void chatterTemCallback(const annex_tutorials_ros::temperature::ConstPtr& tem)
{
    std::thread::id this_id = std::this_thread::get_id();

    std::stringstream kk;
    kk << "Temperature t : " << tem->t << " - Thread id : " << this_id;

    ROS_INFO("I heard: [%s]", kk.str().c_str());
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener_async");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber subNum = n.subscribe("chatterNum", 1000, chatterNumCallback);

  ros::Subscriber subTem = n.subscribe("chatterTem", 1000, chatterTemCallback);

  /**
   * A more useful threaded spinner is the AsyncSpinner. Instead of a blocking spin() call,
   * it has start() and stop() calls, and will automatically stop when it is destroyed.
   * An equivalent use of AsyncSpinner to the MultiThreadedSpinnere, is:
   */
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
