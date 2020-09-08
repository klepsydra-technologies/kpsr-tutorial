#include "ros/ros.h"
#include "annex_tutorials_ros/Num.h"
#include "annex_tutorials_ros/temperature.h"
#include <thread>
#include <sstream>
#include "callback_queue.h"

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
  ros::init(argc, argv, "listener_async_callback");

  // Creating a callback queue
  ros::CallbackQueue my_callback_queue;

  ros::NodeHandle n;

  n.setCallbackQueue(&my_callback_queue);

  my_callback_queue.callAvailable(ros::WallDuration());

  ros::Subscriber subNum = n.subscribe("chatterNum", 1000, chatterNumCallback);

  ros::Subscriber subTem = n.subscribe("chatterTem", 1000, chatterTemCallback);

  ros::AsyncSpinner spinner(3, &my_callback_queue);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
