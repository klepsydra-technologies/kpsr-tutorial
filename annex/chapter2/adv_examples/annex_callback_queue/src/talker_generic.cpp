#include "ros/ros.h"
#include "annex_tutorials_ros/Num.h"
#include "annex_tutorials_ros/temperature.h"
#include <thread>
#include <sstream>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "talker_generic");

   ros::NodeHandle n;

   ros::Publisher chatter_pub_Num = n.advertise<annex_tutorials_ros::Num>("chatterNum", 1000);
   ros::Publisher chatter_pub_Temp = n.advertise<annex_tutorials_ros::temperature>("chatterTem", 1000);

   ros::Rate loop_rate(10);

   int count = 0;
   while (ros::ok())
   {
     annex_tutorials_ros::Num tmsg;
     annex_tutorials_ros::temperature tem;

     tmsg.x = 1;
     tmsg.y = 2;
     tem.t = 25;
     std::thread::id this_id = std::this_thread::get_id();

     std::stringstream ss;
     ss << "Position (x : " << tmsg.x << ", y: "<< tmsg.y << ") - Temperature t :" << tem.t << " - Thread id : " << this_id << count;

     ROS_INFO("%s", ss.str().c_str());

     chatter_pub_Num.publish(tmsg);
     chatter_pub_Temp.publish(tem);


     ros::spinOnce();

     loop_rate.sleep();
     ++count;
  }

   return 0;
 }
