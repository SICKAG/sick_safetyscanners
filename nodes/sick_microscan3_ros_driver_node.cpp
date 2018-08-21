#include <ros/ros.h>
#include "sick_microscan3_ros_driver/Microscan3Ros.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "sick_microscan3_ros_driver");

   sick::Microscan3Ros microscan3_ros;

   ros::spin();
   return 0;
}
