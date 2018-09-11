#include <ros/ros.h>
#include "sick_microscan3_ros_driver/Microscan3Ros.h"

/**
 * @brief main The Main node to start the Ros Driver, this method is executed via launch file
 * @param argc number of arguments given
 * @param argv arguments
 * @return is succesful, will always return 0
 */
int main(int argc, char** argv)
{
   ros::init(argc, argv, "sick_microscan3_ros_driver");

   sick::Microscan3Ros microscan3_ros;

   ros::spin();
   return 0;
}
