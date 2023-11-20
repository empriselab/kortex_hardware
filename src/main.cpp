#include <iostream>
#include <sstream>

#include <Gen3Robot.h>
#include <ros/rate.h>

#include "std_msgs/String.h"

int main(int argc, char* argv[])
{
  ROS_INFO_STREAM("Gen3 HARDWARE starting");
  ros::init(argc, argv, "kortex_hardware");
  ros::NodeHandle nh;

  Gen3Robot robot(nh);
  controller_manager::ControllerManager cm(&robot);
  bool use_admittance;
  ros::param::get("~use_admittance", use_admittance);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Ros control rate of 1100Hz
  ros::Rate controlRate(1100);
  while (ros::ok())
  {
    robot.read();
    cm.update(robot.get_time(), robot.get_period());

    if (!use_admittance)
      robot.write();
    controlRate.sleep();
  }

  return 0;
}
