/*

Copyright (c) 2017, Brian Bingham
Copyright (c)  2020, Parker Hannifin Corp
This code is licensed under MIT license (see LICENSE file for details)

*/

#include "microstrain_3dm.h"
#include "microstrain_diagnostic_updater.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto us = std::make_shared<Microstrain::Microstrain>();

  us->setup();

  //Diagnostics updater for status
  us->declare_parameter("diagnostics", true);

  if( us->get_parameter("diagnostics").as_bool())
  {
      ros_mscl::RosDiagnosticUpdater ros_diagnostic_updater(us);
  }

  rclcpp::Rate r(us->m_spin_rate);

  while(rclcpp::ok())
  {
      us->process();
      rclcpp::spin_some(us);

      r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
