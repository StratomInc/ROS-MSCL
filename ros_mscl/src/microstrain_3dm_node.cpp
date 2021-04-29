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

  std::shared_ptr<Microstrain::Microstrain> microstrain = std::make_shared<Microstrain::Microstrain>();

  microstrain->setup();

  //Diagnostics updater for status
  microstrain->declare_parameter("diagnostics", true);

  if( microstrain->get_parameter("diagnostics").as_bool())
  {
      ros_mscl::RosDiagnosticUpdater ros_diagnostic_updater(microstrain);
  }

  rclcpp::Rate r(microstrain->m_spin_rate);

  while(rclcpp::ok())
  {
      microstrain->process();
      rclcpp::spin_some(microstrain);

      r.sleep();
  }

  microstrain->cleanup();

  rclcpp::shutdown();
  return 0;
}
