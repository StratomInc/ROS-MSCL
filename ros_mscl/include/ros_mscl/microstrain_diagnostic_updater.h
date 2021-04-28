#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include <mscl_msgs/msg/status.hpp>
#include "microstrain_3dm.h"

#include <string>


namespace ros_mscl
{
  class RosDiagnosticUpdater : private diagnostic_updater::Updater
  {
  public:
    RosDiagnosticUpdater(rclcpp::Node::SharedPtr n);

    void generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void packetDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void portDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void imuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void statusCallback(const mscl_msgs::msg::Status::SharedPtr status);

  private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<mscl_msgs::msg::Status>::SharedPtr status_sub_;

    mscl_msgs::msg::Status last_status_;
  };
}
