#include "ros/ros.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "ros_mscl/status_msg.h"
#include "microstrain_3dm.h"

#include <string>


namespace ros_mscl
{
  class RosDiagnosticUpdater : private diagnostic_updater::Updater
  {
  public:
    RosDiagnosticUpdater();

    void generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void packetDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void portDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void imuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void statusCallback(const ros_mscl::status_msg::ConstPtr& status);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber status_sub_;

    ros_mscl::status_msg last_status_;
  };
}
