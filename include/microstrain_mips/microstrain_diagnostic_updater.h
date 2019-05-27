#include "ros/ros.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "microstrain_mips/status_msg.h"
#include "microstrain_3dm.h"

#include <string>


namespace microstrain_mips
{
  class RosDiagnosticUpdater : private diagnostic_updater::Updater
  {
  public:
    RosDiagnosticUpdater(Microstrain::Microstrain *device);

    void generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void packetDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void portDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void imuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void statusCallback(const microstrain_mips::status_msg::ConstPtr& status);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber status_sub_;

    microstrain_mips::status_msg last_status_;
  };
}
