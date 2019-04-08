#include "ros/ros.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "microstrain_3dm/status_msg.h"
#include "microstrain_3dm.h"

#include <string>


namespace microstrain_3dm
{
  class RosDiagnosticUpdater : private diagnostic_updater::Updater
  {
  public:
    RosDiagnosticUpdater(Microstrain::Microstrain *device);

    void generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void packetDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void portDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void imuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void statusCallback(const microstrain_3dm::status_msg status);
    //void gpsDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber status_sub_;

    microstrain_3dm::status_msg* last_status_;
  };
}
