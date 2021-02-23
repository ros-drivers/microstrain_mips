#include "ros/ros.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "mscl_msgs/Status.h"
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
    void statusCallback(const mscl_msgs::Status::ConstPtr& status);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber status_sub_;

    mscl_msgs::Status last_status_;
  };
}
