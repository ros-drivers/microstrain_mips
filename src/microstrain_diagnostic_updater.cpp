#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "microstrain_diagnostic_updater.h"
#include "microstrain_3dm.h"
#include <string>



namespace microstrain_3dm
{

RosDiagnosticUpdater::RosDiagnosticUpdater(Microstrain::Microstrain *device)
{
  setHardwareID("unknown");
  add("general", this, &RosDiagnosticUpdater::generalDiagnostics);
  add("packet", this, &RosDiagnosticUpdater::packetDiagnostics);
  add("port", this, &RosDiagnosticUpdater::portDiagnostics);
  add("imu", this, &RosDiagnosticUpdater::imuDiagnostics);

  /*if(device->get_model_gps()){
    add("gps", this, &RosDiagnosticUpdater::gpsDiagnostics);
    GPS = true;
  }*/


  status_sub_ = nh_.subscribe("device/status", 5, &RosDiagnosticUpdater::statusCallback, this);
}


void RosDiagnosticUpdater::generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Device Model", last_status_->device_model);
  stat.add("Status Selector", last_status_->status_selector);
  stat.add("Status Flags", last_status_->status_flags);
  stat.add("System State", last_status_->system_state);
  stat.add("System Timer ms", last_status_->system_timer_ms);
  stat.add("IMU Stream Enabled", last_status_->imu_stream_enabled);
  stat.add("Filter Stream Enabled", last_status_->filter_stream_enabled);

  /*if(GPS){
    stat.add("GPS Stream Enabled", last_status_->gps_stream_enabled);
  }*/

  if (last_status_->status_flags > 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Status flags raised");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Status ok");
  }

}

void RosDiagnosticUpdater::packetDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("IMU Dropped Packets", last_status_->imu_dropped_packets);
  stat.add("Filter Dropped Packets", last_status_->filter_dropped_packets);

  /*if(GPS){
    stat.add("GPS Dropped Packets", last_status_->gps_dropped_packets);

    if(last_status_->gps_dropped_packets > 0){
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Packets dropped");
    }
  }*/

  if (last_status_->imu_dropped_packets > 0 || last_status_->filter_dropped_packets > 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Packets dropped");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No dropped packets");
  }
}

void RosDiagnosticUpdater::portDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("COM1 Port Bytes Written", last_status_->com1_port_bytes_written);
  stat.add("COM1 Port Bytes Read", last_status_->com1_port_bytes_read);
  stat.add("COM1 Port Write Overruns", last_status_->com1_port_write_overruns);
  stat.add("COM1 Port Read Overruns", last_status_->com1_port_read_overruns);

  if (last_status_->com1_port_write_overruns > 0 || last_status_->com1_port_read_overruns > 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Port overruns");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No port overruns");
  }
}

void RosDiagnosticUpdater::imuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("IMU Parser Errors", last_status_->imu_parser_errors);
  stat.add("IMU Message Count", last_status_->imu_message_count);
  stat.add("IMU Last Message ms", last_status_->imu_last_message_ms);

  if (last_status_->imu_parser_errors > 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "IMU Parser Errors");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No IMU parser errors");
  }
}

/*void RosDiagnosticUpdater::gpsDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("GPS Power On", last_status_->gps_power_on);
  stat.add("GPS PPS Triggers", last_status_->num_gps_pps_triggers);
  stat.add("Last GPS PPS Trigger ms", last_status_->last_gps_pps_trigger_ms);
  stat.add("GPS Parser Errors", last_status_->gps_parser_errors);
  stat.add("GPS Message Count", last_status_->gps_message_count);
  stat.add("GPS Last Message ms", last_status_->gps_last_message_ms);

  if (last_status_->gps_parser_errors > 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "GPS Parser Errors");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No GPS parser errors");
  }
}*/

void RosDiagnosticUpdater::statusCallback(const microstrain_3dm::status_msg status)
{
  *last_status_ = status;
  update();
}





}
