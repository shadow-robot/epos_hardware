#ifndef EPOS_HARDWARE_EPOS_HARDWARE_H_
#define EPOS_HARDWARE_EPOS_HARDWARE_H_

#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <boost/scoped_array.hpp>
#include <boost/scoped_ptr.hpp>
#include "epos_hardware/utils.h"
#include "epos_hardware/epos.h"
#include "epos_hardware/epos_manager.h"
#include "epos_hardware/EnableMotors.h"
#include "epos_hardware/DisableMotors.h"
#include "epos_hardware/StopHoming.h"
#include "epos_hardware/StartHoming.h"
#include "epos_hardware/ClearFaults.h"


namespace epos_hardware {

class EposHardware : public hardware_interface::RobotHW {
public:
  EposHardware(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::vector<std::string>& motor_names);
  bool init();
  void read();
  void write();
  void update_diagnostics();
private:
  hardware_interface::ActuatorStateInterface asi;
  hardware_interface::VelocityActuatorInterface avi;
  hardware_interface::PositionActuatorInterface api;

  EposManager epos_manager_;

  transmission_interface::RobotTransmissions robot_transmissions;
  boost::scoped_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader;

  bool enableMotorsSrv(epos_hardware::EnableMotors::Request &req,
                       epos_hardware::EnableMotors::Response &res);

  bool disableMotorsSrv(epos_hardware::DisableMotors::Request &req,
                       epos_hardware::DisableMotors::Response &res);

  bool stopHomingSrv(epos_hardware::StopHoming::Request &req,
                     epos_hardware::StopHoming::Response &res);

  bool startHomingSrv(epos_hardware::StartHoming::Request &req,
                      epos_hardware::StartHoming::Response &res);

  bool clearFaultsSrv(epos_hardware::ClearFaults::Request &req,
                      epos_hardware::ClearFaults::Response &res);

  ros::ServiceServer enable_motors, disable_motors, stop_motor_homing, start_motor_homing, clear_faults;
};

}


#endif
