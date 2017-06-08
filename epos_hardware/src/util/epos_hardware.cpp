#include "epos_hardware/epos_hardware.h"
#include <boost/foreach.hpp>

namespace epos_hardware {

EposHardware::EposHardware(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::vector<std::string>& motor_names)
  : epos_manager_(asi, avi, api, nh, pnh, motor_names) {

  // TODO throw exception or something
  try {
    transmission_loader.reset(new transmission_interface::TransmissionInterfaceLoader(this, &robot_transmissions));
  }
  catch(const std::invalid_argument& ex){
    ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    return;
  }
  catch(const pluginlib::LibraryLoadException& ex){
    ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    return;
  }
  catch(...){
    ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
    return;
  }

  registerInterface(&asi);
  registerInterface(&avi);
  registerInterface(&api);

  std::string urdf_string;
  nh.getParam("robot_description", urdf_string);
  while (urdf_string.empty() && ros::ok()){
    ROS_INFO_STREAM_ONCE("Waiting for robot_description");
    nh.getParam("robot_description", urdf_string);
    ros::Duration(0.1).sleep();
  }

  transmission_interface::TransmissionParser parser;
  std::vector<transmission_interface::TransmissionInfo> infos;
  // TODO: throw exception
  if (!parser.parse(urdf_string, infos)) {
    ROS_ERROR("Error parsing URDF");
    return;
  }

  // build a list of all loaded actuator names
  std::vector<std::string> actuator_names;
  std::vector<boost::shared_ptr<Epos> > motors = epos_manager_.motors();
  BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors) {
    actuator_names.push_back(motor->actuator_name());
  }

  // Load all transmissions that are for the loaded motors
  BOOST_FOREACH(const transmission_interface::TransmissionInfo& info, infos) {
    bool found_some = false;
    bool found_all = true;
    BOOST_FOREACH(const transmission_interface::ActuatorInfo& actuator, info.actuators_) {
      if(std::find(actuator_names.begin(), actuator_names.end(), actuator.name_) != actuator_names.end())
	found_some = true;
      else
	found_all = false;
    }
    if(found_all) {
      if (!transmission_loader->load(info)) {
	ROS_ERROR_STREAM("Error loading transmission: " << info.name_);
	return;
      }
      else
	ROS_INFO_STREAM("Loaded transmission: " << info.name_);
    }
    else if(found_some){
      ROS_ERROR_STREAM("Do not support transmissions that contain only some EPOS actuators: " << info.name_);
    }
  }

  // Advertise services
  enable_motors = nh.advertiseService("enable_motors", &EposHardware::enableMotorsSrv, this);
  disable_motors = nh.advertiseService("disable_motors", &EposHardware::disableMotorsSrv, this);
  stop_motor_homing = nh.advertiseService("stop_motor_homing", &EposHardware::stopHomingSrv, this);
  start_motor_homing = nh.advertiseService("start_motor_homing", &EposHardware::startHomingSrv, this);
  clear_faults = nh.advertiseService("clear_faults", &EposHardware::clearFaultsSrv, this);

}

bool EposHardware::init() {
  return epos_manager_.init();
}

void EposHardware::update_diagnostics() {
  epos_manager_.update_diagnostics();
}

void EposHardware::read() {
  epos_manager_.read();
  if(robot_transmissions.get<transmission_interface::ActuatorToJointStateInterface>())
    robot_transmissions.get<transmission_interface::ActuatorToJointStateInterface>()->propagate();
}

void EposHardware::write() {
  if(robot_transmissions.get<transmission_interface::JointToActuatorVelocityInterface>())
    robot_transmissions.get<transmission_interface::JointToActuatorVelocityInterface>()->propagate();
  if(robot_transmissions.get<transmission_interface::JointToActuatorPositionInterface>())
    robot_transmissions.get<transmission_interface::JointToActuatorPositionInterface>()->propagate();
  epos_manager_.write();
}

bool EposHardware::enableMotorsSrv(epos_hardware::EnableMotors::Request &req,
                                   epos_hardware::EnableMotors::Response &res)
{
    res.enabled = epos_manager_.enable_motors();
    if(res.enabled == true)
        return true;

}

bool EposHardware::disableMotorsSrv(epos_hardware::DisableMotors::Request &req,
                                    epos_hardware::DisableMotors::Response &res)
{
    res.disabled = epos_manager_.disable_motors();
    if(res.disabled == true)
        return true;
}

bool EposHardware::stopHomingSrv(epos_hardware::StopHoming::Request  &req,
                                 epos_hardware::StopHoming::Response &res)
{
    res.stopped = epos_manager_.stop_homing();
    if(res.stopped == true)
        return true;
}

bool EposHardware::startHomingSrv(epos_hardware::StartHoming::Request &req,
                                  epos_hardware::StartHoming::Response &res)
{
    res.started = epos_manager_.start_homing();
    if(res.started == true)
        return true;
}

bool EposHardware::clearFaultsSrv(epos_hardware::ClearFaults::Request &req,
                                  epos_hardware::ClearFaults::Response &res)
{
    res.clear_faults = epos_manager_.clear_faults();
    if(res.clear_faults == true)
        return true;

}

}
