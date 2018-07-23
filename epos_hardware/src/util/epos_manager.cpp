#include "epos_hardware/epos_manager.h"
#include <boost/foreach.hpp>
#include <string>

namespace epos_hardware {

EposManager::EposManager()
{
}

void EposManager::construct_motors(ros::NodeHandle& nh, ros::NodeHandle& pnh,
  const std::vector<std::string>& motor_names,
  hardware_interface::ActuatorStateInterface& asi,
  hardware_interface::VelocityActuatorInterface& avi,
  hardware_interface::PositionActuatorInterface& api)
{
  hardware_interface::ActuatorStateInterface* asi_(&asi);
  hardware_interface::VelocityActuatorInterface* avi_(&avi);
  hardware_interface::PositionActuatorInterface* api_(&api);

  BOOST_FOREACH(const std::string& motor_name, motor_names)
  {
    ROS_INFO_STREAM("Loading EPOS: " << motor_name);
    ros::NodeHandle motor_config_nh(pnh, motor_name);
    boost::shared_ptr<Epos> motor(new Epos(motor_name, nh, motor_config_nh, &epos_factory, *asi_, *avi_, *api_));
    motors_.push_back(motor);
  }
}

bool EposManager::init()
{
  bool success = true;
  BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors_) {
    if(!motor->init()) {
      ROS_ERROR_STREAM("Could not configure motor: " << motor->name());
      success = false;
    }
  }
  return success;
}

void EposManager::update_diagnostics() {
  BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors_) {
    motor->update_diagnostics();
  }
}

void EposManager::read() {
  BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors_) {
    motor->read();
  }
}

void EposManager::write() {
  BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors_) {
    motor->write();
  }
}

bool EposManager::enable_motors(){
    bool success = true;
    BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors_) {
        if(!motor->enable_motors()){
          ROS_ERROR_STREAM("Could not enable motor: " << motor->name());
          success = false;
        }
    }
    return success;
}

bool EposManager::disable_motors(){
    bool success = true;
    BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors_) {
        if(!motor->disable_motors()){
          ROS_ERROR_STREAM("Could not disable motor: " << motor->name());
          success = false;
        }
    }
    return success;
}

bool EposManager::stop_homing(){
    bool success = true;
    BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors_) {
        if(!motor->stop_homing()){
          ROS_ERROR_STREAM("Could not stop homing motor: " << motor->name());
          success = false;
        }
    }
    return success;
}

bool EposManager::start_homing(){
    bool success = true;
    BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors_) {
        if(!motor->start_homing()){
          ROS_ERROR_STREAM("Could not start homing motor: " << motor->name());
          success = false;
        }
    }
    return success;
}

bool EposManager::clear_faults(){
    bool success = true;
    BOOST_FOREACH(const boost::shared_ptr<Epos>& motor, motors_) {
        if(!motor->clear_faults()){
          ROS_ERROR_STREAM("Could not clear faults: " << motor->name());
          success = false;
        }
    }
    return success;
}


}
