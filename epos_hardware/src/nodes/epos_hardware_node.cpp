#include <ros/ros.h>
#include <ros/spinner.h>
#include "epos_hardware/epos_hardware.h"
#include <controller_manager/controller_manager.h>
#include <vector>
#include "epos_hardware/EnableMotors.h"
#include "epos_hardware/DisableMotors.h"
#include "epos_hardware/StopHoming.h"
#include "epos_hardware/StartHoming.h"
#include "epos_hardware/ClearFaults.h"

bool stopHoming(epos_hardware::StopHoming::Request  &req,
    epos_hardware::StopHoming::Response &res, epos_hardware::EposHardware* robot)
{
    res.stopped = robot->stop_homing();
    if(res.stopped == true)
        return true;
}

bool startHoming(epos_hardware::StartHoming::Request &req,
    epos_hardware::StartHoming::Response &res, epos_hardware::EposHardware* robot)
{
    res.started = robot->start_homing();
    if(res.started == true)
        return true;

}

bool clearFaults(epos_hardware::ClearFaults::Request &req,
    epos_hardware::ClearFaults::Response &res, epos_hardware::EposHardware* robot)
{
    res.clear_faults = robot->clear_faults();
    if(res.clear_faults == true)
        return true;

}

bool enableMotors(epos_hardware::EnableMotors::Request &req,
    epos_hardware::EnableMotors::Response &res, epos_hardware::EposHardware* robot)
{
    res.enabled = robot->enable_motors();
    if(res.enabled == true)
        return true;

}

bool disableMotors(epos_hardware::DisableMotors::Request &req,
    epos_hardware::DisableMotors::Response &res, epos_hardware::EposHardware* robot)
{
    res.disabled = robot->disable_motors();
    if(res.disabled == true)
        return true;

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "epos_velocity_hardware");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::vector<std::string> motor_names;
  for(int i = 0; i < argc-1; ++i) {
    motor_names.push_back(argv[i+1]);
  }
  epos_hardware::EposHardware robot(nh, pnh, motor_names);
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(3);
  spinner.start();

  boost::function<bool(epos_hardware::StopHoming::Request & req, epos_hardware::StopHoming::Response & res)>
  stop_homing_cb = boost::bind(&stopHoming, _1, _2, &robot);
  boost::function<bool(epos_hardware::StartHoming::Request & req, epos_hardware::StartHoming::Response & res)>
  start_homing_cb = boost::bind(&startHoming, _1, _2, &robot);
  boost::function<bool(epos_hardware::ClearFaults::Request & req, epos_hardware::ClearFaults::Response & res)>
  clear_faults_cb = boost::bind(&clearFaults, _1, _2, &robot);
  boost::function<bool(epos_hardware::EnableMotors::Request & req, epos_hardware::EnableMotors::Response & res)>
  enable_motors_cb = boost::bind(&enableMotors, _1, _2, &robot);
  boost::function<bool(epos_hardware::DisableMotors::Request & req, epos_hardware::DisableMotors::Response & res)>
  disable_motors_cb = boost::bind(&disableMotors, _1, _2, &robot);

  ros::ServiceServer stop_motor_homing = nh.advertiseService("stop_motor_homing", stop_homing_cb);
  ros::ServiceServer start_motor_homing = nh.advertiseService("start_motor_homing", start_homing_cb);
  ros::ServiceServer clear_faults = nh.advertiseService("clear_faults", clear_faults_cb);
  ros::ServiceServer enable_motors = nh.advertiseService("enable_motors", enable_motors_cb);
  ros::ServiceServer disable_motors = nh.advertiseService("disable_motors", disable_motors_cb);

  ROS_INFO("Initializing Motors");
  if(!robot.init()) {
    ROS_FATAL("Failed to initialize motors");
    return 1;
  }
  ROS_INFO("Motors Initialized");

  ros::Rate controller_rate(50);
  ros::Time last = ros::Time::now();
  while (ros::ok()) {
    robot.read();
    ros::Time now = ros::Time::now();
    cm.update(now, now-last);
    robot.write();
    last = now;
    robot.update_diagnostics();
    controller_rate.sleep();
  }

}
