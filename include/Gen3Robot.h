#ifndef GEN3_ROBOT_H
#define GEN3_ROBOT_H

// pinocchio
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/sample-models.hpp>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_mode_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// ros
#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "kortex_hardware/ModeService.h"

// kinova api
#include <client/RouterClient.h>
#include <client/SessionManager.h>
#include <client/TransportClientTcp.h>
#include <client/TransportClientUdp.h>
#include <client_stubs/ActuatorConfigClientRpc.h>
#include <client_stubs/BaseClientRpc.h>
#include <client_stubs/BaseCyclicClientRpc.h>

// c++
#include <iostream>
#include <limits>
#include <stdexcept>

#include <LowPassFilter.h>
#include <time.h>
#include <unistd.h>

#define TCP_PORT 10000
#define UDP_PORT 10001

using namespace std;
namespace k_api = Kinova::Api;

class Gen3Robot : public hardware_interface::RobotHW
{
public:
  Gen3Robot(ros::NodeHandle nh);

  virtual ~Gen3Robot();

  void initializeSoftLimits();

  ros::Time get_time(void);

  ros::Duration get_period(void);

  inline double degreesToRadians(double degrees);
  inline double radiansToDegrees(double radians);
  inline double radiansToFingerTicks(double radians);
  inline double fingerTicksToRadians(double ticks);

  void sendPositionCommand(const std::vector<double>& command);
  void sendVelocityCommand(const std::vector<double>& command);
  void sendTorqueCommand(std::vector<double>& command);
  void sendCurrentCommand(std::vector<double>& command);
  void sendGripperPositionCommand(const float& command);
  void sendGripperVelocityCommand(const float& command);
  void sendGripperLowLevelCommand(const float& command);
  void setBaseCommand();
  void switchToEffortMode();
  bool setControlMode(
      kortex_hardware::ModeService::Request& req,
      kortex_hardware::ModeService::Response& resp);

  void write(void);
  void read(void);

private:
  std::string m_username;
  std::string m_password;
  int m_api_session_inactivity_timeout_ms;
  int m_api_connection_inactivity_timeout_ms;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::JointModeInterface jm_interface;

  int num_full_dof;
  int num_arm_dof;
  int num_finger_dof = 1;

  // input command vars
  vector<double> cmd_pos;
  vector<double> cmd_vel;
  vector<double> cmd_eff;
  double gripper_cmd_pos;
  double gripper_cmd_vel;
  double gripper_cmd_eff;
  // current control vars
  bool current_control = false;
  std::vector<float> input_current_limit;
  std::vector<float> gear_ratio;

  vector<double> pos; // contains full dof
  vector<double> vel;
  vector<double> eff;
  double gripper_pos;
  double gripper_vel;
  double gripper_eff;

  vector<double> pos_offsets;
  vector<double> soft_limits;
  vector<double> zero_velocity_command;
  vector<double> prev_cmd_pos;
  vector<double> prev_cmd_eff;

  LowPassFilter* in_lpf;
  LowPassFilter* out_lpf;
  bool is_in_lpf_initialized = false;
  bool is_out_lpf_initialized = false;

  hardware_interface::JointCommandModes arm_mode;
  hardware_interface::JointCommandModes gripper_mode;
  hardware_interface::JointCommandModes last_arm_mode;
  ros::ServiceServer mode_service;

  // Kortex Api objects
  std::string m_ip_address;
  k_api::TransportClientTcp* m_tcp_transport;
  k_api::TransportClientUdp* m_udp_transport;
  k_api::RouterClient* m_tcp_router;
  k_api::RouterClient* m_udp_router;
  k_api::SessionManager* m_tcp_session_manager;
  k_api::SessionManager* m_udp_session_manager;

  k_api::Base::BaseClient* mBase;
  k_api::BaseCyclic::BaseCyclicClient* mBaseCyclic;
  k_api::BaseCyclic::Command mBaseCommand;
  k_api::BaseCyclic::Feedback mLastFeedback;
  k_api::Base::Gripper gripper_feedback;
  k_api::Base::GripperRequest gripper_request;
  k_api::Base::GripperCommand gripper_command;
  k_api::GripperCyclic::MotorCommand* gripper_low_level_cmd;
  k_api::Base::Finger* finger;
  bool mFirstFeedbackReceived = false;
  bool mLowLevelServoing = false;
  bool mUseGripper = false;
  bool mUseAdmittance = false;
  // Used for low level gripper control
  double gripper_position_error;
  double gripper_velocity;
  double gripper_proportional_gain = 1.2;

  k_api::ActuatorConfig::ActuatorConfigClient* mActuatorConfig;
  k_api::Base::ServoingModeInformation mServoingMode;
  k_api::ActuatorConfig::ControlModeInformation mControlModeMessage;
  int first_actuator_device_id = 1;
  unsigned int mActuatorCount;

  // For frequency monitoring
  int64_t now = 0;
  int64_t last = 0;

  // Gravity compensation related variables
  pinocchio::Model model;
  pinocchio::Data data;
  std::string mURDFFile;
};

#endif
