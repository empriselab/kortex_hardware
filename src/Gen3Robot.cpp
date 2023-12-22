#include "Gen3Robot.h"

#include <cmath> // std::abs
#include <typeinfo>

#include <unistd.h>

#include "kortex_hardware/ModeService.h"
#include "pinocchio/parsers/urdf.hpp"

using namespace std;

int64_t GetTickUs()
{
#if defined(_MSC_VER)
  LARGE_INTEGER start, frequency;
  LARGE_INTEGER start, frequency;

  QueryPerformanceFrequency(&frequency);
  QueryPerformanceCounter(&start);

  return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
  struct timespec start;
  clock_gettime(CLOCK_MONOTONIC, &start);

  return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

void addGravityCompensation(
    pinocchio::Model& model,
    pinocchio::Data& data,
    std::vector<double>& config,
    std::vector<double>& command)
{
  Eigen::VectorXd q = pinocchio::neutral(model);
  // convert ROS joint config to pinocchio config
  for (int i = 0; i < model.nv; i++)
  {
    int jidx = model.getJointId(model.names[i + 1]);
    int qidx = model.idx_qs[jidx];
    // nqs[i] is 2 for continuous joints in pinocchio
    if (model.nqs[jidx] == 2)
    {
      q[qidx] = std::cos(config[i]);
      q[qidx + 1] = std::sin(config[i]);
    }
    else
    {
      q[qidx] = config[i];
    }
  }

  Eigen::VectorXd gravity
      = pinocchio::computeGeneralizedGravity(model, data, q);
  // add gravity compensation torque to base command
  for (int i = 0; i < model.nv; i++)
  {
    command[i] = command[i] + gravity[i];
  }
}

Gen3Robot::Gen3Robot(ros::NodeHandle nh)
{
  ROS_INFO("Retreiving ROS parameters");
  ros::param::get("~username", m_username);
  ros::param::get("~password", m_password);
  ros::param::get("~ip_address", m_ip_address);
  ros::param::get(
      "~api_session_inactivity_timeout_ms",
      m_api_session_inactivity_timeout_ms);
  ros::param::get(
      "~api_connection_inactivity_timeout_ms",
      m_api_connection_inactivity_timeout_ms);
  ros::param::get("~dof", num_arm_dof);
  ros::param::get("~current_control", current_control);
  ros::param::get("~use_gripper", mUseGripper);
  ros::param::get("~use_admittance", mUseAdmittance);
  ros::param::get("~urdf_file", mURDFFile);

  ROS_INFO("Starting to initialize kortex_hardware");
  num_full_dof = num_arm_dof + num_finger_dof;
  // From kortex_driver/src/non-generated/kortex_arm_driver.cpp
  m_tcp_transport = new k_api::TransportClientTcp();
  m_udp_transport = new k_api::TransportClientUdp();
  m_tcp_transport->connect(m_ip_address, TCP_PORT);
  m_udp_transport->connect(m_ip_address, UDP_PORT);
  m_tcp_router
      = new k_api::RouterClient(m_tcp_transport, [](k_api::KError err) {
          ROS_ERROR(
              "Kortex API error was encountered with the TCP router: %s",
              err.toString().c_str());
        });
  m_udp_router
      = new k_api::RouterClient(m_udp_transport, [](k_api::KError err) {
          ROS_ERROR(
              "Kortex API error was encountered with the UDP router: %s",
              err.toString().c_str());
        });
  m_tcp_session_manager = new k_api::SessionManager(m_tcp_router);
  m_udp_session_manager = new k_api::SessionManager(m_udp_router);
  mBase = new k_api::Base::BaseClient(m_tcp_router);
  mBaseCyclic = new k_api::BaseCyclic::BaseCyclicClient(m_udp_router);

  mActuatorConfig
      = new k_api::ActuatorConfig::ActuatorConfigClient(m_tcp_router);
  mServoingMode = k_api::Base::ServoingModeInformation();
  mControlModeMessage = k_api::ActuatorConfig::ControlModeInformation();

  int i;
  cmd_pos.resize(num_full_dof);
  cmd_vel.resize(num_full_dof);
  cmd_eff.resize(num_full_dof);
  zero_velocity_command.resize(num_full_dof, 0.0);
  pos.resize(num_full_dof);
  vel.resize(num_full_dof);
  eff.resize(num_full_dof);
  pos_offsets.resize(num_full_dof);
  soft_limits.resize(num_full_dof);
  // cmd_cart_vel.resize(6); // SE(3)

  for (std::size_t i = 0; i < pos_offsets.size(); ++i)
    pos_offsets[i] = 0.0;

  for (std::size_t i = 0; i < cmd_vel.size(); ++i)
    cmd_vel[i] = 0.0;

  // connect and register the joint state interface.
  // this gives joint states (pos, vel, eff) back as an output.
  for (std::size_t i = 0; i < num_arm_dof; ++i)
  {
    std::string jnt_name = "joint_" + std::to_string(i + 1);

    // connect and register the joint state interface.
    // this gives joint states (pos, vel, eff) back as an output.
    hardware_interface::JointStateHandle state_handle(
        jnt_name, &pos[i], &vel[i], &eff[i]);
    jnt_state_interface.registerHandle(state_handle);

    // connect and register the joint position interface
    // this takes joint velocities in as a command.
    hardware_interface::JointHandle vel_handle(
        jnt_state_interface.getHandle(jnt_name), &cmd_vel[i]);
    jnt_vel_interface.registerHandle(vel_handle);

    // connect and register the joint position interface
    // this takes joint positions in as a command.
    hardware_interface::JointHandle pos_handle(
        jnt_state_interface.getHandle(jnt_name), &cmd_pos[i]);
    jnt_pos_interface.registerHandle(pos_handle);

    // connect and register the joint position interface
    // this takes joint effort in as a command.
    hardware_interface::JointHandle eff_handle(
        jnt_state_interface.getHandle(jnt_name), &cmd_eff[i]);
    jnt_eff_interface.registerHandle(eff_handle);
  }
  mode_service = nh.advertiseService(
      "set_control_mode", &Gen3Robot::setControlMode, this);

  // connect and register the joint state interface for gripper
  hardware_interface::JointStateHandle grp_state_handle(
      "finger_joint",
      &pos[num_full_dof - 1],
      &vel[num_full_dof - 1],
      &eff[num_full_dof - 1]);
  jnt_state_interface.registerHandle(grp_state_handle);

  hardware_interface::JointHandle grp_vel_handle(
      jnt_state_interface.getHandle("finger_joint"),
      &cmd_vel[num_full_dof - 1]);
  jnt_vel_interface.registerHandle(grp_vel_handle);

  hardware_interface::JointHandle grp_pos_handle(
      jnt_state_interface.getHandle("finger_joint"),
      &cmd_pos[num_full_dof - 1]);
  jnt_pos_interface.registerHandle(grp_pos_handle);

  hardware_interface::JointHandle grp_eff_handle(
      jnt_state_interface.getHandle("finger_joint"),
      &cmd_eff[num_full_dof - 1]);
  jnt_eff_interface.registerHandle(grp_eff_handle);

  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_vel_interface);
  registerInterface(&jnt_pos_interface);

  ROS_INFO("Register Effort Interface...");
  registerInterface(&jnt_eff_interface);

  // connect and register the joint mode interface
  // this is needed to determine if velocity or position control is needed.
  hardware_interface::JointModeHandle arm_mode_handle("joint_mode", &arm_mode);
  hardware_interface::JointModeHandle gripper_mode_handle(
      "gripper_mode", &gripper_mode);
  jm_interface.registerHandle(arm_mode_handle);
  jm_interface.registerHandle(gripper_mode_handle);

  registerInterface(&jm_interface);

  // Create the sessions so we can start using the robot
  auto createSessionInfo = Kinova::Api::Session::CreateSessionInfo();
  createSessionInfo.set_username(m_username);
  createSessionInfo.set_password(m_password);
  createSessionInfo.set_session_inactivity_timeout(
      m_api_session_inactivity_timeout_ms);
  createSessionInfo.set_connection_inactivity_timeout(
      m_api_connection_inactivity_timeout_ms);
  try
  {
    m_tcp_session_manager->CreateSession(createSessionInfo);
    ROS_INFO("Session created successfully for TCP services");

    m_udp_session_manager->CreateSession(createSessionInfo);
    ROS_INFO("Session created successfully for UDP services");
  }
  catch (std::runtime_error& ex_runtime)
  {
    std::string error_string
        = "The node could not connect to the arm. Did you specify the right IP "
          "address and is the arm powered on?";
    ROS_ERROR("%s", error_string.c_str());
    throw ex_runtime;
  }

  // Clearing faults
  try
  {
    mBase->ClearFaults();
  }
  catch (...)
  {
    std::cout << "Unable to clear robot faults" << std::endl;
    return;
  }

  finger = gripper_command.mutable_gripper()->add_finger();
  finger->set_finger_identifier(1);

  arm_mode = hardware_interface::JointCommandModes::MODE_VELOCITY;
  gripper_mode = hardware_interface::JointCommandModes::MODE_VELOCITY;
  last_arm_mode = hardware_interface::JointCommandModes::BEGIN;

  // Initialize the low pass filter
  in_lpf = new LowPassFilter(0.001, 30, num_full_dof);
  out_lpf = new LowPassFilter(0.001, 500, num_full_dof);

  // Initialize current control parameters
  if (num_arm_dof == 6)
  {
    input_current_limit = {10.0, 10.0, 10.0, 6.0, 6.0, 6.0};
    gear_ratio = {11.0, 11.0, 11.0, 7.6, 7.6, 7.6};
  }
  else
  {
    // arm with 7 dof
    input_current_limit = {10.0, 10.0, 10.0, 10.0, 6.0, 6.0, 6.0};
    gear_ratio = {11.0, 11.0, 11.0, 11.0, 7.6, 7.6, 7.6};
  }

  // pinnochio initialization
  std::string package_path, urdf_path;
  try
  {
    package_path = ros::package::getPath("kortex_description");
    if (mURDFFile.empty())
    {
      if (num_arm_dof == 6)
        mURDFFile = "gen3_6dof_vision_forque.urdf";
      else
        mURDFFile = "gen3_7dof_vision.urdf";
    }
    urdf_path = package_path + "/robots/" + mURDFFile;
  }
  catch (const std::exception& e)
  {
    std::cerr << "URDF read error: " << e.what() << std::endl;
  }
  pinocchio::urdf::buildModel(urdf_path, model);
  data = pinocchio::Data(model);
}

Gen3Robot::~Gen3Robot()
{
  try
  {
    mBase->ApplyEmergencyStop(0, {false, 0, 100});
    // clear faults
    mBase->ClearFaults();
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "Kortex exception: " << ex.what() << std::endl;

    std::cout << "Error sub-code: "
              << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                     (ex.getErrorInfo().getError().error_sub_code())))
              << std::endl;
  }

  mControlModeMessage.set_control_mode(
      k_api::ActuatorConfig::ControlMode::POSITION);
  for (int idx = 0; idx < mActuatorCount; idx++)
    mActuatorConfig->SetControlMode(mControlModeMessage, idx + 1);

  // Set the servoing mode back to Single Level
  mServoingMode.set_servoing_mode(
      k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  mBase->SetServoingMode(mServoingMode);

  // Wait for a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  m_tcp_session_manager->CloseSession();
  m_udp_session_manager->CloseSession();
  m_tcp_router->SetActivationStatus(false);
  m_udp_router->SetActivationStatus(false);
  m_tcp_transport->disconnect();
  m_udp_transport->disconnect();

  delete mBase;
  delete mBaseCyclic;

  delete m_tcp_session_manager;
  delete m_udp_session_manager;
  delete m_tcp_router;
  delete m_udp_router;
  delete m_tcp_transport;
  delete m_udp_transport;

  ros::Duration(0.10).sleep();
}

bool Gen3Robot::setControlMode(
    kortex_hardware::ModeService::Request& req,
    kortex_hardware::ModeService::Response& resp)
{
  if (req.mode == "position")
  {
    arm_mode = hardware_interface::JointCommandModes::MODE_POSITION;
    resp.success = true;
  }
  else if (req.mode == "velocity")
  {
    arm_mode = hardware_interface::JointCommandModes::MODE_VELOCITY;
    resp.success = true;
  }
  else if (req.mode == "effort")
  {
    arm_mode = hardware_interface::JointCommandModes::MODE_EFFORT;
    resp.success = true;
  }
  else if (req.mode == "stop")
  {
    arm_mode = hardware_interface::JointCommandModes::EMERGENCY_STOP;
    resp.success = true;
  }
  else
  {
    ROS_ERROR("Invalid control mode");
    resp.success = false;
  }
}

void Gen3Robot::initializeSoftLimits()
{
  ROS_INFO("Initializing soft limits for Gen3");
  // TODO: Instead convert to a init function that allows us to set soft limits
  // etc.
}

ros::Time Gen3Robot::get_time(void)
{
  return ros::Time::now();
}

ros::Duration Gen3Robot::get_period(void)
{
  return ros::Duration(0.01);
}

inline double Gen3Robot::degreesToRadians(double degrees)
{
  return (M_PI / 180.0) * degrees;
}

inline double Gen3Robot::radiansToDegrees(double radians)
{
  return (180.0 / M_PI) * radians;
}

void Gen3Robot::sendPositionCommand(const std::vector<double>& command)
{
  mLastFeedback = mBaseCyclic->RefreshFeedback();
  if (command == prev_cmd_pos)
    return;

  try
  {
    mBase->StopAction();
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "Kortex exception: " << ex.what() << std::endl;

    std::cout << "Error sub-code: "
              << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                     (ex.getErrorInfo().getError().error_sub_code())))
              << std::endl;
  }
  prev_cmd_pos = command;

  auto action = k_api::Base::Action();

  auto reach_joint_angles = action.mutable_reach_joint_angles();
  auto joint_angles = reach_joint_angles->mutable_joint_angles();

  auto actuator_count = mBase->GetActuatorCount();

  for (size_t i = 0; i < actuator_count.count(); ++i)
  {
    auto joint_angle = joint_angles->add_joint_angles();
    joint_angle->set_joint_identifier(i);
    joint_angle->set_value(radiansToDegrees(command.at(i)));
  }

  std::cout << "Executing action" << std::endl;
  try
  {
    mBase->ExecuteAction(action);
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "Kortex exception: " << ex.what() << std::endl;

    std::cout << "Error sub-code: "
              << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                     (ex.getErrorInfo().getError().error_sub_code())))
              << std::endl;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "runtime error: " << ex2.what() << std::endl;
  }
  catch (...)
  {
    std::cout << "Unknown error." << std::endl;
  }
}

void Gen3Robot::sendGripperPositionCommand(const float& command)
{
  std::cout << "Sending gripper position command: " << command << std::endl;
  finger->set_value(command);
  gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
  try
  {
    mBase->SendGripperCommand(gripper_command);
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "Kortex exception: " << ex.what() << std::endl;

    std::cout << "Error sub-code: "
              << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                     (ex.getErrorInfo().getError().error_sub_code())))
              << std::endl;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "runtime error: " << ex2.what() << std::endl;
  }
  catch (...)
  {
    std::cout << "Unknown error." << std::endl;
  }
}

void Gen3Robot::sendVelocityCommand(const std::vector<double>& command)
{
  mLastFeedback = mBaseCyclic->RefreshFeedback();

  auto action = k_api::Base::Action();
  action.set_name("angular action movement");
  action.set_application_data("");

  auto joint_speeds = action.mutable_send_joint_speeds();

  for (std::size_t i = 0; i < command.size() - 1; ++i)
  {
    auto joint_speed = joint_speeds->add_joint_speeds();
    joint_speed->set_joint_identifier(i);
    joint_speed->set_value(radiansToDegrees(command.at(i)));
    joint_speed->set_duration(0.1); // TODO: magic number 0.1
  }

  try
  {
    mBase->SendJointSpeedsCommand(*joint_speeds);
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "Kortex exception: " << ex.what() << std::endl;

    std::cout << "Error sub-code: "
              << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                     (ex.getErrorInfo().getError().error_sub_code())))
              << std::endl;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "runtime error: " << ex2.what() << std::endl;
  }
  catch (...)
  {
    std::cout << "Unknown error." << std::endl;
  }
}

void Gen3Robot::sendGripperVelocityCommand(const float& command)
{
  std::cout << "Sending gripper velocity command: " << command << std::endl;
  gripper_command.set_mode(k_api::Base::GRIPPER_SPEED);
  finger->set_value(command);
  try
  {
    mBase->SendGripperCommand(gripper_command);
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "Kortex exception: " << ex.what() << std::endl;

    std::cout << "Error sub-code: "
              << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                     (ex.getErrorInfo().getError().error_sub_code())))
              << std::endl;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "runtime error: " << ex2.what() << std::endl;
  }
  catch (...)
  {
    std::cout << "Unknown error." << std::endl;
  }
}

void Gen3Robot::setBaseCommand()
{
  k_api::BaseCyclic::Command base_command;
  // Initialize each actuator to their current position
  for (unsigned int i = 0; i < mActuatorCount; i++)
  {
    base_command.add_actuators()->set_position(
        mLastFeedback.actuators(i).position());
  }
  mBaseCommand = base_command;

  // Initialize gripper low level command pointer
  gripper_low_level_cmd = mBaseCommand.mutable_interconnect()
                              ->mutable_gripper_command()
                              ->add_motor_cmd();
  // Set position to current gripper position
  gripper_low_level_cmd->set_position(pos[num_full_dof - 1] * 100);
  gripper_low_level_cmd->set_velocity(0.0);
  gripper_low_level_cmd->set_force(100.0);
}

void Gen3Robot::switchToEffortMode()
{
  bool return_status = true;

  // Get actuator count
  mActuatorCount = mBase->GetActuatorCount().count();

  // Clearing faults
  try
  {
    mBase->ClearFaults();
  }
  catch (...)
  {
    std::cout << "Unable to clear robot faults" << std::endl;
    return;
  }

  k_api::BaseCyclic::Feedback base_feedback;

  mServoingMode.set_servoing_mode(
      k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  mBase->SetServoingMode(mServoingMode);
  mLowLevelServoing = true;
  setBaseCommand();

  // Taken from Kinova API
  // Send a first frame
  mLastFeedback = mBaseCyclic->Refresh(mBaseCommand);

  // Taken from Kinova API
  // Set all actuators to torque mode now that the command is equal to measure
  if (current_control)
    mControlModeMessage.set_control_mode(
        k_api::ActuatorConfig::ControlMode::CURRENT);
  else
    mControlModeMessage.set_control_mode(
        k_api::ActuatorConfig::ControlMode::TORQUE);
  for (int idx = 1; idx < mActuatorCount + 1; idx++)
    mActuatorConfig->SetControlMode(mControlModeMessage, idx);
  std::cout << "Switched successfully to effort mode" << std::endl;
}

void Gen3Robot::sendTorqueCommand(std::vector<double>& command)
{
  if (!mLowLevelServoing)
    return;
  addGravityCompensation(model, data, pos, command);

  // // Initialize each actuator to their current position
  for (unsigned int i = 0; i < mActuatorCount; i++)
  {
    // Taken from Kinova API
    // Position command to first actuator is set to measured one to avoid
    // following error to trigger Bonus: When doing this instead of disabling
    // the following error, if communication is lost and first
    //        actuator continues to move under torque command, resulting
    //        position error with command will trigger a following error and
    //        switch back the actuator in position command to hold its position
    mBaseCommand.mutable_actuators(i)->set_position(
        mLastFeedback.actuators(i).position());
  }
  now = GetTickUs();
  int rate = now - last;
  try
  {
    for (unsigned int idx = 0; idx < mActuatorCount; idx++)
      mBaseCommand.mutable_actuators(idx)->set_torque_joint(command.at(idx));

    // Incrementing identifier ensures actuators can reject out of time frames
    mBaseCommand.set_frame_id(mBaseCommand.frame_id() + 1);
    if (mBaseCommand.frame_id() > 65535)
      mBaseCommand.set_frame_id(0);

    for (unsigned int idx = 0; idx < mActuatorCount; idx++)
    {
      mBaseCommand.mutable_actuators(idx)->set_command_id(
          mBaseCommand.frame_id());
    }
    mLastFeedback = mBaseCyclic->Refresh(mBaseCommand, 0);
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "API error: " << ex.what() << std::endl;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "Error: " << ex2.what() << std::endl;
  }

  last = GetTickUs();
  prev_cmd_eff = command;
}

void Gen3Robot::sendCurrentCommand(std::vector<double>& command)
{
  if (!mLowLevelServoing)
    return;
  addGravityCompensation(model, data, pos, command);

  // Convert torque to current
  // motor coefficient 1~4 : 11 Nm/A, 5~7 : 7.6 Nm/A (consider gear ratio)
  for (int i = 0; i < num_arm_dof; i++)
  {
    command[i] = command[i] / gear_ratio[i];
  }
  if (is_out_lpf_initialized != true)
  {
    out_lpf->initLPF(command);
    is_out_lpf_initialized = true;
  }
  // lpf->getFilteredEffort(eff);
  std::vector<double> filteredCommand = out_lpf->getFilteredEffort(command);
  command = filteredCommand;

  // To avoid current warning
  for (int i = 0; i < mActuatorCount; i++)
  {
    if (std::fabs(command.at(i)) > input_current_limit.at(i))
    {
      command.at(i) = input_current_limit.at(i) * command.at(i)
                      / std::fabs(command.at(i));
    }
  }
  prev_cmd_eff = command;

  // TODO: Check this. Manually delay to match the rate of 1kHz; ROS rate can be
  // inaccurate sometimes
  while (now - last < 1000)
  {
    now = GetTickUs();
  }
  try
  {
    // Initialize each actuator to their current position and set current
    // command
    for (unsigned int i = 0; i < mActuatorCount; i++)
    {
      // Taken from Kinova API
      // Position command to first actuator is set to measured one to avoid
      // following error to trigger Bonus: When doing this instead of disabling
      // the following error, if communication is lost and first
      //        actuator continues to move under torque command, resulting
      //        position error with command will trigger a following error and
      //        switch back the actuator in position command to hold its
      //        position
      mBaseCommand.mutable_actuators(i)->set_position(
          mLastFeedback.actuators(i).position());
      mBaseCommand.mutable_actuators(i)->set_current_motor(command.at(i));
    }

    // Incrementing identifier ensures actuators can reject out of time frames
    mBaseCommand.set_frame_id(mBaseCommand.frame_id() + 1);
    if (mBaseCommand.frame_id() > 65535)
      mBaseCommand.set_frame_id(0);

    for (unsigned int idx = 0; idx < mActuatorCount; idx++)
    {
      mBaseCommand.mutable_actuators(idx)->set_command_id(
          mBaseCommand.frame_id());
    }
    mLastFeedback = mBaseCyclic->Refresh(mBaseCommand, 0);
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "API error: " << ex.what() << std::endl;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "Error: " << ex2.what() << std::endl;
  }
  last = GetTickUs();
}

void Gen3Robot::sendGripperLowLevelCommand(const float& command)
{
  gripper_position_error = (command - pos[num_full_dof - 1]) * 100.0;

  if (fabs(gripper_position_error) < 1.5)
  {
    gripper_low_level_cmd->set_velocity(0.0);
    return;
  }

  gripper_velocity = gripper_proportional_gain * fabs(gripper_position_error);
  if (gripper_velocity > 100.0)
  {
    gripper_velocity = 100.0;
  }

  gripper_low_level_cmd->set_position(command * 100.0);
  gripper_low_level_cmd->set_velocity(gripper_velocity);
}

void Gen3Robot::write(void)
{
  // Ensures safe switching between modes and servoing levels
  if (last_arm_mode != arm_mode)
  {
    try
    {
      // clear faults if any
      mBase->ClearFaults();
      std::cout << "Switching control mode" << std::endl;
    }
    catch (k_api::KDetailedException& ex)
    {
      std::cout << "Kortex exception: " << ex.what() << std::endl;

      std::cout << "Error sub-code: "
                << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                       (ex.getErrorInfo().getError().error_sub_code())))
                << std::endl;
    }
    if (last_arm_mode == hardware_interface::JointCommandModes::MODE_POSITION)
      prev_cmd_pos.clear();
    if (last_arm_mode == hardware_interface::JointCommandModes::MODE_EFFORT)
    {
      prev_cmd_eff.clear();
      // Set first actuator back in position
      mControlModeMessage.set_control_mode(
          k_api::ActuatorConfig::ControlMode::POSITION);
      for (int idx = 0; idx < mActuatorCount; idx++)
        mActuatorConfig->SetControlMode(mControlModeMessage, idx + 1);

      // Set the servoing mode back to Single Level
      mServoingMode.set_servoing_mode(
          k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      mBase->SetServoingMode(mServoingMode);
      mLowLevelServoing = false;

      // Wait for a bit
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    if (arm_mode == hardware_interface::JointCommandModes::MODE_EFFORT
        || arm_mode == hardware_interface::JointCommandModes::EMERGENCY_STOP)
    {
      switchToEffortMode();
    }
    last_arm_mode = arm_mode;
  }

  if (mUseGripper)
  {
    if (mLowLevelServoing)
    {
      // Use gripper in low level servoing mode if arm is in
      // low level servoing mode
      // TODO: Add support for input velocity and force limits
      // See: api_cpp/examples/01-gripper_low_level_command.cpp
      sendGripperLowLevelCommand(cmd_pos[num_full_dof - 1]);
    }
    else
    {
      switch (gripper_mode)
      {
        case hardware_interface::JointCommandModes::MODE_VELOCITY:
          sendGripperVelocityCommand(cmd_vel[num_full_dof - 1]);
          break;
        case hardware_interface::JointCommandModes::MODE_POSITION:
          sendGripperPositionCommand(cmd_pos[num_full_dof - 1]);
          break;
        default:
          // Stop Gripper
          sendGripperVelocityCommand(0);
      }
    }
  }
  switch (arm_mode)
  {
    case hardware_interface::JointCommandModes::MODE_VELOCITY:
      sendVelocityCommand(cmd_vel);
      break;
    case hardware_interface::JointCommandModes::MODE_POSITION:
      sendPositionCommand(cmd_pos);
      break;
    case hardware_interface::JointCommandModes::MODE_EFFORT:
      if (current_control)
        sendCurrentCommand(cmd_eff);
      else
        sendTorqueCommand(cmd_eff);
      break;
    case hardware_interface::JointCommandModes::EMERGENCY_STOP: {
      vector<double> zero_torque(num_full_dof, 0.0);
      if (current_control)
        sendCurrentCommand(zero_torque);
      else
        sendTorqueCommand(zero_torque);
      break;
    }
    default:
      // Stop Arm
      vector<double> zero(num_full_dof, 0.0);
      sendVelocityCommand(zero);
  }
}

void Gen3Robot::read(void)
{
  // Read the feedback
  if (!mFirstFeedbackReceived || mUseAdmittance)
  {
    mLastFeedback = mBaseCyclic->RefreshFeedback();
    mFirstFeedbackReceived = true;
  }
  for (std::size_t i = 0; i < num_arm_dof; ++i)
  {
    pos[i] = degreesToRadians(double(mLastFeedback.actuators(i).position()));
    if (pos[i] > M_PI)
      pos[i] -= 2 * M_PI;
    vel[i] = degreesToRadians(double(mLastFeedback.actuators(i).velocity()));
    eff[i] = double(mLastFeedback.actuators(i).torque());
  }

  if (is_in_lpf_initialized != true)
  {
    in_lpf->initLPF(eff);
    is_in_lpf_initialized = true;
  }
  // in_lpf->getFilteredEffort(eff);
  // std::vector<double> filteredEff = in_lpf->getFilteredEffort(eff);
  // eff = filteredEff;

  // Read finger state. Note: position and velocity are percentage values
  // (0-100). Effort is set as current consumed by gripper motor (mA).
  pos[num_full_dof - 1]
      = mLastFeedback.interconnect().gripper_feedback().motor()[0].position()
        / 100.0;
  vel[num_full_dof - 1]
      = mLastFeedback.interconnect().gripper_feedback().motor()[0].velocity()
        / 100.0;
  eff[num_full_dof - 1] = mLastFeedback.interconnect()
                              .gripper_feedback()
                              .motor()[0]
                              .current_motor();
}
