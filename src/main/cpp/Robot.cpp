// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kTestThrottle, kTestThrottle);
  m_chooser.AddOption(kTestLimit, kTestLimit);
  m_chooser.AddOption(kTestFollower, kTestFollower);
  m_chooser.AddOption(kTestPosition, kTestPosition);
  m_chooser.AddOption(kTestVelocity, kTestVelocity);

  frc::SmartDashboard::PutData("Test Modes", &m_chooser);
  frc::SmartDashboard::PutNumber("Position", m_position);
  frc::SmartDashboard::PutNumber("Velocity", m_rpm);

  // Put default PID coefficients on dashboard
  frc::SmartDashboard::PutNumber("P Gain", m_pidCoeff.kP);
  frc::SmartDashboard::PutNumber("I Gain", m_pidCoeff.kI);
  frc::SmartDashboard::PutNumber("D Gain", m_pidCoeff.kD);
  frc::SmartDashboard::PutNumber("I Zone", m_pidCoeff.kIz);
  frc::SmartDashboard::PutNumber("Feed Forward", m_pidCoeff.kFF);
  frc::SmartDashboard::PutNumber("Max Output", m_pidCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Min Output", m_pidCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Position Soft Limit", m_softLimit);
  frc::SmartDashboard::PutNumber("Desired Position", m_position);
  frc::SmartDashboard::PutNumber("Desired Velocity", m_rpm);

  m_left.RestoreFactoryDefaults();
  m_right.RestoreFactoryDefaults();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("Left Position", m_leftEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Right Position", m_rightEncoder.GetPosition());

  frc::SmartDashboard::PutNumber("Left RPM", m_leftEncoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Right RPM", m_rightEncoder.GetVelocity());
}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
  // Read test type
  m_testSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Test selected: {}\n", m_testSelected);

  if (m_testSelected == kTestThrottle) {
    m_testMode = TEST_THROTTLE;
  }
  if (m_testSelected == kTestLimit) {
    m_testMode = TEST_LIMIT_SWITCH;
  }
  if (m_testSelected == kTestFollower) {
    m_testMode = TEST_FOLLOWER;
  }
  if (m_testSelected == kTestPosition) {
    m_testMode = TEST_POSITION_PID;
  }
  if (m_testSelected == kTestVelocity) {
    m_testMode = TEST_VELOCITY_PID;
  } 

  // Get PID coefficients from dashboard
  m_pidCoeff.kP         = frc::SmartDashboard::GetNumber("P Gain", m_pidCoeff.kP);
  m_pidCoeff.kI         = frc::SmartDashboard::GetNumber("I Gain", m_pidCoeff.kI);
  m_pidCoeff.kD         = frc::SmartDashboard::GetNumber("D Gain", m_pidCoeff.kD);
  m_pidCoeff.kIz        = frc::SmartDashboard::GetNumber("I Zone", m_pidCoeff.kIz);
  m_pidCoeff.kFF        = frc::SmartDashboard::GetNumber("Feed Forward", m_pidCoeff.kFF);
  m_pidCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Max Output", m_pidCoeff.kMaxOutput);
  m_pidCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Min Output", m_pidCoeff.kMinOutput);

  // Set PID coefficients
  m_pid.SetP(m_pidCoeff.kP);
  m_pid.SetI(m_pidCoeff.kI);
  m_pid.SetD(m_pidCoeff.kD);
  m_pid.SetIZone(m_pidCoeff.kIz);
  m_pid.SetFF(m_pidCoeff.kFF);
  m_pid.SetOutputRange(m_pidCoeff.kMinOutput, m_pidCoeff.kMaxOutput);

  m_position  = frc::SmartDashboard::GetNumber("Desired Position", m_position);
  m_rpm       = frc::SmartDashboard::GetNumber("Desired Velocity", m_rpm);
  m_softLimit = frc::SmartDashboard::GetNumber("Position Soft Limit", m_softLimit);

  // Set Folloer mode
  if (m_testMode == TEST_FOLLOWER) {
      m_right.Follow(m_left, true);
  }

  if (m_testMode == TEST_POSITION_PID) {
      m_left.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, m_softLimit);
  }

  // Zero encoders
  m_leftEncoder.SetPosition(0.0);
  m_rightEncoder.SetPosition(0.0);
}

void Robot::TeleopPeriodic() {

  switch (m_testMode)
  {
    case TEST_THROTTLE:
      // Percent Throttle
      m_left.Set(m_stick.GetLeftX());
      break;

    case TEST_LIMIT_SWITCH:
      // Limit Switch Stop
      if (!m_limit_switch.Get()) {
        m_left.Set(m_stick.GetLeftX());
      }
      break;

    case TEST_FOLLOWER:
      // Follow
      m_left.Set(m_stick.GetLeftX());
      break;

    case TEST_POSITION_PID:
      // Position PID
      if (m_stick.GetAButtonPressed()) {
        m_pid.SetReference(m_position, rev::CANSparkMax::ControlType::kPosition);
      }
      break;

    case TEST_VELOCITY_PID:
      // Velocity PID
      if (m_stick.GetAButton()) {
        m_pid.SetReference(m_rpm, rev::CANSparkMax::ControlType::kVelocity);
      } else {
        m_pid.SetReference(0.0, rev::CANSparkMax::ControlType::kVelocity);
      }
      break;
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
