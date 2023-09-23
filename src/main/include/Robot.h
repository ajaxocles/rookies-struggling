// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <rev/CANSparkMax.h>
#include <frc/XboxController.h>
#include <frc/DigitalInput.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kTestThrottle  = "Throttle";
  const std::string kTestLimit     = "Limit";
  const std::string kTestFollower  = "Follower";
  const std::string kTestPosition  = "Position";
  const std::string kTestVelocity  = "Velocity";

  std::string m_testSelected;

  enum testMode {
    TEST_THROTTLE,
    TEST_LIMIT_SWITCH,
    TEST_FOLLOWER,
    TEST_POSITION_PID,
    TEST_VELOCITY_PID,
  } m_testMode;

  rev::CANSparkMax m_right{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_left{4, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxRelativeEncoder m_leftEncoder  = m_left.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightEncoder = m_right.GetEncoder();

  rev::SparkMaxPIDController m_pid = m_left.GetPIDController();

  frc::DigitalInput m_limit_switch{0};

  frc::XboxController m_stick{0};

  double m_rpm       = 0.0;
  double m_position  = 0.0;
  double m_softLimit = 100.0;

  // PID coefficient structure
  struct pidCoeff {
    double kP;
    double kI;
    double kD;
    double kIz;
    double kFF;
    double kMinOutput;
    double kMaxOutput;
  };
  pidCoeff m_pidCoeff {0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};

};
