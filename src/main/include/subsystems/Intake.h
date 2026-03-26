// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/controls/MotionMagicTorqueCurrentFOC.hpp>

#include "Constants.h"

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  void Periodic() override;

  frc2::CommandPtr SetAngle(units::angle::turn_t turns);

  frc2::CommandPtr DropIntake();

  frc2::CommandPtr RaiseIntake();

  units::turn_t GetAnglePivotMotor();

  frc2::CommandPtr SetVel(units::turns_per_second_t vel);

  frc2::CommandPtr RunIntake();

  frc2::CommandPtr RunIntakeReverse();

  units::turns_per_second_t GetSpeedRollerMotor();

  bool IsValidPosition(units::turn_t);
  
  //void Periodic() override;

 private:

  ctre::phoenix6::hardware::TalonFX m_PivotMotor;
  ctre::phoenix6::controls::MotionMagicVoltage m_PoseRequest;

  ctre::phoenix6::hardware::TalonFX m_RollerMotor;
  ctre::phoenix6::controls::MotionMagicVelocityVoltage m_VelRequest;
};
