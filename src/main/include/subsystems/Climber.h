// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>

#include "Constants.h"

#include <ctre/phoenix6/CANBus.hpp>

class Climber : public frc2::SubsystemBase {
 public:
  Climber();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
private:

   units::angle::turn_t InchToTurns(units::length::inch_t inch);

   frc2::CommandPtr SetMotorPosition(units::length::inch_t positionOne);

public:

   units::angle::turn_t GetMiddleClimberPosition();

   frc2::CommandPtr LowerClimber();

   frc2::CommandPtr RaiseClimber();

  void Periodic() override;

 private:

  ctre::phoenix6::hardware::TalonFX m_MiddleClimber;

  ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC m_PoseRequestOne;
  
};
