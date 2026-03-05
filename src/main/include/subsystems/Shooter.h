// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>

#include <ctre/phoenix6/controls/Follower.hpp>

#include "Constants.h"

#include <ctre/phoenix6/CANBus.hpp>

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  ////////   FLYWHEELS

  //Flywheel one and two (one is left, two is right) Units: Turns per second

  frc2::CommandPtr SetFlywheelVel(units::turns_per_second_t vel);

  frc2::CommandPtr SetFeedVel(units::turns_per_second_t vel);

  frc2::CommandPtr RunFlywheel();

  frc2::CommandPtr RunFeed();

  units::turns_per_second_t GetFlywheelVel();

  ////////   FLAP

  //Flap (rotates to angle the fuel) Units: Turns

  frc2::CommandPtr SetFlapPosition(units::angle::turn_t position);

  units::angle::turn_t GetFlapPosition();


  
  //Will be called periodically whenever the CommandScheduler runs.
  void Periodic() override;

 private:


  ctre::phoenix6::hardware::TalonFX m_FlywheelL;

  ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC m_VelRequestOne;

  
  ctre::phoenix6::hardware::TalonFX m_FlywheelR;

  ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC m_VelRequestTwo;

  
  ctre::phoenix6::hardware::TalonFX m_ShooterFeed;

  ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC m_VelRequestFeed;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
