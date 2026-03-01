// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

using namespace IntakeConstants;

Intake::Intake(): 
m_PivotMotor(kCanIDPivot, kCanBus), m_PoseRequest(0_tr),
m_RollerMotor(kCanIDRoller, kCanBus), m_VelRequest(0_rpm) {
    m_PivotMotor.GetConfigurator().Apply(KPivotConfigs);
    m_RollerMotor.GetConfigurator().Apply(KRollerConfigs);
}

frc2::CommandPtr Intake::SetAngle(units::turn_t pos){
      return RunOnce([this, pos] {
          m_PivotMotor.SetControl(m_PoseRequest.WithPosition(pos));
  });
}

units::turn_t Intake::GetAnglePivotMotor(){
  return m_PivotMotor.GetPosition().GetValue();
}

units::turn_t Intake::GetPivotMax(){
  return  kPivotUpperLimit;
}

units::turn_t Intake::GetPivotMin(){
  return  kPivotLowerLimit;
}

frc2::CommandPtr Intake::SetVel(units::turns_per_second_t vel){
      return RunOnce([this, vel] {
          m_RollerMotor.SetControl(m_VelRequest.WithVelocity(vel));
  });
}

units::angular_velocity::turns_per_second_t Intake::GetSpeedRollerMotor(){
  return m_PivotMotor.GetVelocity().GetValue();
}

void Intake::Periodic() {}
