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

frc2::CommandPtr Intake::SetAngle(units::angle::turn_t pos){
      return RunOnce([this, pos] {
           if(IsValidPosition(pos)){
                  m_PivotMotor.SetControl(m_PoseRequest.WithPosition(pos));
               }
  });
}

frc2::CommandPtr Intake::DropIntake(){
      return SetAngle(kPivotLowerLimit);
  };


frc2::CommandPtr Intake::RaiseIntake(){
      return SetAngle(kPivotUpperLimit);
  };


units::turn_t Intake::GetAnglePivotMotor(){
  return m_PivotMotor.GetPosition().GetValue();
}

frc2::CommandPtr Intake::SetVel(units::turns_per_second_t vel){
      return RunOnce([this, vel] {
          m_RollerMotor.SetControl(m_VelRequest.WithVelocity(vel));
  });
}

frc2::CommandPtr Intake::RunIntake(){
    return StartEnd([this]{
        m_RollerMotor.SetControl(m_VelRequest.WithVelocity(kIntakeVelocityConstant));
    },[this]{
        m_RollerMotor.StopMotor();
    });
}

frc2::CommandPtr Intake::RunIntakeReverse(){
  return StartEnd([this]{
      m_RollerMotor.SetControl(m_VelRequest.WithVelocity(-kIntakeVelocityConstant));
  },[this]{
      m_RollerMotor.StopMotor();
  });
}

units::angular_velocity::turns_per_second_t Intake::GetSpeedRollerMotor(){
  return m_PivotMotor.GetVelocity().GetValue();
}

bool Intake::IsValidPosition(units::turn_t pos) {
     return (pos>=kPivotLowerLimit && pos<=kPivotUpperLimit);
}

void Intake::Periodic() {}
