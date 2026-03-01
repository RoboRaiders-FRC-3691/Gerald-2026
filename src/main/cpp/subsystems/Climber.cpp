// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

using namespace ClimberConstants;

Climber::Climber(): 
m_MiddleClimber(kCanIDOne, kCanBus), m_PoseRequestOne(0_tr){
    m_MiddleClimber.GetConfigurator().Apply(KMotorOneConfigs);
};

// This method will be called once per scheduler run
void Climber::Periodic() {}

   units::angle::turn_t Climber::InchToTurns(units::length::inch_t inch){
          return units::turn_t(inch.value()*kTurnsPerInch.value()); 
   }

   frc2::CommandPtr Climber::SetMotorPosition(units::length::inch_t pos){
            return RunOnce([this, pos] {
                m_MiddleClimber.SetControl(m_PoseRequestOne.WithPosition(InchToTurns(pos)));
        });
   };

   units::angle::turn_t Climber::GetMiddleClimberPosition(){
        return m_MiddleClimber.GetPosition().GetValue();
   };

   frc2::CommandPtr Climber::LowerClimber(){
            return RunOnce([this] {
                    SetMotorPosition(kPivotUpperLimit);
                  
        });
   };

   frc2::CommandPtr Climber::RaiseClimber(){
            return RunOnce([this] {
                SetMotorPosition(kPivotUpperLimit);
                SetMotorPosition(kPivotLowerLimit);
        });
   };

