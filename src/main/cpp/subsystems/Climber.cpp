// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

using namespace ClimberConstants;

Climber::Climber(): 
m_MiddleClimber(kCanIDOne, kCanBus), m_PoseRequest(0_tr){
    m_MiddleClimber.GetConfigurator().Apply(KMotorOneConfigs);
};

units::angle::turn_t Climber::GetMiddleClimberPosition()
{
     return m_MiddleClimber.GetPosition().GetValue();
};

frc2::CommandPtr Climber::LowerClimber(){
          return SetClimberPosition(kPivotLowerLimit);
};

frc2::CommandPtr Climber::RaiseClimber(){
          return SetClimberPosition(kPivotUpperLimit);
};


void Climber::Periodic() {}

units::angle::turn_t Climber::InchToTurns(units::length::inch_t inch){
     return inch*kTurnsPerInch;
}

frc2::CommandPtr Climber::SetClimberPosition(units::length::inch_t pos){
          return RunOnce([this, pos] {
               if(IsValidPosition(pos)){
                    m_MiddleClimber.SetControl(m_PoseRequest.WithPosition(InchToTurns(pos)));
               }
     });
};

bool Climber::IsValidPosition(units::length::inch_t pos) {
     return (pos>=kPivotLowerLimit && pos<=kPivotUpperLimit);
}

frc2::CommandPtr Climber::Climb(){
    frc2::CommandPtr command = frc2::cmd::Sequence(
          RaiseClimber(),
          LowerClimber()      
    );

    return command;
}