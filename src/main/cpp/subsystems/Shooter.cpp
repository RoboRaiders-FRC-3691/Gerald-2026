// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

using namespace ShooterConstants;

Shooter::Shooter() : 
m_FlywheelL(kCanIDL, kCanBus), m_VelRequestOne(0_rpm), 
m_FlywheelR(kCanIDR, kCanBus), m_VelRequestTwo(0_rpm),
m_ShooterFeed(kCanIDFeed, kCanBus), m_VelRequestFeed(0_rpm) {
    m_FlywheelL.GetConfigurator().Apply(KFlywheelL);
    m_FlywheelR.GetConfigurator().Apply(KFlywheelR);
    m_ShooterFeed.GetConfigurator().Apply(KFeedConfigs);
    m_FlywheelL.SetControl(ctre::phoenix6::controls::Follower{m_FlywheelR.GetDeviceID(), InvertFollowDir});
}

frc2::CommandPtr Shooter::SetFlywheelVel(units::turns_per_second_t vel){
    return RunOnce([this, vel] {
                m_FlywheelR.SetControl(m_VelRequestTwo.WithVelocity(vel));
                m_ShooterFeed.SetControl(m_VelRequestFeed.WithVelocity(kShooterFeedSpeed));
        });
}

units::angular_velocity::turns_per_second_t Shooter::GetFlywheelVel(){
    return m_FlywheelR.GetVelocity().GetValue();
}


// This method will be called once per scheduler run
void Shooter::Periodic() {}
