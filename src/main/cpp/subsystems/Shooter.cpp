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
        });
}

frc2::CommandPtr Shooter::SetFeedVel(units::turns_per_second_t vel){
    return RunOnce([this, vel] {
                m_ShooterFeed.SetControl(m_VelRequestFeed.WithVelocity(vel));
        });
}

frc2::CommandPtr Shooter::RunFlywheel(){
    return SetFlywheelVel(kShooterFlywheelConstant);
}

frc2::CommandPtr Shooter::RunFeed(){
    return StartEnd([this]{
        m_ShooterFeed.SetControl(m_VelRequestFeed.WithVelocity(kFeedVelocityConstant));
    },[this]{
        m_ShooterFeed.StopMotor();
    });
}


frc2::CommandPtr Shooter::ShootFor(units::time::second_t duration){
    using frc2::cmd::WaitUntil;
    using frc2::cmd::Wait;

    frc2::CommandPtr command = frc2::cmd::Sequence(
        SetFlywheelVel(kShooterFlywheelConstant),
        WaitUntil([this] { return FlywheelTargetVelocityReached(); }),
        SetFeedVel(kFeedVelocityConstant),
        Wait(duration),
        SetFlywheelVel(0_tps),
        SetFeedVel(0_tps)
    );

    return command;
}

ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> Shooter::GetFlywheelVel(){
    return m_FlywheelR.GetVelocity();
}

bool Shooter::FlywheelTargetVelocityReached(){
    return GetFlywheelVel().IsNear(kShooterFlywheelConstant, kShooterFlywheelTolerance); 
}


// This method will be called once per scheduler run
void Shooter::Periodic() {}

