// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Utils/Widgets/SwerveWidget.h"

#include <memory>
#include <utility>

#include <networktables/DoubleArrayTopic.h>
#include <networktables/NTSendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>


SwerveWidget::SwerveWidget() {
    wpi::SendableRegistry::Add(this, "SwerveWidget");
}

SwerveWidget::SwerveWidget(SwerveWidget&& rhs) : SendableHelper(std::move(rhs)) {
    std::swap(m_table, rhs.m_table);
}

SwerveWidget& SwerveWidget::operator=(SwerveWidget&& rhs) {
    SendableHelper::operator=(std::move(rhs));
    std::swap(m_table, rhs.m_table);
    return *this;
}

void SwerveWidget::Update(ctre::phoenix6::swerve::impl::SwerveDrivetrainImpl::SwerveDriveState swerveState){
    m_robotAngle = swerveState.Pose.Rotation();

    for(int i = 0; i < 4; i++){
        m_moduleStates[i] = swerveState.ModuleStates[i];
    }
}

void SwerveWidget::Update(wpi::array<frc::SwerveModuleState, 4U> moduleStates, frc::Rotation2d robotHeading){
    m_robotAngle = robotHeading;
    m_moduleStates = moduleStates;
}


void SwerveWidget::InitSendable(nt::NTSendableBuilder& builder) {
    builder.SetSmartDashboardType("SwerveDrive");

        builder.AddDoubleProperty("Front Right Angle", ([this]{return m_moduleStates[0].angle.Degrees().value();}), nullptr);
        builder.AddDoubleProperty("Front Right Velocity", ([this]{return m_moduleStates[0].speed.value();}), nullptr);

        builder.AddDoubleProperty("Front Left Angle", ([this]{return m_moduleStates[1].angle.Degrees().value();}), nullptr);
        builder.AddDoubleProperty("Front Left Velocity", ([this]{return m_moduleStates[1].speed.value();}), nullptr);

        builder.AddDoubleProperty("Back Left Angle", ([this]{return m_moduleStates[2].angle.Degrees().value();}), nullptr);
        builder.AddDoubleProperty("Back Left Velocity", ([this]{return m_moduleStates[2].speed.value();}), nullptr);

        builder.AddDoubleProperty("Back Right Angle", ([this]{return m_moduleStates[3].angle.Degrees().value();}), nullptr);
        builder.AddDoubleProperty("Back Right Velocity", ([this]{return m_moduleStates[3].speed.value();}), nullptr);

        builder.AddDoubleProperty("Robot Angle", ([this]{return m_robotAngle.Degrees().value();}), nullptr);

    m_table = builder.GetTable();
}
