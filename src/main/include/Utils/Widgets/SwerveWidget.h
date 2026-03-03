#pragma once

#include <memory>
#include <string_view>
#include <sstream>

#include <networktables/NTSendable.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/DoubleTopic.h>

#include <units/length.h>
#include <wpi/array.h>
#include <wpi/sendable/SendableHelper.h>

#include "ctre/phoenix6/swerve/impl/SwerveDrivetrainImpl.hpp"
#include "frc/kinematics/SwerveModuleState.h"

class SwerveWidget : public nt::NTSendable, public wpi::SendableHelper<SwerveWidget> {
    public:
        // SwerveWidget constructor
        SwerveWidget();

        SwerveWidget(SwerveWidget&& rhs);
        SwerveWidget& operator=(SwerveWidget&& rhs);

        void InitSendable(nt::NTSendableBuilder& builder) override;

        // Update the swerve widget stored drive state
        void Update(ctre::phoenix6::swerve::impl::SwerveDrivetrainImpl::SwerveDriveState swerveState);

        // Update the swerve widget stored drive state
        void Update(wpi::array<frc::SwerveModuleState, 4U> moduleStates, frc::Rotation2d robotHeading);

    private:
        // Define widget network table
        std::shared_ptr<nt::NetworkTable> m_table;

        // Define cached swerve state
        wpi::array<frc::SwerveModuleState, 4U> m_moduleStates = {frc::SwerveModuleState(), frc::SwerveModuleState(), frc::SwerveModuleState(), frc::SwerveModuleState()};
        frc::Rotation2d m_robotAngle;

};

