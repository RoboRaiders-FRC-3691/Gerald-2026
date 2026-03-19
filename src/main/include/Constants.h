// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



#pragma once

#include <units/time.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>
#include <units/length.h>
#include <frc2/command/CommandPtr.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANcoder.hpp"

#include "ctre/phoenix6/configs/Configurator.hpp"
#include "ctre/phoenix6/configs/Slot0Configs.hpp"

#include <filesystem>


// Define the compound unit type: turns per inch (uses camel case for units name to match units library conventions)
using turns_per_inch = units::compound_unit<units::turns, units::inverse<units::inches>>;



namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;

}

namespace IntakeConstants {
    inline constexpr ctre::phoenix6::CANBus kCanBus{"*"};

    inline constexpr int kCanIDCANCoder = 5;

    inline constexpr int kCanIDPivot = 21;
    inline constexpr units::angle::turn_t kPivotUpperLimit = .36_tr;
    inline constexpr units::angle::turn_t kPivotLowerLimit = -.07_tr;

    inline constexpr int kCanIDRoller = 20;
    inline constexpr units::turns_per_second_t kIntakeVelocityConstant = -65_tps;

    static constexpr ctre::phoenix6::configs::CANcoderConfiguration kIntakeCANCoderConfigs = ctre::phoenix6::configs::CANcoderConfiguration{}
        .WithMagnetSensor(ctre::phoenix6::configs::MagnetSensorConfigs{}
            .WithMagnetOffset(0.3310546875_tr)
            .WithSensorDirection(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        );

    static constexpr ctre::phoenix6::configs::TalonFXConfiguration KPivotConfigs = ctre::phoenix6::configs::TalonFXConfiguration{}
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs{}
                    
            .WithKP(20)
            .WithKI(0)
            .WithKD(0.7)

            .WithKS(0.6)
            .WithKV(0.25)
            .WithKA(0.15)
            .WithKG(0.45)
            .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine)
        )
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs{}
            .WithMotionMagicCruiseVelocity(5_tps)
            .WithMotionMagicAcceleration(12_tr_per_s_sq)
            .WithMotionMagicJerk(120_tr_per_s_cu)
        )
        .WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs{}
            .WithInverted(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive)
        )
        .WithFeedback(ctre::phoenix6::configs::FeedbackConfigs{}
            .WithFeedbackRemoteSensorID(kCanIDCANCoder)
            .WithFeedbackSensorSource(ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder)
        );


    static constexpr ctre::phoenix6::configs::TalonFXConfiguration KRollerConfigs = ctre::phoenix6::configs::TalonFXConfiguration{}
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs{}
                    
            .WithKP(5)
            .WithKI(0)
            .WithKD(.1)

            .WithKS(0.4)
            .WithKV(0.15)
            .WithKA(0)
            .WithKG(0)
            .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine)
        )
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs{}
            .WithMotionMagicCruiseVelocity(80_tps)
            .WithMotionMagicAcceleration(160_tr_per_s_sq)
            .WithMotionMagicJerk(1600_tr_per_s_cu)
        )
        .WithFeedback(ctre::phoenix6::configs::FeedbackConfigs{}
            .WithSensorToMechanismRatio(1.25)
        );
}

namespace ShooterConstants {
    inline constexpr ctre::phoenix6::CANBus kCanBus{"*"};

    inline constexpr bool InvertFollowDir = true;

    inline constexpr int kCanIDL = 30;

    inline constexpr int kCanIDR = 31;
    inline constexpr units::turns_per_second_t kShooterFlywheelConstant = -55_tps;
    inline constexpr units::turns_per_second_t kShooterFlywheelTolerance = 0.5_tps;
    inline constexpr units::time::second_t kTimeToShootStartFuel = 6_s;

    inline constexpr int kCanIDFeed = 32;
    inline constexpr units::turns_per_second_t kFeedVelocityConstant = -50_tps;

    static constexpr ctre::phoenix6::configs::TalonFXConfiguration KFlywheelL = ctre::phoenix6::configs::TalonFXConfiguration{}
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs{}
                    
            .WithKP(5)
            .WithKI(0)
            .WithKD(0.45)

            .WithKS(0.45)
            .WithKV(0.175)
            .WithKA(0)
            .WithKG(0)
            .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine)
        )
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs{}
            .WithMotionMagicCruiseVelocity(90_tps)
            .WithMotionMagicAcceleration(180_tr_per_s_sq)
            .WithMotionMagicJerk(1800_tr_per_s_cu)
        )
        .WithFeedback(ctre::phoenix6::configs::FeedbackConfigs{}
            .WithSensorToMechanismRatio(1.5)
        );

    static constexpr ctre::phoenix6::configs::TalonFXConfiguration KFlywheelR = ctre::phoenix6::configs::TalonFXConfiguration{}
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs{}
                    
            .WithKP(5)
            .WithKI(0)
            .WithKD(0.45)

            .WithKS(0)
            .WithKV(0.175)
            .WithKA(0)
            .WithKG(0)
            .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine)
        )
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs{}
            .WithMotionMagicCruiseVelocity(90_tps)
            .WithMotionMagicAcceleration(180_tr_per_s_sq)
            .WithMotionMagicJerk(1800_tr_per_s_cu)
        )
        .WithFeedback(ctre::phoenix6::configs::FeedbackConfigs{}
            .WithSensorToMechanismRatio(1.5)
        );

    static constexpr ctre::phoenix6::configs::TalonFXConfiguration KFeedConfigs = ctre::phoenix6::configs::TalonFXConfiguration{}
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs{}
                    
            .WithKP(4)
            .WithKI(0)
            .WithKD(0.1)

            .WithKS(0.5)
            .WithKV(0.2)
            .WithKA(0)
            .WithKG(0)
            .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine)
        )
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs{}
            .WithMotionMagicCruiseVelocity(80_tps)
            .WithMotionMagicAcceleration(160_tr_per_s_sq)
            .WithMotionMagicJerk(1600_tr_per_s_cu)
        )
        .WithFeedback(ctre::phoenix6::configs::FeedbackConfigs{}
            .WithSensorToMechanismRatio(1)
        );

}

namespace ClimberConstants {
    inline constexpr ctre::phoenix6::CANBus kCanBus{"*"};

    inline constexpr int kCanIDOne = 40;
    inline constexpr units::length::inch_t kPivotUpperLimit = 9.25_in;
    inline constexpr units::length::inch_t kPivotLowerLimit = 0.25_in;


    inline constexpr units::unit_t<turns_per_inch> kTurnsPerInch{0.324966};


    static constexpr ctre::phoenix6::configs::TalonFXConfiguration KMotorOneConfigs = ctre::phoenix6::configs::TalonFXConfiguration{}
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs{}
            .WithKS(.12)
            .WithKV(.2)
            .WithKA(.025)

            .WithKP(4)
            .WithKI(0)
            .WithKD(.5)

            .WithKG(.2)
            .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Elevator_Static)
        )
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs{}
            .WithMotionMagicCruiseVelocity(150_tps)
            .WithMotionMagicAcceleration(250_tr_per_s_sq)
            .WithMotionMagicJerk(2500_tr_per_s_cu)
        )
        .WithFeedback(ctre::phoenix6::configs::FeedbackConfigs{}
            .WithSensorToMechanismRatio(15)
        );


}

namespace VisionConstants {

    inline const std::filesystem::path kNodesDirectory = "/VisionNodeConfigs/";
    inline const frc::AprilTagFieldLayout kTagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);

    // Max pose ambiguity (for single tag)
    inline constexpr double kMaxSingleTagAmbiguity = 0.2;
    inline constexpr double kMaxMultiTagAmbiguity = 0.3;

    // Max distance (for single tag)
    inline constexpr units::inch_t kMaxSingleTagDistance = 150_in;
    inline constexpr units::inch_t kMaxMultiTagDistance = 200_in;
  
    // Default standard deviations 
    inline constexpr wpi::array<double, 3U> kSingleTagStdDevs {3.75, 3.75, 7.5};
    inline constexpr wpi::array<double, 3U> kMultiTagStdDevs {0.75, 0.75, 1.5};

    // Standard deviations scale factor limit (scaled as: distance^2 * kStdDevsScaleFactorLimit/(MaxTagDistance^2))
    inline constexpr double kStdDevsScaleFactorLimit = 1.0 / 30.0;

}


