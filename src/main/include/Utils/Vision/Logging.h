#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <units/length.h>
#include <units/angle.h>
#include <fmt/core.h>
#include <string_view>

#include <frc/Errors.h>

// NOTE: We are using simple functions instead of macros for a cleaner API.
// NOTE: if constexpr is used to strip debug logs at compile-time for performance while maintaining clarity.

namespace Logging {
    // Logging debug flag confused?
    // kIsDebugMode bool present for clarity. Minimal overhead overlooked.
    inline constexpr bool kIsDebugMode = 
    #ifndef NDEBUG
        true;
    #else
        false;
    #endif

    inline void Debug(std::string message) {
        if constexpr (kIsDebugMode) {
            frc::DataLogManager::Log("Debug: " + message);
        }
    }

    inline void Info(std::string message) {
        frc::DataLogManager::Log("Info: " + message);
    }

    inline void Warn(std::string message) {
        frc::DataLogManager::Log("WARNING: " + message);
    }

    inline void Error(std::string message) {
        frc::DataLogManager::Log("ERROR: " + message);
    }

    // Additional Logging Helpers
    inline std::string Pose2dToCustomaryString(const frc::Pose2d& pose) {
        return fmt::format("X: {:.2f}in, Y: {:.2f}in, Rot: {:.1f}deg", 
            units::inch_t{pose.X()}.value(),
            units::inch_t{pose.Y()}.value(),
            pose.Rotation().Degrees().value()
        );
    }
}