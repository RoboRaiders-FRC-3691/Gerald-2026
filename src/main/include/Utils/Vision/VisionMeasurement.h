#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/time.h>
#include <wpi/array.h>

struct VisionMeasurement {
    // The field-to-robot pose calculated by the vision system
    frc::Pose2d estimatedPose;
    
    // The FPGA timestamp of when the image was actually taken
    units::second_t timestamp;
    
    // How much we trust this measurement: {X, Y, Theta}
    // Lower numbers mean less deviation so we trust it more.
    wpi::array<double, 3> stdDevs; 
};