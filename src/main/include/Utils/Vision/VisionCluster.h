#pragma once

#include <units/length.h>

#include <frc/Filesystem.h>
#include <filesystem>
#include <wpi/array.h>

#include <string>
#include <vector>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "Utils/Vision/JsonCamera.h"
#include "Utils/Vision/PhotonCamEstimator.h"

#include <Constants.h>

// Simple constructor to pair an rstimated robot pose and its calculated standard deviations
struct VisionPoseResult {
    photon::EstimatedRobotPose visionEstimate;
    wpi::array<double, 3U> standardDeviations;
};

class VisionCluster{
    public:
        // VisionCluser constructor (Creates a vision cluster by loading cameras from deploy/PhotonCameras/)
        VisionCluster(); // TODO: Add constructor overload or defualt values to enable custom filepaths to cameras directory and alternative field images. (This would increase code reusability)

        // Returns the latest std::vector of vision estimates from the cluster
        std::vector<VisionPoseResult> GetVisionEstimates();

        // Takes an estimated robot pose and filters it returning a vision pose result containing the position, and the position std devs
        std::optional<VisionPoseResult> FilterVisionEstimate(std::optional<photon::EstimatedRobotPose> estimatedPosition);

    private:
        // Define an std::vector to hold the cameras in the cluster
        std::vector<PhotonCamEstimator> m_photonCameras;

        // Define an std::vector of vision estimates
        std::vector<VisionPoseResult> m_visionEst;
};