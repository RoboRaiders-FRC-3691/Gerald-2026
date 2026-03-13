#pragma once

#include <frc/geometry/Transform3d.h>

#include <vector>
#include <string>

#include <frc/Filesystem.h>
#include <filesystem>

#include <iostream>
#include <fstream>

#include <wpi/json.h>

#include <fmt/core.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include "Utils/Vision/Logging.h"


class VisionNode {
    public:
        VisionNode(std::string cameraName, frc::Transform3d robotToCam, frc::AprilTagFieldLayout tagLayout);
        
        // Takes a filepath to a json file (including file extension) & returns a VisionNode object built from the file
        static VisionNode FromFile(std::filesystem::path filepath, frc::AprilTagFieldLayout aprilTags);

        std::vector<photon::EstimatedRobotPose> GetUnreadMeasurements();

    private:
        photon::PhotonCamera m_Camera;
        photon::PhotonPoseEstimator m_Estimator;

        bool m_ConnectionStatus;
};