#pragma once

#include <vector>

#include <photon/PhotonCamera.h>

#include <frc/Filesystem.h>
#include <filesystem>

#include <iostream>
#include <fstream>

#include <wpi/json.h>

#include <units/length.h>
#include <units/angle.h>

#include "Utils/Vision/PhotonCamEstimator.h"

namespace JsonCamera{
    // Returns a PhotonCamEstimator loaded from a file
    PhotonCamEstimator LoadCameraFromFile(std::filesystem::path filepath, frc::AprilTagFieldLayout aprilTags);

    // Returns a std::vector of PhotonCamEstimators loaded from a folder
    std::vector<PhotonCamEstimator> LoadCamerasFromFolder(std::filesystem::path directoryPath, frc::AprilTagFieldLayout aprilTags);
};       