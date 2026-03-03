#pragma once

#include <frc/geometry/Transform3d.h>

#include <vector>
#include <string>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

class PhotonCamEstimator : public photon::PhotonCamera, public photon::PhotonPoseEstimator {
    public:
        // PhotonCamEstimator constructor
        PhotonCamEstimator(std::string name, frc::AprilTagFieldLayout aprilTags, photon::PoseStrategy poseStrategy, frc::Transform3d robotToCam);

        // Updates PhotonCamEstimator
        // Returns a std::vector of std::optional EstimatedRobotPoses
        std::vector <std::optional<photon::EstimatedRobotPose>> Update();

    private:
        // Define std::vector to hold camera restults
        std::vector <photon::PhotonPipelineResult> m_results;

        // Define std::vector to hold the current estimated robot positions
        std::vector <std::optional<photon::EstimatedRobotPose>> m_robotPositions;
};