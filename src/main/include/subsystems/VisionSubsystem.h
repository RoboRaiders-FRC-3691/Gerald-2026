#pragma once

#include <vector>
#include <frc2/command/SubsystemBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include "Utils/Vision/VisionNode.h"
#include "Utils/Vision/VisionMeasurement.h"

#include "Constants.h"

class VisionSubsystem : public frc2::SubsystemBase {
public:
    VisionSubsystem();

    void Periodic() override;

    std::vector<VisionMeasurement> GetVisionEstimates();

private:
    void LoadNodesFromDirectory();

    std::optional<VisionMeasurement> FilterVisionEstimate(photon::EstimatedRobotPose estimatedRobotPosition);

private:
    std::vector<VisionNode> m_Nodes;
};