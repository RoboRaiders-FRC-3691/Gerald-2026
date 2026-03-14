#include "subsystems/VisionSubsystem.h"

using namespace VisionConstants;

VisionSubsystem::VisionSubsystem(){
    LoadNodesFromDirectory();
}

void VisionSubsystem::Periodic(){

}

std::vector<VisionMeasurement> VisionSubsystem::GetVisionEstimates(){

    std::vector<VisionMeasurement> visionEst;

    for (auto& node : m_Nodes){
        //Loop through each estimated position returned from the Update() method
        for (auto position : node.GetUnreadMeasurements()){

            auto filteredEstimate = FilterVisionEstimate(position);

            if(filteredEstimate.has_value()){
                visionEst.push_back(filteredEstimate.value());
            }
        }
    }

    return visionEst; 
}

void VisionSubsystem::LoadNodesFromDirectory() {

    std::filesystem::path nodeConfigsFullDirectory = frc::filesystem::GetDeployDirectory().append(kNodesDirectory);

    // 1. Check if the directory even exists
    if (!std::filesystem::exists(nodeConfigsFullDirectory)) {
        Logging::Error(fmt::format("[VISION] Directory NOT FOUND: {}. No cameras will be loaded.", 
            nodeConfigsFullDirectory.string()));
        return; 
    }

    if (!std::filesystem::is_directory(nodeConfigsFullDirectory)) {
        Logging::Error(fmt::format("[VISION] Path {} is not a directory! No cameras will be loaded.", 
            nodeConfigsFullDirectory.string()));
        return;
    }

    // 2. Iterate through files
    Logging::Info(fmt::format("[VISION] Searching for camera configs in: {}", nodeConfigsFullDirectory.string()));

    for (const auto& entry : std::filesystem::directory_iterator(nodeConfigsFullDirectory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") {
            try {
                m_Nodes.push_back(VisionNode::FromFile(entry.path(), kTagLayout));
            } catch (const std::exception& e) {
                // If one camera file is corrupt, we log it and keep trying the others
                Logging::Error(fmt::format("[VISION] Failed to load node from {}: {}", 
                    entry.path().filename().string(), e.what()));
            }
        }
    }

    // 3. Final Summary
    if (m_Nodes.empty()) {
        Logging::Warn("[VISION] Directory scan complete: 0 vision nodes created. Check file extensions and internal formatting!");
    } else {
        Logging::Info(fmt::format("[VISION] Successfully initialized {} camera nodes.", m_Nodes.size()));
    }
}

std::optional<VisionMeasurement> VisionSubsystem::FilterVisionEstimate(photon::EstimatedRobotPose estimatedRobotPosition){

        wpi::array<double, 3> stdDevs = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};

        //Create a variable to hold the average distance and ambiguity
        units::inch_t avgDistance = 0_in;
        double avgAmbiguity = 0;
                        
        unsigned int numTargets = estimatedRobotPosition.targetsUsed.size();

        // extra statement to ensure we never divide by zero. Shouldnt be a problem as positions shouldnt be estimated without any targets.
        if (numTargets == 0) {
            return std::nullopt;
        }

        for(auto target : estimatedRobotPosition.targetsUsed){
            avgDistance += target.GetBestCameraToTarget().Translation().ToTranslation2d().Norm();
            avgAmbiguity += target.GetPoseAmbiguity();
        }

        avgDistance /= numTargets;
        avgAmbiguity /= numTargets;

        bool accepted = false;

        if(numTargets > 1 && avgAmbiguity <= kMaxMultiTagAmbiguity && avgDistance <= kMaxMultiTagDistance) {
            stdDevs = kMultiTagStdDevs;
            accepted = true;
        } 
        else if(numTargets == 1 && avgAmbiguity <= kMaxSingleTagAmbiguity && avgDistance <= kMaxSingleTagDistance) {
            stdDevs = kSingleTagStdDevs;
            accepted = true;
        }

        if (accepted) {
            double scaleFactor = (pow(avgDistance.value(), 2) * (kStdDevsScaleFactorLimit/pow(kMaxSingleTagDistance.value(), 2)));
            for(double& stdDev : stdDevs) { stdDev *= scaleFactor; }

            return VisionMeasurement {estimatedRobotPosition.estimatedPose.ToPose2d(), estimatedRobotPosition.timestamp, stdDevs};
        } else {
            Logging::Debug(fmt::format("[VISION] Filter Rejected Estimate | Tags: {} | AvgDist: {:.1f}in | AvgAmbiguity: {:.2f}", 
                numTargets, avgDistance.value(), avgAmbiguity));
            
            return std::nullopt;
        }
}
