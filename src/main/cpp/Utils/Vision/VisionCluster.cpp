#include "Utils/Vision/VisionCluster.h"

using namespace VisionConstants;

VisionCluster::VisionCluster(){
    std::filesystem::path cameraPath = frc::filesystem::GetDeployDirectory().append("/PhotonCameras/");
    m_photonCameras = JsonCamera::LoadCamerasFromFolder(cameraPath, frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField));
}

std::vector<VisionPoseResult> VisionCluster::GetVisionEstimates(){

    //Make sure the m_visionEst is empty before we start
    m_visionEst.clear();

    //First loop is not range based due to a copy error (Needs to be fixed in the future. I think I need a copy constructor.)
    for (size_t i = 0; i < m_photonCameras.size(); i++){
        //Loop through each estimated position returned from the Update() method
        for (auto& position : m_photonCameras[i].Update()){

                auto filteredEstimate = FilterVisionEstimate(position);

                if(filteredEstimate.has_value()){
                    m_visionEst.push_back(filteredEstimate.value());
                }
            }
        }
    return m_visionEst;  
}

std::optional<VisionPoseResult> VisionCluster::FilterVisionEstimate(std::optional<photon::EstimatedRobotPose> estimatedPosition){
    //Check if the std::optional has a value
    if(estimatedPosition.has_value()){

        photon::EstimatedRobotPose poseVal = estimatedPosition.value();

        wpi::array<double, 3> stdDevs = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};

        //Create a variable to hold the average distance and ambiguity
        units::inch_t avgDistance = 0_in;
        double avgAmbiguity = 0;
                        
        unsigned int numTargets = poseVal.targetsUsed.size();

        //extra statement to ensure we never divide by zero. Shouldnt be a problem as positions shouldnt be estimated without any targets.
        if (numTargets == 0) {
            return std::nullopt;
        }

        for(auto target : poseVal.targetsUsed){
            avgDistance += target.GetBestCameraToTarget().Translation().ToTranslation2d().Norm();
            avgAmbiguity += target.GetPoseAmbiguity();
        }

        avgDistance /= numTargets;
        avgAmbiguity /= numTargets;

        if(numTargets > 1 && avgAmbiguity <= kMaxMultiTagAmbiguity && avgDistance <= kMaxMultiTagDistance){
            stdDevs = kMultiTagStdDevs;

            // Scale standard deviations
            double scaleFactor = (pow(avgDistance.value(), 2) * (kStdDevsScaleFactorLimit/pow(kMaxSingleTagDistance.value(), 2)));
            for(double& stdDev : stdDevs){
                stdDev *= scaleFactor;
            }
        }
        else if(numTargets == 1 && avgAmbiguity <= kMaxSingleTagAmbiguity && avgDistance <= kMaxSingleTagDistance){
            stdDevs = kSingleTagStdDevs;

            // Scale standard deviations
            double scaleFactor = (pow(avgDistance.value(), 2) * (kStdDevsScaleFactorLimit/pow(kMaxSingleTagDistance.value(), 2)));
            for(double& stdDev : stdDevs){
                stdDev *= scaleFactor;
            }
        }
        else {
            // Discard pose estimation otherwise
            return std::nullopt;
        }

        return VisionPoseResult {poseVal, stdDevs};
    }

    return std::nullopt;
}

