#include "Utils/Vision/PhotonCamEstimator.h"

PhotonCamEstimator::PhotonCamEstimator(std::string name, frc::AprilTagFieldLayout aprilTags, photon::PoseStrategy poseStrategy, frc::Transform3d robotToCam) : photon::PhotonCamera(name), photon::PhotonPoseEstimator(aprilTags, robotToCam){
    
}

//Updates the PhotonCamEstimtor with the latest results and returns the estimated positions
std::vector <std::optional<photon::EstimatedRobotPose>> PhotonCamEstimator::Update(){

    //Clear the vector so we can replace the values
    m_robotPositions.clear();
    
    m_results = photon::PhotonCamera::GetAllUnreadResults();

    for (auto result : m_results){
        if(result.HasTargets()){
            m_robotPositions.push_back(photon::PhotonPoseEstimator::EstimateCoprocMultiTagPose(result));
        }
    }

    return m_robotPositions;

}


