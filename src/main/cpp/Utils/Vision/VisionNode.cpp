#include "Utils/Vision/VisionNode.h"

VisionNode::VisionNode(std::string cameraName, frc::Transform3d robotToCam, frc::AprilTagFieldLayout tagLayout)
                       : m_Camera(cameraName), m_Estimator(tagLayout, robotToCam), m_ConnectionStatus(true){}

VisionNode VisionNode::FromFile(std::filesystem::path filepath, frc::AprilTagFieldLayout aprilTags){
    std::string fileName = filepath.filename().string();

    try{
        Logging::Debug(fmt::format("[VISION] Loading camera from: {}", filepath.string()));
        std::ifstream camStream(filepath);

        if (!camStream.is_open()) {
            Logging::Error(fmt::format("Could not open camera file stream at {}. Throwing relevant exception", filepath.string()));
            throw std::runtime_error("Could not open camera file stream.");
        }

        wpi::json jsonFile;

        Logging::Debug(fmt::format("[VISION] Parsing JSON format for: {}", fileName));
        jsonFile = wpi::json::parse(camStream);

        camStream.close();

        Logging::Debug(fmt::format("[VISION] Extracting keys from JSON: {}", fileName));
        wpi::json cameraPositionJson = jsonFile.at("PositionOffset");
        
        // load the transform 3d for the camera from the json file. (Assume that length units are in inches and rotational units are in degrees)
        frc::Transform3d cameraOffset = frc::Transform3d(
            units::length::inch_t(cameraPositionJson.at("X")),
            units::length::inch_t(cameraPositionJson.at("Y")),
            units::length::inch_t(cameraPositionJson.at("Z")),
            frc::Rotation3d(
                units::angle::degree_t(cameraPositionJson.at("Roll")),
                units::angle::degree_t(cameraPositionJson.at("Pitch")),
                units::angle::degree_t(cameraPositionJson.at("Yaw"))
            )
        );                 

        std::string camName = jsonFile.at("CameraName");

        Logging::Info(fmt::format("[VISION] Successfully built camera: {}", camName));

        return VisionNode(camName, cameraOffset, aprilTags);

    } catch (const wpi::json::exception& e) {
        // LOG THE ERROR BEFORE KILLING THE ROBOT
        Logging::Error(fmt::format("[FATAL] JSON Error in {}: {}", fileName, e.what()));
        
        // Re-throw so the Subsystem kills the robot boot process
        throw; 
    } catch (const std::exception& e) {
        Logging::Error(fmt::format("[FATAL] Unexpected Error loading {}: {}", fileName, e.what()));
        throw;
    }
}

std::vector<photon::EstimatedRobotPose> VisionNode::GetUnreadMeasurements(){

    std::vector<photon::EstimatedRobotPose> robotPositions;
    
    bool currentlyConnected = m_Camera.IsConnected();

    // State Change: Connected -> Disconnected
    if (!currentlyConnected && m_ConnectionStatus) {
        Logging::Warn(fmt::format("[VISION] Camera '{}' DISCONNECTED! Check power/ethernet.", m_Camera.GetCameraName()));
        m_ConnectionStatus = false;
    } 
    // State Change: Disconnected -> Connected
    else if (currentlyConnected && !m_ConnectionStatus) {
        Logging::Info(fmt::format("[VISION] Camera '{}' RECONNECTED. Systems nominal.", m_Camera.GetCameraName()));
        m_ConnectionStatus = true;
    }

    if (!currentlyConnected) {
        Logging::Debug(fmt::format("[VISION] Camera {} - Disconnected, returning early.", m_Camera.GetCameraName()));
        return robotPositions;
    }

    std::vector<photon::PhotonPipelineResult> cameraResults = m_Camera.GetAllUnreadResults();
         
    for (auto result : cameraResults){
        if(result.HasTargets()){
            std::optional<photon::EstimatedRobotPose> pose = m_Estimator.EstimateCoprocMultiTagPose(result);

            if(!pose.has_value()){
                pose = m_Estimator.EstimateLowestAmbiguityPose(result);
            }

            if(pose.has_value()){
                robotPositions.push_back(pose.value());
                
                Logging::Debug(fmt::format("[VISION] Position Estimate Generated | Camera: {} | Number of Tags: {} | Estimated Position: {}", 
                    m_Camera.GetCameraName(), 
                    pose->targetsUsed.size(),
                    Logging::Pose2dToCustomaryString(pose->estimatedPose.ToPose2d())
                ));
            }
            
        }
    }

    return robotPositions;

}
