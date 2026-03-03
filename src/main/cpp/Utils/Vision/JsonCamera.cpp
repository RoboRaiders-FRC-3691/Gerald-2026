#include "Utils/Vision/JsonCamera.h"

using namespace JsonCamera;

std::unordered_map <std::string, photon::PoseStrategy> strategyMap = {{"AVERAGE_BEST_TARGETS", photon::PoseStrategy::AVERAGE_BEST_TARGETS},
                                                                      {"CLOSEST_TO_CAMERA_HEIGHT", photon::PoseStrategy::CLOSEST_TO_CAMERA_HEIGHT},
                                                                      {"CLOSEST_TO_LAST_POSE", photon::PoseStrategy::CLOSEST_TO_LAST_POSE},
                                                                      {"CLOSEST_TO_REFERENCE_POSE", photon::PoseStrategy::CLOSEST_TO_REFERENCE_POSE},
                                                                      {"LOWEST_AMBIGUITY", photon::PoseStrategy::LOWEST_AMBIGUITY},
                                                                      {"MULTI_TAG_PNP_ON_COPROCESSOR", photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR},
                                                                      {"MULTI_TAG_PNP_ON_RIO", photon::PoseStrategy::MULTI_TAG_PNP_ON_RIO}};


//Takes a filepath to a json file (including file extension) & returns a PhotonCamEstimator object for the camera described in the file.
 PhotonCamEstimator JsonCamera::LoadCameraFromFile(std::filesystem::path filepath, frc::AprilTagFieldLayout aprilTags){
    //Create a filestream
    std::ifstream camStream(filepath);

    //Create a json object
    wpi::json jsonFile;

    //Grab the json file data from the filestream
    jsonFile = wpi::json::parse(camStream);

   camStream.close();

    wpi::json cameraPositionJson = jsonFile.at("PositionOffset");
    // //load the transform 3d for the camera from the json file. (Assume that length units are in inches and rotational units are in degrees)
    frc::Transform3d cameraOffset = frc::Transform3d(units::length::inch_t(cameraPositionJson.at("X")),
                                                     units::length::inch_t(cameraPositionJson.at("Y")),
                                                     units::length::inch_t(cameraPositionJson.at("Z")),
                                    frc::Rotation3d(units::angle::degree_t(cameraPositionJson.at("Roll")),
                                                    units::angle::degree_t(cameraPositionJson.at("Pitch")),
                                                    units::angle::degree_t(cameraPositionJson.at("Yaw"))));                 

    return PhotonCamEstimator(jsonFile.at("CameraName"), aprilTags, strategyMap[jsonFile.at("PoseStrategy")], cameraOffset);
}

std::vector<PhotonCamEstimator> JsonCamera::LoadCamerasFromFolder(std::filesystem::path directoryPath, frc::AprilTagFieldLayout aprilTags){

    //Create a vector for the cameras
    std::vector<PhotonCamEstimator> cameraVect;

    // To check if the directory exists or not 
    if (std::filesystem::exists(directoryPath)
        && std::filesystem::is_directory(directoryPath)) {
        // Loop through each item (file or subdirectory) in 
        // the directory 
        for (const auto& entry :
            std::filesystem::directory_iterator(directoryPath)) {
            //Check if the entry points to a file
            if(entry.is_regular_file()){
                //Create variable for filename
                std::filesystem::path entryFilePath = entry.path();
                cameraVect.push_back(LoadCameraFromFile(entryFilePath, aprilTags));
            }

        }
    }

    return cameraVect;

}
