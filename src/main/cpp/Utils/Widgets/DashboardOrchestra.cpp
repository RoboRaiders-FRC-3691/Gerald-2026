#include <Utils/Widgets/DashboardOrchestra.h>

Widgets::DashboardOrchestra::DashboardOrchestra(std::filesystem::path musicFolder) : ctre::phoenix6::Orchestra(), m_PausePlay("Dashboard-Orchestra/Pause-Play"), m_StopTrack("Dashboard-Orchestra/Stop-Track"){
    
    UpdateTrackList(musicFolder);

}

void Widgets::DashboardOrchestra::UpdateTrackList(std::filesystem::path musicFolder){
    // To check if the directory exists or not 
    if (std::filesystem::exists(musicFolder)
        && std::filesystem::is_directory(musicFolder)) {
        // Loop through each item (file or subdirectory) in 
        // the directory 
        for (const auto& entry :
            std::filesystem::directory_iterator(musicFolder)) {
            // Check if the entry points to a file
            if(entry.is_regular_file()){

                // Loop through the file name and replace any non valid characters with '?'
                std::string trackName = entry.path().stem().string();
                for (char& character : trackName){
                    if(!std::isprint(static_cast<unsigned char>(character))){
                        character = '?';
                        
                        // Print warning/error message
                        std::cout << "Track: " << trackName << " contains invalid characters" << std::endl;
                    }
                }

                // For each track add the name and filepath as an option to the track selector
                m_TrackSelector.AddOption(trackName, entry.path());
                //std::cout << trackName << std::endl; // Name debut statement
            }
        }
    }
}

void Widgets::DashboardOrchestra::InitDashboardInterface(){
    frc::SmartDashboard::PutData("Track Select", &m_TrackSelector);

    m_PausePlay.BindRising([this]{
            ctre::phoenix6::Orchestra::Play();
            std::cout << "Playing Track." << std::endl;
            //Debug Statements
            // if(ctre::phoenix6::Orchestra::IsPlaying()){
            //     std::cout << "Orch is now playing" << std::endl;
            // }
            // else{
            //     std::cout << "Orch isnt actually playing" << std::endl;
            // }
        });


    m_PausePlay.BindFalling([this]{
            //Debug Statements
            // if(ctre::phoenix6::Orchestra::IsPlaying()){
            //     std::cout << "Orch was playing now stopping" << std::endl;
            // }
            // else{
            //     std::cout << "Something went wrong" << std::endl;
            // }

            ctre::phoenix6::Orchestra::Pause();
            std::cout << "Track Paused." << std::endl;
        });
    
    m_StopTrack.BindRising([this]{
            ctre::phoenix6::Orchestra::Stop();
            std::cout << "Track Stopped." << std::endl;
            m_StopTrack.SetState(false);
        });

    m_TrackSelector.OnChange([this](std::filesystem::path trackChoice){
            if(ctre::phoenix6::Orchestra::IsPlaying()){
                m_StopTrack.SetState(false);
                m_PausePlay.SetState(false);
                ctre::phoenix6::Orchestra::Stop();  // "Manually" stop to ensure the track is stopped even if the trigger loop hasnt run yet (May be unneeded and removed later)
            }

            ctre::phoenix6::Orchestra::LoadMusic(trackChoice.generic_string().c_str());

            std::cout << "Track: " << trackChoice.stem() << " has been selected." << std::endl;
        });
}