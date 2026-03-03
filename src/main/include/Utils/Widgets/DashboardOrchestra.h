#pragma once

#include <memory>

#include <iostream>

#include <filesystem>
#include <frc/Filesystem.h>

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>

#include "ctre/phoenix6/Orchestra.hpp"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Utils/Widgets/DashboardButton.h"

namespace Widgets{
    class DashboardOrchestra : public ctre::phoenix6::Orchestra {
        public:
            // DashboardOrchestra constructor
            DashboardOrchestra(std::filesystem::path musicFolder);

            // Updates the track list from the music folder
            void UpdateTrackList(std::filesystem::path musicFolder);

            // Initializes the Orchestra interface on the dashboard over networktables
            void InitDashboardInterface();

        private:
            // Define a dashboard button for pause/play 
            DashboardButton m_PausePlay;

            // Define a dashboard button for stop/reset
            DashboardButton m_StopTrack;

            // Define a dashboard chooser for track selection
            frc::SendableChooser<std::filesystem::path> m_TrackSelector;
    };
}