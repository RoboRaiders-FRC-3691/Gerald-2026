#pragma once

#include <iostream>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>

#include <frc2/command/button/Trigger.h>

class CommandDashboardButton{
    public:
        // CommandDashboardButton constructor
        CommandDashboardButton(std::string name);
    
        // Returns a trigger to the dashboard button
        frc2::Trigger GetButton(
                            frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop());

        // Sets the dashboard button state
        void SetState(bool newState, units::time::microsecond_t time = 0_s);

    private:
        // Define the button network table
        std::shared_ptr<nt::NetworkTable> ntTable;

        // Define the button bool entry
        nt::BooleanEntry ButtonState;
};