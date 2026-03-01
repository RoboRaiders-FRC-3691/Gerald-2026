#pragma once

#include <iostream>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>

#include <frc/event/BooleanEvent.h>

#include <frc2/command/CommandScheduler.h>

class DashboardButton{
    public:
        // DashboardButton constructor
        DashboardButton(std::string name);

        // DashboardButton constructor
        DashboardButton(std::string name, std::shared_ptr<nt::NetworkTableInstance> ntInstance);

        // DashboardButton constructor
        DashboardButton(std::string name, std::shared_ptr<nt::NetworkTableInstance> ntInstance,  frc::EventLoop* loop);
    
        // Get the button state as a bool
        bool GetState();

        // Set the button state
        void SetState(bool newState, units::time::microsecond_t time = 0_s);
        
        // Bind action to run when button state transitions from false to true
        void BindRising(std::function<void()> action);

        // Bind action to run when button state transitions from true to false
        void BindFalling(std::function<void()> action);

        // Bind action to run while button state is true
        void BindTrue(std::function<void()> action);

        // Bind action to run while button state is false
        void BindFalse(std::function<void()> action);

    private:
        // Define the button network table
        std::shared_ptr<nt::NetworkTable> m_Table;

        // Define the button bool entry
        nt::BooleanEntry m_ButtonEntry;

        // Define the button loop
        frc::EventLoop* m_loop;

        // Define various button bool events
        frc::BooleanEvent m_MainEvent;
        frc::BooleanEvent m_InvertedEvent;
        frc::BooleanEvent m_RisingEvent;
        frc::BooleanEvent m_FallingEvent;
        
};