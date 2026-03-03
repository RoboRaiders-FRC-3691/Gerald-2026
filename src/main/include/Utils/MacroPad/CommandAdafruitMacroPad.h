#pragma once

#include <frc/GenericHID.h>
#include <wpi/array.h>

#include "Utils/MacroPad/AdafruitMacroPad.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/CommandGenericHID.h>

    class CommandAdafruitMacroPad : public frc2::CommandGenericHID {
        public:
            // Construct an instance of a MacroPad.
            // The port is the port on the Driver Station that the MacroPad is plugged into (0-5).
            CommandAdafruitMacroPad(int port);

            // Get a refrence to the underlying GenericHID object.
            AdafruitMacroPad& GetHID();

            // Get the Knob Rotation current value
            double GetKnobRotation();

            // Returns a trigger for the knob button
            frc2::Trigger GetKnobButton(
                                    frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop());
            
            // Returns a trigger for a given key
            // Index by key number
            frc2::Trigger GetKey(int keyNumber, 
                                    frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop());

            // Returns a trigger for a given key
            // Index by (row, col)
            frc2::Trigger GetKey(int row, int col,
                                    frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop());
        private:
            //Underlying macropad object
            AdafruitMacroPad m_hid;

    };
    

