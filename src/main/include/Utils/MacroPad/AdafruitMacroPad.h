#pragma once

#include <frc/GenericHID.h>
#include <wpi/array.h>

    class AdafruitMacroPad : public frc::GenericHID {
        public:
            // Construct an instance of a MacroPad.
            // The port is the port on the Driver Station that the MacroPad is plugged into (0-5).
            AdafruitMacroPad(int port);

            ~AdafruitMacroPad() override = default;

            AdafruitMacroPad(AdafruitMacroPad&&) = default;
            AdafruitMacroPad& operator=(AdafruitMacroPad&&) = default;

            // Get the Knob Rotation current value
            double GetKnobRotation();

            // Get the Knob Button current state
            bool GetKnobButton();

            // Get wether the Knob Button was pressed since the last check
            bool GetKnobButtonPressed();

            // Get wether the Knob Button was released since the last check
            bool GetKnobButtonReleased();
            
            // Get a Keys current state
            // Index by key number
            bool GetKey(int keyNumber);

            // Get a Keys current state
            // Index by (row, col)
            bool GetKey(int row, int col);

            // Get wether a given Key was pressed since the last check
            // Index by key number
            bool GetKeyPressed(int keyNumber);

            // Get wether a given Key was pressed since the last check
            // Index by (row, col)
            bool GetKeyPressed(int row, int col);


            // Get wether a given Key was released since the last check
            // Index by key numbe
            bool GetKeyReleased(int keyNumber);

            // Get wether a given Key was released since the last check
            // Index by (row, col)
            bool GetKeyReleased(int row, int col);

    };
