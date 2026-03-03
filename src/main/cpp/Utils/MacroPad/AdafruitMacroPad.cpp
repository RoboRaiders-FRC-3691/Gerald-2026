#include "Utils/MacroPad/AdafruitMacroPad.h"

AdafruitMacroPad::AdafruitMacroPad(int port): GenericHID(port){

}

double AdafruitMacroPad::GetKnobRotation(){
    return GetRawAxis(5);
}

bool AdafruitMacroPad::GetKnobButton(){
    return GetRawButton(13);
}

bool AdafruitMacroPad::GetKnobButtonPressed(){
    return GetRawButtonPressed(13);
}

bool AdafruitMacroPad::GetKnobButtonReleased(){
    return GetRawButtonReleased(13);
}

//Gets the key by number        
bool AdafruitMacroPad::GetKey(int keyNumber){
    return GetRawButton(keyNumber);
}
//Gets the key by row and col. The upper left key is (1, 1) lower right key is (4, 3)
bool AdafruitMacroPad::GetKey(int row, int col){
    return GetKey(((row-1)*3) + col);
}

        
bool AdafruitMacroPad::GetKeyPressed(int keyNumber){
    return GetRawButtonPressed(keyNumber);
}

bool AdafruitMacroPad::GetKeyPressed(int row, int col){
    return GetKeyPressed(((row-1)*3) + col);
}

bool AdafruitMacroPad::GetKeyReleased(int keyNumber){
    return GetRawButtonReleased(keyNumber);
}

bool AdafruitMacroPad::GetKeyReleased(int row, int col){
    return GetKeyReleased(((row-1)*3) + col);
}