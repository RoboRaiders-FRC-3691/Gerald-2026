#include "Utils/MacroPad/CommandAdafruitMacroPad.h"

CommandAdafruitMacroPad::CommandAdafruitMacroPad(int port): CommandGenericHID(port), m_hid{port}{

}

AdafruitMacroPad& CommandAdafruitMacroPad::GetHID(){
    return m_hid;
}

double CommandAdafruitMacroPad::GetKnobRotation(){
    return m_hid.GetKnobRotation();
}

frc2::Trigger CommandAdafruitMacroPad::GetKnobButton(
                                                frc::EventLoop* loop){
    return Button(13, loop); //Use #13 because that is the knob button
}

//Gets the key by number        
frc2::Trigger CommandAdafruitMacroPad::GetKey(int keyNumber,
                                                frc::EventLoop* loop){
    return Button(keyNumber, loop);
}
//Gets the key by row and col. The upper left key is (1, 1) lower right key is (4, 3)
frc2::Trigger CommandAdafruitMacroPad::GetKey(int row, int col,
                                                frc::EventLoop* loop){
    return GetKey((((row-1)*3) + col), loop);
}
