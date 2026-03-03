#include <Utils/Widgets/CommandDashboardButton.h>

CommandDashboardButton::CommandDashboardButton(std::string name) : ntTable(nt::NetworkTableInstance::GetDefault().GetTable("/Dashboard-Widgets")), ButtonState(ntTable->GetBooleanTopic(name).GetEntry(false)){
    ButtonState.SetDefault(false);
}


frc2::Trigger CommandDashboardButton::GetButton(frc::EventLoop* loop){
    return frc2::Trigger(loop, [this] { return ButtonState.Get(); });
}

void CommandDashboardButton::SetState(bool newState, units::time::microsecond_t time){
    ButtonState.Set(newState, time.value());
    //std::cout << "Tried to set state to: " << newState << std::endl; // Debug Statement
}