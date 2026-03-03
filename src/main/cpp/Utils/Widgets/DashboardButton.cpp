#include <Utils/Widgets/DashboardButton.h>

DashboardButton::DashboardButton(std::string name) :
    DashboardButton::DashboardButton(name, std::make_shared<nt::NetworkTableInstance>(nt::NetworkTableInstance::GetDefault()), frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()){};

DashboardButton::DashboardButton(std::string name, std::shared_ptr<nt::NetworkTableInstance> ntInstance) :
    DashboardButton::DashboardButton(name, ntInstance, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()){};


DashboardButton::DashboardButton(std::string name, std::shared_ptr<nt::NetworkTableInstance> ntInstance,  frc::EventLoop* loop) : 
    m_Table(ntInstance->GetTable("/Buttons")),
    m_ButtonEntry(m_Table->GetBooleanTopic(name).GetEntry(false)), 
    m_MainEvent(loop, [this] { return GetState(); }), // "Link" to button state
    m_InvertedEvent(!m_MainEvent), // "Link" to opposite of MainEvent
    m_RisingEvent(m_MainEvent.Rising()), // "Link" to MainEvent Rising
    m_FallingEvent(m_MainEvent.Falling()) // "Link" to MainEvent Falling
{
    m_ButtonEntry.SetDefault(false);
}


bool DashboardButton::GetState(){
    return m_ButtonEntry.Get();
}

void DashboardButton::SetState(bool newState, units::time::microsecond_t time){
    m_ButtonEntry.Set(newState, time.value());
    //std::cout << "Tried to set state to: " << newState << std::endl; // Debug Statement
}

void DashboardButton::BindRising(std::function<void()> action){
    m_RisingEvent.IfHigh(action);
}

void DashboardButton::BindFalling(std::function<void()> action){
    m_FallingEvent.IfHigh(action);
}

void DashboardButton::BindTrue(std::function<void()> action){
    m_MainEvent.IfHigh(action);
}

void DashboardButton::BindFalse(std::function<void()> action){
   m_InvertedEvent.IfHigh(action);
}
