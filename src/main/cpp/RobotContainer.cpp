  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

RobotContainer::RobotContainer()
{
    drivetrain.RegisterVisionCallback([this] {return m_vision.GetVisionEstimates();});
    PathPlannerSetUp();
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        })
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    // joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    // joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
    //     return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    // }));
  
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    //(joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    //(joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    //(joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    //(joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    //joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

    // ROBOT NORMAL BINDINGS
    m_XboxController.RightTrigger().OnTrue(m_shooter.RunFlywheel()); // SHOOTS FUEL
    m_XboxController.RightTrigger().OnFalse(m_shooter.SetFlywheelVel(0_tps)); // Set Flywheel 0 
    m_XboxController.RightBumper().WhileTrue(m_shooter.RunFeed()); // RUNS SHOOTER FEED`
    m_XboxController.LeftTrigger().WhileTrue(m_intake.RunIntake()); // INTAKES FUEL
    m_XboxController.LeftBumper().WhileTrue(m_intake.RunIntakeReverse()); // SPITS OUT FUEL FROM INTAKE
    m_XboxController.X().OnTrue(m_intake.DropIntake()); // LOWERS INTAKE
    m_XboxController.Y().OnTrue(m_intake.RaiseIntake()); // RAISES INTAKE
    m_XboxController.POVUp().OnTrue(m_climber.RaiseClimber()); // RAISES CLIMBER 
    m_XboxController.POVDown().OnTrue(m_climber.LowerClimber()); // LOWERS CLIMBER
}

void RobotContainer::PathPlannerSetUp(){

    pathplanner::NamedCommands::registerCommand("Shoot Starting Fuel", m_shooter.ShootFor(ShooterConstants::kTimeToShootStartFuel));
    pathplanner::NamedCommands::registerCommand("Drop Intake", m_intake.DropIntake());
    pathplanner::NamedCommands::registerCommand("Raise Intake", m_intake.RaiseIntake());
    pathplanner::NamedCommands::registerCommand("Climb", m_climber.Climb());
    
    m_AutoChooser = pathplanner::AutoBuilder::buildAutoChooser();

    frc::SmartDashboard::PutData("Auto Selector", &m_AutoChooser);
}

frc2::Command* RobotContainer::GetAutonomousCommand(){
    return m_AutoChooser.GetSelected();
}
