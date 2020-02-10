// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <cscore.h>
#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/TimedRobot.h>

#include "Constants.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Intake.hpp"
#include "subsystems/Turret.hpp"

namespace frc3512 {

class Robot : public frc::TimedRobot {
public:
    Robot();

    void DisabledInit() override;
    void AutonomousInit() override;
    void TeleopInit() override;
    void TestInit() override;

    void RobotPeriodic() override;
    void DisabledPeriodic() override;
    void AutonomousPeriodic() override;
    void TeleopPeriodic() override;

private:

    frc::Joystick m_driveStick1{kDriveStick1Port};
    frc::Joystick m_driveStick2{kDriveStick2Port};
    frc::Joystick m_appendageStick{kAppendageStickPort};
    frc::Joystick m_appendageStick2{kAppendageStick2Port};

};

}  // namespace frc3512
