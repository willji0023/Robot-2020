// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"

namespace frc3512 {

class Robot : public frc::TimedRobot, public PublishNode {
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
    frc::Joystick m_driveStick1{Constants::Robot::kDriveStick1Port};
    frc::Joystick m_driveStick2{Constants::Robot::kDriveStick2Port};
    frc::Joystick m_appendageStick{Constants::Robot::kAppendageStickPort};
    frc::Joystick m_appendageStick2{Constants::Robot::kAppendageStick2Port};
};

}  // namespace frc3512
