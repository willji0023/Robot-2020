// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <wpi/raw_ostream.h>

namespace frc3512 {

Robot::Robot() : PublishNode("Robot") {
    m_climber.Subscribe(*this);
}

void Robot::DisabledInit() {}

void Robot::AutonomousInit() {}

void Robot::TeleopInit() {}

void Robot::TestInit() {}

void Robot::RobotPeriodic() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousPeriodic() { TeleopPeriodic(); }

void Robot::TeleopPeriodic() {
    auto& ds = frc::DriverStation::GetInstance();
    HIDPacket message{"",
                      m_driveStick1.GetX(),
                      m_driveStick1.GetY(),
                      ds.GetStickButtons(0),
                      m_driveStick2.GetX(),
                      m_driveStick2.GetY(),
                      ds.GetStickButtons(1),
                      m_appendageStick.GetX(),
                      m_appendageStick.GetY(),
                      ds.GetStickButtons(2),
                      m_appendageStick2.GetX(),
                      m_appendageStick2.GetY(),
                      ds.GetStickButtons(3)};
    Publish(message);

    if (message.y3 == 1)
        m_climber.SetElevator(m_appendageStick.GetY());
    if (message.y3 == -1)
        m_climber.SetElevator(m_appendageStick.GetY());
    if (message.x4 == 1)
        m_climber.SetTransverser(m_appendageStick2.GetX());
    if (message.x4 == -1)
        m_climber.SetTransverser(m_appendageStick2.GetX());
}

}  // namespace frc3512

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<frc3512::Robot>(); }
#endif
