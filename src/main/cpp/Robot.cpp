// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

namespace frc3512 {

Robot::Robot() {}

void Robot::DisabledInit() {}

void Robot::AutonomousInit() {}

void Robot::TeleopInit() {}

void Robot::TestInit() {}

void Robot::RobotPeriodic() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousPeriodic() { TeleopPeriodic(); }

void Robot::TeleopPeriodic() {
    if (m_appendageStick.GetRawButton(11)) {
        m_transverser.Set(1.0);
    } else if (m_appendageStick.GetRawButton(12)) {
        m_transverser.Set(-1.0);
    } else {
        m_transverser.Set(0.0);
    }
}

}  // namespace frc3512

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<frc3512::Robot>(); }
#endif
